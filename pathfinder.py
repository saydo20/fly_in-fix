import heapq
from typing import Dict, List, Optional, Tuple, Callable
from models import Graph, Zone


class CapacityLedger:
    """Track residual capacities of graph zones and links during routing."""

    def __init__(self) -> None:
        """Initialize empty capacity mappings for zones and links."""
        self.zone_remaining: Dict[Zone, float] = {}
        self.link_remaining: Dict[frozenset, float] = {}

    def build(self, graph: Graph) -> None:
        """Populate available capacities from graph zones and connections.

        Args:
            graph: Graph containing zones and adjacency connections.
        """
        for zone in graph.zones.values():
            if zone.is_start or zone.is_end:
                self.zone_remaining[zone] = float("inf")
            else:
                self.zone_remaining[zone] = zone.max_drones

        for zone in graph.zones.values():
            for connection in graph.adjacency[zone]:
                key = frozenset({zone.name, connection.destination.name})
                if key not in self.link_remaining:
                    self.link_remaining[key] = connection.max_link_capacity

    def can_traverse(self, zone_a: Zone, zone_b: Zone) -> bool:
        """Check if both target zone and link have remaining capacity.

        Args:
            zone_a: Source zone.
            zone_b: Destination zone.

        Returns:
            True if both zone and link have remaining capacity.
        """
        zone_ok = self.zone_remaining[zone_b] > 0

        key = frozenset({zone_a.name, zone_b.name})
        link_ok = self.link_remaining[key] > 0
        return zone_ok and link_ok

    def path_bottleneck(self, path: List[Zone]) -> float:
        """Find minimum remaining capacity along a path.

        Args:
            path: Ordered list of Zone nodes forming a path.

        Returns:
            Bottleneck capacity value as a float.
        """
        smallest = float("inf")
        for i in range(len(path) - 1):
            zone = path[i]
            if not (zone.is_start or zone.is_end):
                smallest = min(smallest, self.zone_remaining[zone])
            key = frozenset({path[i].name, path[i + 1].name})
            smallest = min(smallest, self.link_remaining[key])
        return smallest

    def consume(self, path: List[Zone], amount: float) -> None:
        """Deduct path traversal capacity from the ledger.

        Args:
            path: Ordered list of Zone nodes along the route.
            amount: Capacity amount to deduct.
        """
        for i in range(len(path) - 1):
            zone = path[i]
            if not (zone.is_start or zone.is_end):
                self.zone_remaining[zone] -= amount
            key = frozenset({path[i].name, path[i + 1].name})
            self.link_remaining[key] -= amount


def dijkstra(graph: Graph, can_traverse: Callable = None
             ) -> Optional[Tuple[List[Zone], float]]:
    """Find shortest path from start to end with priority zone preference.

    Uses a min-heap evaluating (movement_cost, priority_penalty) to prioritize
    priority zones in case of equal movement costs.

    Args:
        graph: Graph topology to search.
        can_traverse: Optional callable to check capacity along edges.

    Returns:
        Tuple containing the path list and total cost, or None if unreachable.
    """
    origin = graph.start
    if origin.cost is None:
        return None

    dist: Dict[Zone, float] = {zone: float("inf")
                               for zone in graph.zones.values()}
    priority_score: Dict[Zone, float] = {zone: float("inf")
                                         for zone in graph.zones.values()}
    previous: Dict[Zone, Optional[Zone]] = {zone: None
                                            for zone in graph.zones.values()}
    visited: Dict[Zone, bool] = {zone: False for zone in graph.zones.values()}

    dist[origin] = 0
    priority_score[origin] = 0
    pq = [(0, 0, origin.name, origin)]

    while pq:
        d, p, _, u = heapq.heappop(pq)
        if visited[u]:
            continue
        visited[u] = True
        if u is graph.end:
            break

        for connection in graph.adjacency[u]:
            v = connection.destination
            weight = v.cost
            if weight is None:
                continue
            if visited[v]:
                continue
            if can_traverse is not None and not can_traverse(u, v):
                continue

            new_cost = d + weight
            new_score = p + (0 if v.type == "priority" else 1)
            if (new_cost, new_score) < (dist[v], priority_score[v]):
                dist[v] = new_cost
                priority_score[v] = new_score
                previous[v] = u
                heapq.heappush(pq, (new_cost, new_score, v.name, v))

    if dist[graph.end] == float("inf"):
        return None

    path: List[Zone] = []
    current: Optional[Zone] = graph.end
    while current is not None:
        path.append(current)
        current = previous[current]
    path.reverse()

    return path, dist[graph.end]


class RoutePlanner:
    """Discover non-conflicting paths and assign drones across them."""

    def discover_paths(self, graph: Graph, nb_drones: int
                       ) -> List[Tuple[List[Zone], float, float]]:
        """Find augmenting paths until total capacity accommodates all drones.

        Args:
            graph: Graph network to navigate.
            nb_drones: Total number of drones to route.

        Returns:
            List of tuples containing (path, cost, bottleneck_capacity).

        Raises:
            ValueError: If no valid route exists between start and end hubs.
        """
        ledger = CapacityLedger()
        ledger.build(graph)

        found_paths: List[Tuple[List[Zone], float, float]] = []
        total_capacity = 0.0
        iterations = 0

        while True:
            iterations += 1
            result = dijkstra(graph, can_traverse=ledger.can_traverse)

            if result is None:
                break

            path, cost = result
            bottleneck = ledger.path_bottleneck(path)
            ledger.consume(path, bottleneck)

            found_paths.append((path, cost, bottleneck))
            total_capacity += bottleneck

            if total_capacity >= nb_drones:
                break

        if not found_paths:
            raise ValueError("no route exists between start and end")

        return found_paths

    def assign_drones(self, found_paths, nb_drones: int) -> List[List[Zone]]:
        """Allocate drones across discovered paths proportional to capacity.

        Args:
            found_paths: List of discovered paths with costs and bottlenecks.
            nb_drones: Total count of drones to allocate.

        Returns:
            List of paths assigned to each drone.

        Raises:
            ValueError: If total available capacity is zero.
        """
        sorted_paths = sorted(found_paths, key=lambda entry: entry[1])
        total_capacity = sum(bottleneck for _, _, bottleneck in sorted_paths)

        if total_capacity <= 0:
            raise ValueError("no capacity available")

        quotas = [int(nb_drones * bottleneck / total_capacity)
                  for _, _, bottleneck in sorted_paths]
        remainder = nb_drones - sum(quotas)

        i = 0
        while remainder > 0:
            quotas[i % len(quotas)] += 1
            remainder -= 1
            i += 1

        assignments: List[List[Zone]] = []
        quotas_left = quotas.copy()

        while len(assignments) < nb_drones:
            for idx, (path, _, _) in enumerate(sorted_paths):
                if quotas_left[idx] > 0:
                    assignments.append(list(path))
                    quotas_left[idx] -= 1

        return assignments
