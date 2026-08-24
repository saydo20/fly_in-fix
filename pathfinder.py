import heapq
from typing import Dict, List, Optional, Tuple
from models import Graph, Zone


def _dijkstra_dists(graph: Graph, start: Zone) -> Tuple[Dict[Zone, float], Dict[Zone, float], Dict[Zone, Optional[Zone]]]:
    """
    Run Dijkstra from `start` without any capacity constraints.

    Returns
    -------
    dist  : dist[v]  = minimum cost of going from start to v
            (paying destination zone costs; start itself is not paid for)
    score : score[v] = secondary score — number of non-priority hops from start to v
            (lower = more priority zones used = preferred)
    prev  : prev[v]  = predecessor of v on the cheapest path from start

    Blocked zones (cost = None) are never visited.
    """
    INF = float("inf")
    dist: Dict[Zone, float] = {z: INF for z in graph.zones.values()}
    score: Dict[Zone, float] = {z: INF for z in graph.zones.values()}
    prev: Dict[Zone, Optional[Zone]] = {z: None for z in graph.zones.values()}
    visited: Dict[Zone, bool] = {z: False for z in graph.zones.values()}

    dist[start] = 0
    score[start] = 0
    heap = [(0, 0, start.name, start)]

    while heap:
        d, s, _, u = heapq.heappop(heap)
        if visited[u]:
            continue
        visited[u] = True

        for connection in graph.adjacency[u]:
            v = connection.destination
            if v.cost is None or visited[v]:
                continue
            new_cost = d + v.cost
            new_score = s + (0 if v.type == "priority" else 1)
            if (new_cost, new_score) < (dist[v], score[v]):
                dist[v] = new_cost
                score[v] = new_score
                prev[v] = u
                heapq.heappush(heap, (new_cost, new_score, v.name, v))

    return dist, score, prev


def dijkstra(graph: Graph, start: Optional[Zone] = None) -> List[Zone]:
    """
    Find the shortest path from `start` (default: graph.start) to graph.end.

    Cost model
    ----------
    - Blocked zones (cost = None) are never visited.
    - Normal / priority zones: cost 1 per zone.
    - Restricted zones: cost 2 per zone (drone spends an extra turn to cross).
    - Priority zones are preferred when two paths have equal total cost (lower
      secondary score = more priority zones used).

    Returns
    -------
    List[Zone] from start to end (inclusive).

    Raises
    ------
    ValueError if no path exists.
    """
    origin: Zone = start if start is not None else graph.start
    dist, _, prev = _dijkstra_dists(graph, origin)

    if dist[graph.end] == float("inf"):
        raise ValueError("No path from start to end exists.")
    path: List[Zone] = []
    current: Optional[Zone] = graph.end
    while current is not None:
        path.append(current)
        current = prev[current]
    path.reverse()
    return path


class Pathfinder:
    """
    Per-drone next-step advisor with automatic load balancing.

    Core idea: reverse Dijkstra
    ---------------------------
    We run Dijkstra once backward from graph.end (treated as the origin).
    The result `_rev_dist[v]` is the cost of travelling from zone v to the
    end hub (i.e. it equals the sum of zone costs v, v+1, …, end along the
    cheapest path in reverse, which equals the forward path cost).

    For any drone at zone Z, we want the neighbour V that minimises:
        total forward cost through V
            = V.cost + (cost from V onward to end)

    It can be shown that comparing  _rev_dist[V]  is equivalent to comparing
    this total cost (the end hub's own cost is a constant addend).

    Load balancing at equal-cost branch points
    ------------------------------------------
    When two or more neighbours share the same (rev_dist, rev_score) — meaning
    they lead to the end with equal total cost — we pick the LEAST LOADED one:

        load(V) = V.occupancy + V.in_transit_count

    This spreads drones across parallel routes of equal quality (e.g. the three
    independent restricted-zone chains in the challenger map) without ever
    routing through a longer detour.  For single-path segments the behaviour is
    identical to a static shortest path.
    """

    def __init__(self, graph: Graph) -> None:
        self.graph = graph
        rev_dist, rev_score, _ = _dijkstra_dists(graph, graph.end)
        self._rev_dist: Dict[Zone, float] = rev_dist
        self._rev_score: Dict[Zone, float] = rev_score

    def find_path(self) -> List[Zone]:
        return dijkstra(self.graph)

    def find_next_step(self, from_zone: Zone) -> Optional[Zone]:
        if from_zone is self.graph.end:
            return None
        INF = float("inf")
        best_metric: Tuple[float, float] = (INF, INF)
        candidates: List[Zone] = []
        for connection in self.graph.adjacency[from_zone]:
            v = connection.destination
            if v.cost is None:
                continue
            rev_d = self._rev_dist.get(v, INF)
            rev_s = self._rev_score.get(v, INF)
            if rev_d == INF:
                continue
            metric = (rev_d, rev_s)
            if metric < best_metric:
                best_metric = metric
                candidates = [v]
            elif metric == best_metric:
                candidates.append(v)
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda z: (z.occupancy + z.in_transit_count, z.name),
        )
