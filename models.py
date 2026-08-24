from enum import Enum
from typing import Dict, List, Optional, Tuple


class ParseError(Exception):
    def __init__(self, line: Optional[int], message: str) -> None:
        self.line = line
        self.message = message
        if line is not None:
            super().__init__(f"Line {line}: {message}")
        else:
            super().__init__(message)


class LineType(Enum):
    DRONE_COUNT = "nb_drones"
    START_HUB = "start_hub"
    END_HUB = "end_hub"
    HUB = "hub"
    CONNECTION = "connection"


class Zone:
    """
    A node in the airspace graph.

    Attributes
    ----------
    name             : unique zone identifier
    x_y              : (x, y) grid coordinates (informational only)
    is_start         : True for the departure hub
    is_end           : True for the arrival hub
    type             : one of  normal | restricted | priority | blocked
    color            : display color (informational only)
    max_drones       : max simultaneous occupants; None means unlimited
    occupancy        : drones physically present right now
    in_transit_count : drones committed to arrive here next turn (reserved spots)
    cost             : movement cost in turns (1, 2, or None for blocked)
    """

    def __init__(
        self,
        name: str,
        x_y: Tuple[int, int],
        line_type: LineType,
        metadata: Dict[str, object],
    ) -> None:
        self.name = name
        self.x_y = x_y
        self.is_start = line_type == LineType.START_HUB
        self.is_end = line_type == LineType.END_HUB
        self.type: str = metadata["zone"]
        self.color = metadata["color"]

        # Start and end hubs have unlimited capacity (no drone cap).
        self.max_drones: Optional[int] = (
            None if (self.is_start or self.is_end) else metadata["max_drones"]
        )

        self.occupancy: int = 0
        # Drones currently crossing *into* this zone (restricted 2-turn rule).
        # They count toward capacity so no other drone over-fills the zone.
        self.in_transit_count: int = 0

        # Movement cost per turn:
        #   normal / priority  → 1 turn   (cost = 1)
        #   restricted         → 2 turns  (cost = 2)
        #   blocked            → cannot enter (cost = None)
        if self.type == "blocked":
            self.cost: Optional[int] = None
        elif self.type == "restricted":
            self.cost = 2
        else:
            self.cost = 1

    def has_capacity(self) -> bool:
        """
        Return True if at least one more drone can commit to entering this zone.
        Both physically present drones and in-transit drones count against the cap.
        """
        if self.max_drones is None:
            return True  # Unlimited (start / end hubs).
        return (self.occupancy + self.in_transit_count) < self.max_drones

    def __repr__(self) -> str:
        return f"Zone({self.name!r}, type={self.type})"


class Connection:
    """
    A directed half-edge in the adjacency list.
    Each bidirectional link is stored as two Connection objects (one per direction).

    Attributes
    ----------
    destination       : the zone this edge leads to
    occupancy         : drones using this link during the *current* turn
    max_link_capacity : maximum drones allowed on this link per turn
    """

    def __init__(self, destination: Zone, max_link_capacity: int = 1) -> None:
        self.destination = destination
        self.occupancy: int = 0
        self.max_link_capacity = max_link_capacity

    def is_movable(self) -> bool:
        """Return True if this link can still accept one more drone this turn."""
        return self.occupancy < self.max_link_capacity

    def __repr__(self) -> str:
        return (
            f"Connection(-> {self.destination.name}, cap={self.max_link_capacity})"
        )


class Graph:
    """Weighted, undirected airspace graph."""

    def __init__(self) -> None:
        self.zones: Dict[str, Zone] = {}
        self.adjacency: Dict[Zone, List[Connection]] = {}
        self.start: Optional[Zone] = None
        self.end: Optional[Zone] = None

    # ── Build ────────────────────────────────────────────────────────────────

    def add_zone(
        self, zone: Zone, line_number: int, line_type: LineType
    ) -> None:
        """Register a zone; raises ParseError on duplicates or multiple hubs."""
        if zone.name in self.zones:
            raise ParseError(line_number, f'Zone "{zone.name}" already exists.')
        self.zones[zone.name] = zone
        self.adjacency[zone] = []

        if line_type == LineType.START_HUB:
            if self.start is not None:
                raise ParseError(line_number, "Only one start_hub is allowed.")
            self.start = zone
        elif line_type == LineType.END_HUB:
            if self.end is not None:
                raise ParseError(line_number, "Only one end_hub is allowed.")
            self.end = zone

    def add_connection(
        self,
        zone1: Zone,
        zone2: Zone,
        line_number: int,
        max_link_capacity: int = 1,
    ) -> None:
        """Add a bidirectional link; raises ParseError on duplicates."""
        for connection in self.adjacency[zone1]:
            if connection.destination is zone2:
                raise ParseError(
                    line_number,
                    f'Connection "{zone1.name}-{zone2.name}" already exists.',
                )
        self.adjacency[zone1].append(Connection(zone2, max_link_capacity))
        self.adjacency[zone2].append(Connection(zone1, max_link_capacity))

    def validate(self) -> None:
        """Raise ParseError if start_hub or end_hub is missing."""
        if self.start is None:
            raise ParseError(None, "no start_hub zone was defined in the file.")
        if self.end is None:
            raise ParseError(None, "no end_hub zone was defined in the file.")

    # ── Queries ──────────────────────────────────────────────────────────────

    def get_neighbors(self, zone: Zone) -> List[Zone]:
        """Return all zones directly connected to `zone`."""
        return [c.destination for c in self.adjacency[zone]]

    def get_connection(self, zone1: Zone, zone2: Zone) -> Connection:
        """Return the Connection from zone1 to zone2; raises ValueError if absent."""
        for connection in self.adjacency[zone1]:
            if connection.destination is zone2:
                return connection
        raise ValueError(
            f"No connection between '{zone1.name}' and '{zone2.name}'."
        )

    def can_move(self, from_zone: Zone, to_zone: Zone) -> bool:
        """Return True if both the link and the destination zone have capacity."""
        connection = self.get_connection(from_zone, to_zone)
        return connection.is_movable() and to_zone.has_capacity()

    # ── Debug ────────────────────────────────────────────────────────────────

    def print_graph(self) -> None:
        for name, zone in self.zones.items():
            print(f"{name} => type: {zone.type}, color: {zone.color}")
        print()
        for zone, connections in self.adjacency.items():
            neighbors = ", ".join(c.destination.name for c in connections)
            print(f"{zone.name} => {neighbors}")


class Drone:
    """
    Represents a single drone in the simulation.

    State
    -----
    id            : unique identifier (e.g. "D1")
    current_zone  : zone the drone currently occupies
    delivered     : True once the drone reaches end_hub
    in_transit    : True while the drone is crossing a restricted zone (2-turn move)
    transit_from  : source zone of the ongoing restricted-zone crossing
    transit_to    : destination zone of the ongoing restricted-zone crossing
    """

    def __init__(self, drone_id: str, start_zone: Zone) -> None:
        self.id = drone_id
        self.current_zone: Zone = start_zone
        self.delivered: bool = False
        self.in_transit: bool = False
        self.transit_from: Optional[Zone] = None
        self.transit_to: Optional[Zone] = None

    def __repr__(self) -> str:
        status = "delivered" if self.delivered else self.current_zone.name
        return f"Drone({self.id}, {status})"
