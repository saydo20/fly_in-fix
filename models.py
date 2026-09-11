from enum import Enum
from typing import Dict, List, Optional, Tuple, Union


class ParseError(Exception):
    """Custom exception raised for syntax and semantic map parsing errors."""

    def __init__(self, line: Optional[int], message: str) -> None:
        """Initialize a parsing error with line number and message.

        Args:
            line: 1-indexed line number where the error occurred, if known.
            message: Descriptive error message.
        """
        self.line = line
        self.message = message
        if line is not None:
            super().__init__(f"Line {line}: {message}")
        else:
            super().__init__(message)


class LineType(Enum):
    """Supported line prefixes in map configuration files."""

    DRONE_COUNT = "nb_drones:"
    START_HUB = "start_hub:"
    END_HUB = "end_hub:"
    HUB = "hub:"
    CONNECTION = "connection:"


class Zone:
    """Represent a graph zone vertex and its operational constraints."""

    def __init__(
            self, name: str, x_y: Tuple[int, int],
            line_type: LineType, metadata: Dict[str, object]
    ) -> None:
        """Initialize a zone with coordinates, type, and capacity metadata.

        Args:
            name: Unique identifier for the zone.
            x_y: Coordinates tuple (x, y).
            line_type: Category prefix defining whether it is start,
                hub, or end.
            metadata: Dictionary containing parsed attributes (zone, color,
                max_drones).
        """
        self.name = name
        self.x_y = x_y
        self.line_type = line_type
        self.is_start = line_type == LineType.START_HUB
        self.is_end = line_type == LineType.END_HUB

        type_val = metadata.get("zone")
        self.type: Optional[str] = (
            str(type_val) if type_val is not None else None
        )

        color_val = metadata.get("color")
        self.color: Optional[str] = (
            str(color_val) if color_val is not None else None
        )

        self.max_drones: Union[int, float] = (
            float('inf') if (self.is_start or self.is_end)
            else int(str(metadata["max_drones"]))
        )
        self.occupancy = 0
        self.in_transit_count = 0
        self.cost: Optional[int] = 1

        if self.type == "restricted":
            self.cost = 2
        elif self.type == "blocked":
            self.cost = None

    def has_capacity(self) -> bool:
        """Check whether the zone can accommodate an additional drone.

        Returns:
            True if capacity is available or if zone is start/end hub.
        """
        if self.is_start or self.is_end:
            return True

        return (self.occupancy + self.in_transit_count) < self.max_drones


class Connection:
    """Represent an edge between adjacent zones and its capacity limit."""

    def __init__(self, destination: Zone, max_link_capacity: int) -> None:
        """Initialize a connection to a destination zone.

        Args:
            destination: Target Zone node reached through this link.
            max_link_capacity: Maximum concurrent drone traversals allowed.
        """
        self.destination = destination
        self.occupancy = 0
        self.max_link_capacity = max_link_capacity

    def is_movable(self) -> bool:
        """Check whether the link capacity allows another traversal.

        Returns:
            True if current occupancy is below maximum capacity.
        """
        return self.occupancy < self.max_link_capacity


class Graph:
    """Represent the network topology of connected zones and pathways."""

    def __init__(self) -> None:
        """Initialize an empty graph with zones and adjacency mappings."""
        self.zones: Dict[str, Zone] = {}
        self.adjacency: Dict[Zone, List[Connection]] = {}
        self.start: Optional[Zone] = None
        self.end: Optional[Zone] = None

    def add_zone(
            self, zone: Zone, line_number: int, line_type: LineType
    ) -> None:
        """Add a new zone to the graph and record start or end references.

        Args:
            zone: The Zone instance to register.
            line_number: Line number for error reporting.
            line_type: Category prefix of the zone.

        Raises:
            ParseError: If zone name already exists or multiple start/end hubs.
        """
        if zone.name in self.zones:
            raise ParseError(
                line_number, f'Zone "{zone.name}" already exists.'
            )
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
            max_link_capacity: int
    ) -> None:
        """Add a bidirectional connection between two zones.

        Args:
            zone1: First endpoint zone.
            zone2: Second endpoint zone.
            line_number: Line number for error reporting.
            max_link_capacity: Maximum concurrent capacity for this link.

        Raises:
            ParseError: If connection already exists.
        """
        for connection in self.adjacency[zone1]:
            if connection.destination is zone2:
                raise ParseError(
                    line_number,
                    f'Connection "{zone1.name}-{zone2.name}" already exists.'
                )

        self.adjacency[zone1].append(Connection(zone2, max_link_capacity))
        self.adjacency[zone2].append(Connection(zone1, max_link_capacity))

    def validate(self) -> None:
        """Validate that both a start_hub and an end_hub are defined.

        Raises:
            ParseError: If start or end hub is missing.
        """
        if self.start is None:
            raise ParseError(None,
                             "no start_hub zone was defined in the file.")
        if self.end is None:
            raise ParseError(None, "no end_hub zone was defined in the file.")

    def get_connection(self, zone1: Zone, zone2: Zone) -> Connection:
        """Retrieve the connection object linking zone1 to zone2.

        Args:
            zone1: Source zone.
            zone2: Destination zone.

        Returns:
            The Connection object leading to zone2.

        Raises:
            ValueError: If no connection exists between the two zones.
        """
        for connection in self.adjacency[zone1]:
            if connection.destination == zone2:
                return connection
        raise ValueError(
            f"no connection between '{zone1.name}' and '{zone2.name}'"
        )

    def get_zone_colors(self) -> Dict[str, str]:
        """Retrieve mapping of zone names to their declared colors.

        Returns:
            Dictionary mapping zone names to color strings.
        """
        return {
            zone_name: zone.color for zone_name, zone in self.zones.items()
            if zone.color is not None
        }


class Drone:
    """Represent a drone navigating through an assigned path."""

    def __init__(self, drone_id: str, path: List[Zone]) -> None:
        """Initialize a drone with a unique ID and planned route.

        Args:
            drone_id: Unique drone identifier string (e.g., 'D1').
            path: List of Zone nodes from start to destination.
        """
        self.id = drone_id
        self.path = path
        self.schedule: List[str] = []
        self.step_index = 0
        self.current_zone: Zone = path[0]
        self.delivered = False
        self.in_transit: bool = False
        self.transit_from: Optional[Zone] = None
        self.transit_to: Optional[Zone] = None
        self.turns_waited: int = 0
