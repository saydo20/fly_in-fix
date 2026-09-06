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
    def __init__(self, name: str, x_y: Tuple[int, int], line_type: LineType, metadata: Dict[str, object]) -> None:
        self.name = name
        self.x_y = x_y
        self.line_type = line_type
        self.is_start = line_type == LineType.START_HUB
        self.is_end = line_type == LineType.END_HUB
        self.type = metadata["zone"]
        self.color = metadata.get("color")
        self.max_drones: Optional[int] = (float('inf') if (self.is_start or self.is_end) else metadata["max_drones"])
        self.occupancy = 0
        self.in_transit_count = 0
        self.cost = 1
        if self.type == "restricted":
            self.cost = 2
        elif self.type == "blocked":
            self.cost = None


    def has_capacity(self) -> bool:
        if self.is_start or self.is_end:
            return True
            
        return (self.occupancy + self.in_transit_count) < self.max_drones

    def __repr__(self) -> str:
        return f"Zone({self.name!r}, type={self.type})"


class Connection:
    def __init__(self, destination: Zone, max_link_capacity: int = 1) -> None:
        self.destination = destination
        self.occupancy = 0
        self.max_link_capacity = max_link_capacity

    def is_movable(self):
        return self.occupancy < self.max_link_capacity

    def __repr__(self) -> str:
        return f"Connection(-> {self.destination.name}, capacity={self.max_link_capacity})"


class Graph:
    def __init__(self) -> None:
        self.zones: Dict[str, Zone] = {}
        self.adjacency: Dict[Zone, List[Connection]] = {}
        self.start: Optional[Zone] = None
        self.end: Optional[Zone] = None

    def add_zone(self, zone: Zone, line_number: int, line_type: LineType) -> None:
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

    def add_connection(self, zone1: Zone, zone2: Zone, line_number: int, max_link_capacity: int = 1) -> None:
        for connection in self.adjacency[zone1]:
            if connection.destination is zone2:
                raise ParseError(line_number, f'Connection "{zone1.name}-{zone2.name}" already exists.')

        self.adjacency[zone1].append(Connection(zone2, max_link_capacity))
        self.adjacency[zone2].append(Connection(zone1, max_link_capacity))

    def get_neighbors(self, zone: Zone) -> List[Zone]:
        return [connection.destination for connection in self.adjacency[zone]]

    def validate(self) -> None:
        if self.start is None:
            raise ParseError(None, "no start_hub zone was defined in the file.")
        if self.end is None:
            raise ParseError(None, "no end_hub zone was defined in the file.")

    def get_occupancy(self, fromm: Zone, to: Zone) -> int:
        for con in self.adjacency[fromm]:
            if con.destination == to:
                return con.occupancy

    def get_connection(self, zone1: Zone, zone2: Zone) -> Connection:
        for connection in self.adjacency[zone1]:
            if connection.destination == zone2:
                return connection
        raise ValueError(f"no connection between '{zone1.name}' and '{zone2.name}'")

    def can_move(self, fromm: Zone, to: Zone) -> bool:
        connection = self.get_connection(fromm, to)
        if (
            self.zones[to.name].has_capacity() and 
            connection.is_movable()
        ):
            return True
        return False

    def restart_capacity(self, fromm: Zone, to: Zone) -> None:
        for conection in self.adjacency[fromm]:
            if conection.destination == to:
                conection.occupancy = conection.max_link_capacity

    def print_graph(self) -> None:
        for key, value in self.zones.items():
            print(f"{key} => type: {value.type}, color: {value.color}")
        print()
        for zone, connections in self.adjacency.items():
            neighbors = ", ".join(c.destination.name for c in connections)
            print(f"{zone.name} => {neighbors}")

    def get_zone_colors(self) -> Dict[str, str]:
        return {
            zone_name: zone.color for zone_name, zone in self.zones.items() if zone.color is not None
        }


class Drone:
    def __init__(self, drone_id: str, path: List[Zone]) -> None:
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

    def set_schedule(self, schedule: List[str]) -> None:
        self.schedule = schedule

    def next_instruction(self) -> Optional[str]:
        if self.delivered or self.step_index >= len(self.schedule):
            return None
        return self.schedule[self.step_index]

    def reroute(self, new_remaining_path: List[Zone]) -> None:
        self.path = self.path[:self.step_index] + new_remaining_path
        self.turns_waited = 0

    def __repr__(self) -> str:
        status = "delivered" if self.delivered else self.current_zone.name
        return f"Drone({self.id}, {status})"
