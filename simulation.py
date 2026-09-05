from typing import List, Set
from models import Graph, Drone
from pathfinder import RoutePlanner


class Simulation:

    def __init__(self, graph: Graph, nb_drones: int) -> None:
        self.graph = graph
        self.connection_list: List = []
        planner = RoutePlanner()

        found_paths = planner.discover_paths(graph, nb_drones)
        assignments = planner.assign_drones(found_paths, nb_drones)

        self.drones: List[Drone] = [
            Drone(f"D{i + 1}", path)
            for i, path in enumerate(assignments)
        ]


    def move_transit_drones(self, moves: List, moved_this_turn: Set):
        for drone in self.drones:
            if drone.delivered or not drone.in_transit:
                continue

            to_zone = drone.transit_to

            if to_zone is None:
                continue


            current_zone = drone.current_zone
            next_zone = drone.path[drone.step_index + 1]

            connection = self.graph.get_connection(current_zone, next_zone)
            connection_revers = self.graph.get_connection(next_zone, current_zone)

            connection.occupancy += 1
            connection_revers.occupancy += 1
            to_zone.in_transit_count -= 1
            if not to_zone.is_end:
                to_zone.occupancy += 1

            drone.current_zone = to_zone
            drone.step_index += 1
            drone.in_transit = False
            drone.transit_from = None
            drone.transit_to = None
            drone.turns_waited = 0

            moves.append(f"{drone.id}-{to_zone.name}")
            moved_this_turn.add(drone.id)

            if drone.step_index == len(drone.path) - 1:
                drone.delivered = True

    def move_normal_drones(self, moves: List, moved_this_turn: Set):
        for drone in self.drones:
            if (drone.delivered or drone.in_transit or drone.id in moved_this_turn):
                continue

            if drone.step_index + 1 >= len(drone.path):
                drone.delivered = True
                continue

            current_zone = drone.current_zone
            next_zone = drone.path[drone.step_index + 1]

            connection = self.graph.get_connection(current_zone, next_zone)
            connection_revers = self.graph.get_connection(next_zone, current_zone)
            if connection.is_movable() and next_zone.has_capacity():
                connection.occupancy += 1
                connection_revers.occupancy += 1
                self.connection_list.extend([connection, connection_revers])

                if next_zone.type == "restricted":
                    if not current_zone.is_start:
                        current_zone.occupancy -= 1
                    next_zone.in_transit_count += 1
                    drone.in_transit = True
                    drone.transit_from = current_zone
                    drone.transit_to = next_zone
                    moves.append(f"{drone.id}-{current_zone.name}-{next_zone.name}")
                else:
                    if not current_zone.is_start:
                        current_zone.occupancy -= 1
                    if not next_zone.is_end:
                        next_zone.occupancy += 1
                    drone.current_zone = next_zone
                    drone.step_index += 1
                    moves.append(f"{drone.id}-{next_zone.name}")
                    if drone.step_index == len(drone.path) - 1:
                        drone.delivered = True

                drone.turns_waited = 0
                moved_this_turn.add(drone.id)
            else:
                drone.turns_waited += 1

    def run(self) -> List[str]:
        lines: List[str] = []

        max_turns = len(self.graph.zones) * len(self.drones) * 10
        turn = 0

        while any(not drone.delivered for drone in self.drones):
            self.connecion_list = []
            turn += 1
            if turn > max_turns:
                raise RuntimeError(
                    f"Simulation deadlocked after {turn - 1} turns. "
                    "Some drones could not reach the destination.")
            moves: List[str] = []
            moved_this_turn: Set[str] = set()

            self.move_transit_drones(moves, moved_this_turn)
            self.move_normal_drones(moves, moved_this_turn)
            for connection in self.connection_list:
                        connection.occupancy = 0

            if moves:
                lines.append(" ".join(moves))

        lines.append(f"number of turns is {len(lines)}")
        return lines