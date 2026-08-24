from typing import List, Optional, Set, Tuple

from models import Connection, Drone, Graph, Zone
from pathfinder import Pathfinder


class SimulationError(Exception):
    """Raised when the simulation detects a deadlock or exceeds the turn limit."""


class Simulation:
    """
    Turn-based multi-drone simulation.

    Strategy
    --------
    Each drone asks the Pathfinder for its optimal next step every turn.
    The Pathfinder uses a pre-computed reverse Dijkstra so that when multiple
    equal-cost paths exist (e.g. three parallel restricted-zone chains), it
    automatically routes each drone to the LEAST LOADED option.  On maps with
    a single optimal path the behavior is identical to a static shortest path:
    drones queue through bottlenecks without wasting turns on detours.

    Turn structure (two ordered phases)
    ------------------------------------
    Phase 1 – Transit completion
        Drones that began crossing a restricted zone last turn now arrive at
        their destination.  Running this phase first means the freed capacity
        is visible to Phase 2 drones in the same turn.

    Phase 2 – New movements
        Every grounded drone calls find_next_step().  If the returned zone AND
        the link to it both have remaining capacity the drone moves; otherwise
        it waits.  Occupancy counters are updated immediately after each move,
        so later drones in the same Phase 2 loop see accurate state.

    After both phases all per-turn link occupancies are reset to zero.

    Deadlock detection
    ------------------
    * A turn with zero moves while undelivered drones still exist is a
      permanent deadlock → SimulationError is raised immediately.
      (In-transit drones always generate a Phase 1 move, so zero total moves
      implies no drone was in transit AND no grounded drone could advance.)
    * A hard MAX_TURNS ceiling guards against algorithmic cycles.
    """

    def __init__(self, graph: Graph, nb_drones: int) -> None:
        self.graph = graph
        self.pathfinder = Pathfinder(graph)

        try:
            self.pathfinder.find_path()
        except ValueError as exc:
            raise ValueError(f"Invalid map: {exc}") from exc

        self.drones: List[Drone] = [
            Drone(f"D{i + 1}", graph.start) for i in range(nb_drones)
        ]


    def run(self) -> List[str]:
        """
        Simulate until every drone has been delivered.

        Returns
        -------
        List of output lines — one per turn — followed by the turn-count summary.

        Raises
        ------
        SimulationError  if a deadlock is detected or MAX_TURNS is exceeded.
        """
        output_lines: List[str] = []
        turns = 0

        while any(not d.delivered for d in self.drones):

            turn_moves, connections_used = self._run_turn()

            for conn in connections_used:
                conn.occupancy = 0
            if not turn_moves:
                blocked = [d.id for d in self.drones if not d.delivered]
                raise SimulationError(
                    f"Deadlock detected: drones {blocked} cannot move."
                )

            output_lines.append(" ".join(turn_moves))
            turns += 1

        output_lines.append(f"number of turns is {len(output_lines)}")
        return output_lines


    def _run_turn(self) -> Tuple[List[str], List[Connection]]:
        """
        Execute one full simulation turn.

        Returns
        -------
        moves             : formatted move strings for this turn's output line
        connections_used  : Connection objects whose occupancy must be reset
        """
        moves: List[str] = []
        connections_used: List[Connection] = []
        moved_this_turn: Set[str] = set()

        for drone in self.drones:
            if drone.delivered or not drone.in_transit:
                continue
            move = self._complete_transit(drone, connections_used)
            moves.append(move)
            moved_this_turn.add(drone.id)

        for drone in self.drones:
            if drone.delivered or drone.in_transit or drone.id in moved_this_turn:
                continue
            move = self._attempt_move(drone, connections_used)
            if move is not None:
                moves.append(move)

        return moves, connections_used

    def _complete_transit(self, drone: Drone, connections_used: List[Connection]) -> str:
        """
        Finalise a restricted-zone crossing started on the previous turn.

        The drone physically arrives at transit_to.  We also consume the link
        capacity for this turn so Phase 2 cannot send another drone through
        the same link simultaneously.

        Returns the formatted output string for this arrival.
        """
        from_zone: Zone = drone.transit_from
        to_zone: Zone = drone.transit_to

        connection = self.graph.get_connection(from_zone, to_zone)
        connection_rev = self.graph.get_connection(to_zone, from_zone)
        connection.occupancy += 1
        connection_rev.occupancy += 1
        connections_used.extend([connection, connection_rev])

        to_zone.in_transit_count -= 1
        if not to_zone.is_end:
            to_zone.occupancy += 1

        drone.current_zone = to_zone
        drone.in_transit = False
        drone.transit_from = None
        drone.transit_to = None

        if to_zone.is_end:
            drone.delivered = True

        return f"{drone.id}-{to_zone.name}"

    def _attempt_move(self, drone: Drone, connections_used: List[Connection]) -> Optional[str]:
        """
        Try to advance `drone` one step toward the end hub.

        The Pathfinder picks the optimal next zone (least-loaded among
        equal-cost alternatives).  We then verify live capacity: if the chosen
        zone or the link to it is momentarily full, the drone waits.

        Occupancy counters are updated immediately so later drones in the same
        Phase 2 loop see the current state of zones and links.

        Move types
        ----------
        Normal / priority zone  → drone arrives in the same turn.
        Restricted zone         → drone departs this turn, arrives next turn
                                  (output: "DroneId-from_zone-to_zone").
        """
        next_zone = self.pathfinder.find_next_step(drone.current_zone)
        if next_zone is None:
            return None

        connection = self.graph.get_connection(drone.current_zone, next_zone)
        connection_rev = self.graph.get_connection(next_zone, drone.current_zone)

        if not connection.is_movable() or not next_zone.has_capacity():
            return None

        connection.occupancy += 1
        connection_rev.occupancy += 1
        connections_used.extend([connection, connection_rev])
        current_zone = drone.current_zone

        if next_zone.type == "restricted":
            if not current_zone.is_start:
                current_zone.occupancy -= 1
            next_zone.in_transit_count += 1
            drone.in_transit = True
            drone.transit_from = current_zone
            drone.transit_to = next_zone
            return f"{drone.id}-{current_zone.name}-{next_zone.name}"
        else:
            if not current_zone.is_start:
                current_zone.occupancy -= 1
            if not next_zone.is_end:
                next_zone.occupancy += 1
            drone.current_zone = next_zone
            if next_zone.is_end:
                drone.delivered = True
            return f"{drone.id}-{next_zone.name}"
