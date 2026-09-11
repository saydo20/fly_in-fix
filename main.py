import sys
from parsing import Parser
from simulation import Simulation, print_colored_simulation


def main() -> None:
    """Execute drone pathfinding simulation from command-line arguments.

    Parses the map file, initializes the simulation engine, and
    outputs the colorized turn-by-turn drone movements.
    """
    parser = Parser(sys.argv)
    parser.parsing()
    sim = Simulation(parser.graph, parser.nb_drones)
    parsed_colors = parser.graph.get_zone_colors()
    print_colored_simulation(sim.run(), parsed_colors)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"error : {error}")
