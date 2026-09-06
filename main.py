import sys

from parsing import Parser, ParseError
from simulation import Simulation, print_colored_simulation


def main() -> None:
    parser = Parser(sys.argv)
    parser.parsing()
    sim = Simulation(parser.graph, parser.nb_drones)
    parsed_colors = parser.graph.get_zone_colors()
    print_colored_simulation(sim.run(), parsed_colors)



if __name__ == "__main__":
    try:
        main()
    except (ParseError, ValueError) as error:
        print(f"error : {error}")
