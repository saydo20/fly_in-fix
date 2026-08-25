import sys

from parsing import Parser, ParseError
from simulation import Simulation


def main() -> None:
    parser = Parser(sys.argv)
    parser.parsing()
    sim = Simulation(parser.graph, parser.nb_drones)
    print(*sim.run(), sep="\n")


if __name__ == "__main__":
    try:
        main()
    except (ParseError, ValueError) as error:
        print(f"error : {error}")
