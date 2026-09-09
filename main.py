import sys
from parsing import Parser, ParseError
from simulation import Simulation, print_colored_simulation
from pathfinder import dijkstra


def main() -> None:
    parser = Parser(sys.argv)
    parser.parsing()
    # sim = Simulation(parser.graph, parser.nb_drones)
    dist = dijkstra(parser.graph, None)
    for zone, cost in dist.items():
        print(zone.name, cost)
    # parsed_colors = parser.graph.get_zone_colors()
    # print_colored_simulation(sim.run(), parsed_colors)



if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"error : {error}")
