*This project has been created as part of the 42 curriculum by sjdia.*

# Fly-In: Autonomous Drone Routing Simulation

An optimal multi-agent drone routing and dispatch simulation built from scratch in Python. The system coordinates an autonomous fleet of drones traveling across a constrained topological graph from a launch base (`start_hub`) to a delivery destination (`end_hub`) in the minimum possible number of simulation turns.

---

## Table of Contents

1. [Description](#description)
2. [Features](#features)
3. [Instructions](#instructions)
   - [Prerequisites](#prerequisites)
   - [Installation & Setup](#installation--setup)
   - [Execution](#execution)
   - [Makefile Targets](#makefile-targets)
   - [Code Quality & Linting](#code-quality--linting)
4. [Algorithm Choices & Implementation Strategy](#algorithm-choices--implementation-strategy)
   - [Graph Architecture & Constraints](#graph-architecture--constraints)
   - [Multi-Criteria Shortest Path (Custom Dijkstra)](#multi-criteria-shortest-path-custom-dijkstra)
   - [Residual Capacity Ledger & Multi-Path Augmentation](#residual-capacity-ledger--multi-path-augmentation)
   - [Proportional Fleet Allocation](#proportional-fleet-allocation)
   - [Two-Phase Discrete-Turn Simulation Engine](#two-phase-discrete-turn-simulation-engine)
5. [Visual Representation Features](#visual-representation-features)
6. [Example Input and Expected Output](#example-input-and-expected-output)
   - [Sample Map File](#sample-map-file)
   - [Simulation Run & Output Breakdown](#simulation-run--output-breakdown)
7. [Performance Benchmarks](#performance-benchmarks)
8. [Resources](#resources)
   - [References & Documentation](#references--documentation)
   - [Use of Artificial Intelligence](#use-of-artificial-intelligence)

---

## Description

**Fly-In** addresses a Multi-Agent Pathfinding (MAPF) and network flow problem under discrete-time execution. A fleet of $N$ drones must navigate an interconnected network of hubs while strictly respecting topological bottlenecks, zone capacities, movement costs, and simultaneous traversal rules.

### Core Objectives
- **Turn Minimization**: Guide all drones from `start_hub` to `end_hub` in the fewest possible discrete simulation turns.
- **Strict Capacity Compliance**: Prevent collision and structural overload across all graph elements.
- **Zero Graph Libraries**: Implemented entirely with custom Object-Oriented data structures without third-party graph frameworks (such as NetworkX or graphlib).
- **Type Safety & Standards Compliance**: Fully typed codebase conforming to PEP 8, PEP 257 docstrings, and verified via `flake8` and `mypy`.

### Zone Types & Physics
| Zone Type | Movement Cost | Description |
| :--- | :---: | :--- |
| **`normal`** | 1 turn | Standard waypoint hub with standard 1-turn traversal. |
| **`restricted`** | 2 turns | High-security / hazardous airspace. Entering takes 2 full turns; drone occupies the connection link during transit. |
| **`priority`** | 1 turn | Preferred aerial corridor. 1-turn movement cost, favored over normal zones during pathfinding tie-breaking. |
| **`blocked`** | $\infty$ | Prohibited airspace / physical obstacle. Unreachable; completely pruned from path discovery. |

### Capacity Constraints
- **Zone Capacity (`max_drones`)**: Specifies the maximum number of drones that may occupy a zone at any given moment (default: 1). Note that `start_hub` and `end_hub` have infinite capacity.
- **Connection Capacity (`max_link_capacity`)**: Limits how many drones can traverse a bidirectional connection link simultaneously during the same turn (default: 1).
- **Dynamic Space Recycling**: Drones departing a zone immediately free up capacity during that same turn, allowing trailing drones to enter seamlessly without artificial turn delays.

---

## Features

- **Pure Object-Oriented Architecture**: Modular decoupling across `Graph`, `Zone`, `Connection`, `Drone`, `Parser`, `RoutePlanner`, and `Simulation`.
- **Fault-Tolerant Custom Parser**: Robust syntax validation, regex-based token parsing, and informative error diagnostics reporting exact line numbers and root causes.
- **Lexicographical Multi-Objective Pathfinding**: Dual-key Dijkstra algorithm favoring `priority` zones while finding the shortest cost routes.
- **Network Flow Path Augmentation**: Discovers non-interfering and alternative paths using residual capacity tracking until all drones have allocated routes.
- **Conflict-Free Two-Phase Simulation**: Resolves transit drone arrivals before launching new movements to eliminate race conditions and deadlocks.
- **Rich Terminal Visualization**: Colorizes movements according to custom map metadata using an integrated 140+ color palette.

---

## Instructions

### Prerequisites
- **Python 3.10** or later
- **pip** package manager

### Installation & Setup

1. **Clone the repository:**
   ```bash
   git clone git@github.com:saydo20/fly_in-fix.git
   cd fly_in-fix
   ```

2. **Create and activate a virtual environment:**
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. **Install dependencies:**
   ```bash
   pip install rich flake8 mypy
   ```

### Execution

Run the simulation by providing a map configuration file as a command-line argument:
```bash
python3 main.py <path_to_map>
```

**Examples:**
```bash
# Easy linear path map
python3 main.py maps/easy/01_linear_path.txt

# Medium priority puzzle map
python3 main.py maps/medium/03_priority_puzzle.txt

# Hard ultimate challenge map
python3 main.py maps/hard/03_ultimate_challenge.txt
```

### Makefile Targets

A provided `Makefile` automates routine operations:

- **Install dependencies**:
  ```bash
  make install
  ```
- **Run default map**:
  ```bash
  make run
  ```
- **Run a specific map**:
  ```bash
  make run MAP=maps/hard/01_maze_nightmare.txt
  ```
- **Interactive debugging (PDB)**:
  ```bash
  make debug
  ```
- **Clean cached files and bytecode**:
  ```bash
  make clean
  ```
- **Run linters (Flake8 & MyPy)**:
  ```bash
  make lint
  ```

### Code Quality & Linting

The repository strictly adheres to 42 coding standards and static typing requirements.

- **Check style and formatting (Flake8):**
  ```bash
  flake8 .
  ```
- **Static Type Checking (MyPy):**
  ```bash
  mypy . --warn-return-any --warn-unused-ignores --ignore-missing-imports --disallow-untyped-defs --check-untyped-defs
  ```
- **Strict Type Checking (Optional):**
  ```bash
  mypy . --strict
  ```
- **Interactive Debugging:**
  ```bash
  python3 -m pdb main.py maps/easy/01_linear_path.txt
  ```

---

## Algorithm Choices & Implementation Strategy

```
  +------------------+       +---------------------+       +-----------------------+
  |    Input Map     |  -->  |    Parser & Graph   |  -->  |     RoutePlanner      |
  |  (Syntax Check)  |       |  Adjacency Network  |       | (Dijkstra + Residual) |
  +------------------+       +---------------------+       +-----------------------+
                                                                       |
                                                                       v
  +------------------+       +---------------------+       +-----------------------+
  | Final Output &   |  <--  |  Rich Console View  |  <--  |   Simulation Engine   |
  | Turn Statistics  |       |  (Colorized Steps)  |       |  (Two-Phase Execution)|
  +------------------+       +---------------------+       +-----------------------+
```

### Graph Architecture & Constraints
The network is structured as an undirected adjacency graph without relying on external libraries:
- `Zone`: Represents vertices with integer Cartesian coordinates $(x, y)$, zone types, maximum drone capacities (`max_drones`), current occupancy, in-transit counters, and traversal costs.
- `Connection`: Models bidirectional edges between zones, managing directional links and independent capacity trackers (`max_link_capacity`).
- `Graph`: Houses the network directory, verifies the existence of exactly one start and one end hub, and validates connection invariants.

### Multi-Criteria Shortest Path (Custom Dijkstra)
Pathfinding is implemented using a modified Dijkstra algorithm utilizing Python's `heapq` priority queue. Rather than evaluating cost alone, priority is evaluated through a lexicographical tuple:

$$\text{Priority Key} = (\text{Total Movement Cost}, \text{Priority Penalty Score})$$

1. **Movement Cost**:
   - `normal`: Cost = 1 turn.
   - `restricted`: Cost = 2 turns.
   - `blocked`: Inaccessible (cost = $\infty$, skipped entirely).
2. **Priority Penalty Score**:
   - `priority` zones grant a penalty increment of $+0$.
   - Non-priority zones add a penalty increment of $+1$.
   
When multiple paths have identical total movement costs, the algorithm automatically favors routes traversing `priority` corridors.

### Residual Capacity Ledger & Multi-Path Augmentation
Routing multiple drones down a single path creates bottlenecks when the path capacity is lower than the fleet size. To maximize throughput:
- A `CapacityLedger` monitors the remaining capacity of intermediate zones (`zone_remaining`) and bidirectional connections (`link_remaining`).
- For each path discovered by Dijkstra:
  $$\text{bottleneck} = \min \left( \min_{z \in \text{path} \setminus \{\text{start}, \text{end}\}} \text{zone\_remaining}[z], \min_{e \in \text{edges}} \text{link\_remaining}[e] \right)$$
- The path's bottleneck capacity is deducted from the ledger.
- Dijkstra is repeatedly invoked on the residual network until the cumulative capacity across all discovered paths meets or exceeds the fleet size ($\sum \text{bottleneck} \ge \text{nb\_drones}$), or no additional paths remain.

### Proportional Fleet Allocation
Once the optimal set of paths is discovered:
- The `RoutePlanner` computes proportional drone quotas based on each path's bottleneck capacity:
  $$\text{quota}_i = \left\lfloor \text{nb\_drones} \times \frac{\text{bottleneck}_i}{\text{total\_capacity}} \right\rfloor$$
- Any remaining drones are assigned greedily to the lowest-cost paths.
- Drones are assigned to paths in round-robin order sorted by path cost, ensuring shorter paths are dispatched first.

### Two-Phase Discrete-Turn Simulation Engine
The simulation loop executes discrete turns until all drones reach `end_hub`. To avoid race conditions and strictly enforce turn mechanics, each turn is split into two phases:

1. **Phase 1: Transit Movement Resolution (`move_transit_drones`)**:
   - Drones that entered a `restricted` zone during the previous turn are currently "in flight" across the connection link.
   - These drones are finalized into the destination zone: link occupancy is released, destination `in_transit_count` is decremented, destination `occupancy` is incremented, and status is logged as `D<ID>-<destination>`.
2. **Phase 2: Normal Movement & Transit Departure (`move_normal_drones`)**:
   - Waiting drones evaluate their next step.
   - If the target zone is `normal` or `priority`, and capacity is available:
     - The drone moves immediately (1 turn), vacating its current zone and claiming the target zone (`D<ID>-<zone>`).
   - If the target zone is `restricted`:
     - The drone departs into transit (`D<ID>-<from>-<to>`), increments the target's `in_transit_count`, vacates its current zone, and marks itself as in-flight for Phase 1 of the subsequent turn.

---

## Visual Representation Features

The project incorporates terminal visualization using the `rich` library and a custom color management module (`colors.py`):

1. **140+ Named Color Palette**:
   - Full support for standard CSS/X11 color keywords (e.g., `green`, `red`, `blue`, `yellow`, `cyan`, `coral`, `gold`, `mediumpurple`, etc.) mapped to verified HEX color codes.
   - Fallback parser to safely accept Rich style strings while discarding invalid entries without crashing.

2. **Context-Aware Visual Output**:
   - When a zone defines a `[color=<name>]` attribute in the map file, that zone's name is dynamically rendered in that exact color in the terminal log.
   - Drones in flight display both departure and destination zones with their respective colors (e.g., `D1-start-waypoint1`).

3. **Enhancing User & Evaluator Experience**:
   - **Immediate Status Tracking**: Peer reviewers can instantly follow individual drone trajectories visually without getting lost in monochrome text walls.
   - **Bottleneck Identification**: High-traffic convergence hubs and restricted zones stand out vibrantly, simplifying verification of capacity rules.
   - **Clear Termination Diagnostics**: Turn counts and delivery statuses are highlighted upon completion.

---

## Example Input and Expected Output

### Sample Map File
Below is the map definition from `maps/easy/01_linear_path.txt`:

```ini
# Easy Level 1: Simple linear path
nb_drones: 2

start_hub: start 0 0 [color=green]
hub: waypoint1 1 0 [color=blue zone=restricted max_drones=2] 
hub: waypoint2 2 0 [color=blue]
end_hub: goal 3 0 [color=red]

connection: start-waypoint1
connection: waypoint1-waypoint2
connection: waypoint2-goal
```

**Syntax Breakdown:**
- `nb_drones: 2`: Fleet size of 2 drones (`D1`, `D2`).
- `start_hub: start 0 0 [color=green]`: Launch base at $(0, 0)$.
- `hub: waypoint1 1 0 [...]`: Restricted zone costing 2 turns to enter, with capacity for 2 drones.
- `hub: waypoint2 2 0 [...]`: Standard zone with 1 turn movement cost and default capacity of 1 drone.
- `end_hub: goal 3 0 [color=red]`: Target destination at $(3, 0)$.
- `connection: ...`: Undirected edges linking the route.

### Simulation Run & Output Breakdown

Running the program:
```bash
python3 main.py maps/easy/01_linear_path.txt
```

**Terminal Output:**
```text
D1-start-waypoint1
D1-waypoint1
D1-waypoint2 D2-start-waypoint1
D2-waypoint1 D1-goal
D2-waypoint2
D2-goal
number of turns is 6
```

**Step-by-Step Chronology:**
- **Turn 1**: Drone `D1` leaves `start` heading into restricted zone `waypoint1` (`D1-start-waypoint1`).
- **Turn 2**: `D1` completes transit and enters `waypoint1` (`D1-waypoint1`).
- **Turn 3**: `D1` moves to `waypoint2`. Simultaneously, `D2` launches towards `waypoint1` (`D2-start-waypoint1`).
- **Turn 4**: `D1` reaches `goal` (delivered!). Concurrently, `D2` completes transit into `waypoint1` (`D2-waypoint1`).
- **Turn 5**: `D2` moves to `waypoint2` (`D2-waypoint2`).
- **Turn 6**: `D2` reaches `goal` (delivered!). All drones delivered in 6 turns.

---

## Performance Benchmarks

The algorithm was tested against the official benchmark suites across all difficulty tiers:

| Map Difficulty | Map File | Drones | Target Benchmark | Actual Result | Status |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Easy** | `01_linear_path.txt` | 2 | $\le 6$ turns | **6 turns** |  Target Met |
| **Easy** | `02_simple_fork.txt` | 4 | $\le 8$ turns | **4 turns** |  Beats Target (+4) |
| **Easy** | `03_basic_capacity.txt` | 4 | $\le 6$ turns | **4 turns** |  Beats Target (+2) |
| **Medium** | `01_dead_end_trap.txt` | 5 | $\le 12$ turns | **8 turns** |  Beats Target (+4) |
| **Medium** | `02_circular_loop.txt` | 6 | $\le 15$ turns | **15 turns** |  Target Met |
| **Medium** | `03_priority_puzzle.txt` | 5 | $\le 12$ turns | **7 turns** |  Beats Target (+5) |
| **Hard** | `01_maze_nightmare.txt` | 8 | $\le 30$ turns | **13 turns** |  Beats Target (+17) |
| **Hard** | `02_capacity_hell.txt` | 12 | $\le 35$ turns | **16 turns** |  Beats Target (+19) |
| **Hard** | `03_ultimate_challenge.txt` | 15 | $\le 45$ turns | **26 turns** |  Beats Target (+19) |
| **Challenger** | `01_the_impossible_dream.txt` | 25 | Record: 45 turns | **67 turns** |  Solves complex challenge |

*Note: All mandatory benchmarks (Easy, Medium, and Hard) are fully satisfied or significantly beaten by the route planner.*

---

## Resources

### References & Documentation
- **Graph Theory & Pathfinding**:
  - Dijkstra, E. W. (1959). *A note on two problems in connexion with graphs*. Numerische Mathematik, 1, 269–271.
  - Cormen, T. H., Leiserson, C. E., Rivest, R. L., & Stein, C. (2009). *Introduction to Algorithms* (3rd ed.). MIT Press. (Chapter 24: Single-Source Shortest Paths; Chapter 26: Maximum Flow).
- **Multi-Agent Pathfinding (MAPF)**:
  - Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015). *Conflict-Based Search for Optimal Multi-Agent Pathfinding*. Artificial Intelligence, 219, 40–66.
  - Ford, L. R., & Fulkerson, D. R. (1956). *Maximal flow through a network*. Canadian Journal of Mathematics, 8, 399–404.
- **Python Engineering & Tooling**:
  - [Python Standard Library `heapq`](https://docs.python.org/3/library/heapq.html): Priority queue algorithms.
  - [Python Standard Library `typing`](https://docs.python.org/3/library/typing.html): Static type annotations.
  - [Rich Library Documentation](https://rich.readthedocs.io/): Terminal formatting and ANSI styling.
  - [PEP 8 -- Style Guide for Python Code](https://peps.python.org/pep-0008/) & [PEP 257 -- Docstring Conventions](https://peps.python.org/pep-0257/).

### Use of Artificial Intelligence

In accordance with 42 AI usage guidelines, artificial intelligence was incorporated responsibly as an assistant throughout development:

1. **Parser Regex & Edge Case Validation**:
   - AI was used to help formulate and refine regular expressions (`NB_DRONES`, `METADATA`) in `parsing.py` to ensure exact compliance with map formatting and metadata syntax.
   - Edge cases such as duplicate keys, negative numbers, and trailing whitespace handling were validated with AI assistance.

2. **Algorithm Design & Heuristic Formulation**:
   - AI was consulted during the design of the dual-objective tuple heuristic `(cost, priority_penalty)` in `pathfinder.py` to seamlessly bias Dijkstra towards `priority` zones without distorting actual path costs.
   - Conceptual ideas for network flow residual tracking (`CapacityLedger`) were brainstormed with AI to find an efficient greedy path augmentation technique compliant with the "no graph libraries" rule.

3. **Color Palette Mapping**:
   - AI generated the dictionary of 140+ CSS/X11 named color mappings in `colors.py`, ensuring rich visual feedback regardless of the color string supplied in map files.

4. **Code Review & Static Analysis**:
   - AI served as a rubber-ducking partner to identify potential race conditions in two-phase turn execution (differentiating in-transit vs normal drone advancement).
   - Ensured clean typing hints and flake8/mypy conformance across all modules.
