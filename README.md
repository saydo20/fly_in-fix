# 🚁 Fly-In: Drone Pathfinding & Simulation System

A high-performance multi-agent drone routing and discrete-event simulation engine. The system coordinates multiple drones navigating across a constrained spatial network from a start hub to an end hub in minimal turns, while strictly respecting node capacities, edge throughput limits, and zone transit delays.

---

## Table of Contents

1. [System Architecture (Visual Diagrams)](#1-system-architecture)
   - [High-Level Architecture (ASCII & Mermaid)](#high-level-architecture)
   - [End-to-End Data Pipeline](#end-to-end-data-pipeline)
   - [Drone Lifecycle State Machine](#drone-lifecycle-state-machine)
2. [Module-by-Module Breakdown](#2-module-by-module-breakdown)
   - [pathfinder.py (In-Depth Dijkstra & Routing Engine)](#pathfinderpy--routing-engine--dijkstra-deep-dive)
   - [models.py (Domain Entities)](#modelspy--core-domain-models)
   - [parsing.py (Grammar & Validation)](#parsingpy--map-parsing--validation)
   - [simulation.py (Discrete-Event Simulator)](#simulationpy--discrete-event-simulation)
   - [colors.py (W3C Color Adapter)](#colorspy--color-adapter)
   - [main.py (Application Entrypoint)](#mainpy--application-entrypoint)
3. [Map File Format Specification](#3-map-file-format-specification)
4. [How to Run](#4-how-to-run)

---

## 1. System Architecture

### High-Level Architecture (ASCII Diagram)

```
+===================================================================================+
|                                    main.py                                        |
|                          (Entrypoint & Orchestrator)                              |
+===================================================================================+
        |                                                   |
        v                                                   v
+-------------------------------+       +-------------------------------------------+
|          parsing.py           |       |               simulation.py               |
|  - Validates Map Grammar      |       |  - Two-Phase Turn Simulator               |
|  - Extracts Drones & Zones    |       |  - Deadlock Detection                     |
|  - Builds Adjacency Graph     |       |  - Move History Recorder                  |
+-------------------------------+       +-------------------------------------------+
        |                                         ^                       |
        v                                         |                       v
+-------------------------------+                 |           +---------------------+
|           models.py           |                 |           |      colors.py      |
|  - Graph, Zone, Connection    |                 |           |  - W3C 140+ Palette |
|  - Drone state & constraints  |                 |           |  - Rich Tag Adapter |
+-------------------------------+                 |           +---------------------+
        |                                         |                       |
        +-------------------+                     |                       v
                            v                     |           +---------------------+
                 +----------------------+         |           |       Terminal      |
                 |    pathfinder.py     |---------+           |  (Colored Output)   |
                 |  - CapacityLedger    |                     +---------------------+
                 |  - dijkstra()        |
                 |  - RoutePlanner      |
                 +----------------------+
```

### High-Level Architecture (Mermaid Diagram)

```mermaid
graph TB
    subgraph CLI["1. Entrypoint & Orchestration"]
        MAIN["main.py: main()"]
    end

    subgraph PARSING["2. Ingestion & Grammar"]
        MAP["Map File (.txt)"] --> PARSER["parsing.py: Parser"]
        PARSER -. raises .-> PERR["models.py: ParseError"]
    end

    subgraph MODELS["3. Domain Models (models.py)"]
        GRAPH["Graph"]
        ZONE["Zone"]
        CONN["Connection"]
        DRONE["Drone"]

        GRAPH o-- "1..*" ZONE
        GRAPH o-- "0..*" CONN
        ZONE --> CONN
    end

    subgraph ROUTING["4. Routing Engine (pathfinder.py)"]
        LEDGER["CapacityLedger"]
        DIJKSTRA["dijkstra() Engine"]
        PLANNER["RoutePlanner"]

        PLANNER --> LEDGER
        PLANNER --> DIJKSTRA
        LEDGER -. predicate .-> DIJKSTRA
    end

    subgraph SIM["5. Simulator (simulation.py)"]
        SIM_ENG["Simulation Engine"]
        SIM_ENG o-- "1..*" DRONE
    end

    subgraph RENDER["6. Presentation (colors.py)"]
        RENDERER["print_colored_simulation()"]
        COLOR["Color (W3C Palette)"]
        STDOUT["Terminal Output"]

        RENDERER --> COLOR
        RENDERER --> STDOUT
    end

    MAIN --> PARSER
    PARSER --> GRAPH
    MAIN --> SIM_ENG
    SIM_ENG --> PLANNER
    PLANNER --> DRONE
    MAIN --> RENDERER
```

---

### End-to-End Data Pipeline

```
[map.txt] 
   │
   ▼
1. Parser.parsing()
   ├── Validates nb_drones, zones, coordinates, connections, metadata
   └── Constructs Graph (Zones + Bidirectional Connections)
   │
   ▼
2. RoutePlanner.discover_paths()
   └── LOOP:
         ├── dijkstra(graph, can_traverse=ledger.can_traverse)
         ├── Calculate path bottleneck capacity
         ├── Consume capacity in CapacityLedger
         └── Accumulate total capacity until >= nb_drones
   │
   ▼
3. RoutePlanner.assign_drones()
   └── Distribute drones across paths via proportional quotas (Hamilton method)
   │
   ▼
4. Simulation.run()
   └── LOOP turns until all drones reach end_hub:
         ├── Phase 1: move_transit_drones()  (complete 2nd turn of restricted entry)
         ├── Phase 2: move_normal_drones()   (advance idle drones)
         └── Reset link occupancy for next turn
   │
   ▼
5. print_colored_simulation()
   └── Resolves W3C color names to Rich tags and prints turns to stdout
```

---

### Drone Lifecycle State Machine

```
      [Start Hub] (Occupancy: Infinite)
           │
           │  Turn T: connection.is_movable() & next.has_capacity()
           ├───────────────────────────────────────┐
           ▼                                       ▼
  (Next is Normal Hub)                   (Next is Restricted Hub)
           │                                       │
           ▼                                       ▼
   [Normal Hub Landed]                    [Restricted Hub - IN TRANSIT]
   • Occupancy: +1                        • in_transit = True
   • Duration: 1 turn                     • in_transit_count = +1
           │                               • Emits: D1-Start-Waypoint
           │                                       │
           │                                       │ Turn T+1: move_transit_drones()
           │                                       ▼
           │                              [Restricted Hub - LANDED]
           │                              • in_transit = False
           │                              • in_transit_count = -1
           │                              • occupancy = +1
           │                              • Emits: D1-Waypoint
           │                                       │
           └───────────────────┬───────────────────┘
                               │
                               │ Turn T+N: reaches goal
                               ▼
                        [End Hub: Goal]
                        • delivered = True
                        • Occupancy: Infinite
```

---

## 2. Module-by-Module Breakdown

---

### `pathfinder.py` — Routing Engine & Dijkstra Deep Dive

`pathfinder.py` is the algorithmic core of the project. It handles multi-criteria shortest paths, residual capacity management, and fleet path distribution.

#### 1. `CapacityLedger` (Lines 5–46)
Tracks remaining capacities dynamically without altering graph data structures:
* **`build(graph)`**: Initializes `zone_remaining` (`inf` for start/end, `max_drones` for intermediate hubs) and `link_remaining` keyed by `frozenset({zone_a.name, zone_b.name})` for undirected edge capacity.
* **`can_traverse(zone_a, zone_b)`**: Predicate that verifies both destination zone remaining capacity `> 0` and connection link remaining capacity `> 0`.
* **`path_bottleneck(path)`**: Finds the minimum remaining capacity across all intermediate nodes and links on a path ($\min(C_{zone}, C_{link})$).
* **`consume(path, amount)`**: Decrements capacity by `amount` along the path, simulating saturation in a flow network.

#### 2. `dijkstra(graph, can_traverse)` (Lines 48–98) — The Main Algorithm
Adapted Dijkstra implementation featuring:

* **Node Weighting**:
  Instead of edges carrying weights, entering a zone has an inherent turn cost:
  - `normal` / `priority`: Cost = `1`
  - `restricted`: Cost = `2`
  - `blocked`: Cost = `None` (impassable; skipped during neighbor search)

* **Lexicographical Dual-Criteria Priority Queue**:
  Elements pushed into min-heap `pq` follow the tuple format:
  ```python
  (new_cost, new_score, v.name, v)
  ```
  1. `new_cost` (**Primary**): Minimum cumulative traversal cost in turns.
  2. `new_score` (**Secondary**): Preference for priority zones (`+0` for priority zones, `+1` for normal zones). On cost ties, paths with more priority zones are chosen.
  3. `v.name` (**Tie-Breaker**): String comparison preventing `TypeError` when comparing two `Zone` objects.
  4. `v`: Reference to the `Zone` instance.

* **Dynamic Traversal Filter**:
  Accepts optional `can_traverse(u, v)` callback to skip saturated edges or nodes without modifying the original graph.

* **Path Reconstruction**:
  Backtracks predecessor pointers in `previous` from `graph.end` to `graph.start` and reverses the array.

#### 3. `RoutePlanner` (Lines 116–172)
* **`discover_paths(graph, nb_drones)`**:
  Runs an augmenting-path search loop (analogous to Ford-Fulkerson / Edmonds-Karp):
  1. Calls `dijkstra()` with `ledger.can_traverse`.
  2. Calculates the bottleneck capacity of the returned path.
  3. Consumes that capacity in `CapacityLedger`.
  4. Appends path and bottleneck to `found_paths`.
  5. Terminates when accumulated capacity $\ge$ `nb_drones` or no paths remain.
* **`assign_drones(found_paths, nb_drones)`**:
  Sorts paths by cost (fastest first) and allocates drones using the **Hamilton / Largest-Remainder method**:
  - Base quota: $\lfloor \text{nb\_drones} \times \frac{\text{bottleneck}}{\text{total\_capacity}} \rfloor$.
  - Remainder drones are distributed round-robin starting with the lowest-cost paths.

---

### `models.py` — Core Domain Models

Defines all graph, node, edge, and agent representations.

* **`LineType` (Enum)**:
  Recognized line prefixes in map files (`DRONE_COUNT`, `START_HUB`, `END_HUB`, `HUB`, `CONNECTION`).
* **`ParseError` (Exception)**:
  Custom exception formatting line number and message.
* **`Zone`**:
  Graph vertex representing a hub.
  - `cost`: `1` (normal/priority), `2` (restricted), `None` (blocked).
  - `max_drones`: Capacity constraint (default 1).
  - `occupancy`: Current number of landed drones.
  - `in_transit_count`: Current number of drones entering transit.
  - `has_capacity()`: Checks if `(occupancy + in_transit_count) < max_drones`.
* **`Connection`**:
  Directed edge stored in graph adjacency list.
  - `max_link_capacity`: Max drones allowed to traverse this link in the same turn.
  - `is_movable()`: Verifies `occupancy < max_link_capacity`.
* **`Graph`**:
  - `zones`: Dictionary mapping zone names to `Zone` instances.
  - `adjacency`: `Dict[Zone, List[Connection]]`.
  - `add_zone()`, `add_connection()`, `validate()`.
* **`Drone`**:
  Autonomous agent state tracker (`id`, `path`, `step_index`, `current_zone`, `in_transit`, `delivered`).

---

### `parsing.py` — Map Parsing & Validation

Parses map `.txt` files with strict syntax, grammar, and semantic checking.

* **Regular Expressions**:
  - `NB_DRONES`: `^nb_drones:\s*(\d+)\s*$`
  - `METADATA`: `^\[(?:\s*[A-Za-z_]\w*=[^\s\]]+)*\]$`
* **Grammar Verification**:
  1. First directive must be `nb_drones` and declared only once.
  2. Coordinates must be integers.
  3. Zone names cannot contain spaces or hyphens.
  4. Metadata values validated (`zone` in `{normal, restricted, blocked, priority}`, `max_drones` > 0, `max_link_capacity` > 0).
  5. Connection syntax `zone1-zone2` verified against existing registered zones.
  6. Exactly one `start_hub` and one `end_hub` enforced.

---

### `simulation.py` — Discrete-Event Simulation

Executes the turn-by-turn discrete simulation.

* **Two-Phase Turn Execution**:
  To prevent race conditions with 2-turn restricted zones, every turn runs in two distinct phases:
  1. **`move_transit_drones()`**: Finalizes drones completing their transit into restricted hubs. Decrements `in_transit_count`, increments `occupancy`, and marks `in_transit = False`.
  2. **`move_normal_drones()`**: Inspects idle drones. If edge capacity and destination zone capacity permit:
     - If next zone is restricted: sets `in_transit = True`, increments `next_zone.in_transit_count`.
     - If next zone is normal: moves drone directly and updates zone occupancies.
* **Link Occupancy Reset**:
  Clears connection link occupancies at the end of each turn so links can be used again on the next turn.
* **Deadlock Detection**:
  Guards against infinite loops if drones get blocked: aborts with `RuntimeError` if turns exceed `len(zones) * nb_drones * 10`.
* **`print_colored_simulation()`**:
  Renders simulation moves to the terminal using `rich.console.Console`.

---

### `colors.py` — Color Adapter

* **`Color.PALETTE`**: Complete mapping of 140+ standard W3C CSS color names to hexadecimal strings (e.g., `"aliceblue": "#F0F8FF"`, `"crimson": "#DC143C"`).
* **`Color.get_safe_color_tag(color)`**:
  Sanitizes input color strings. If found in `PALETTE`, returns the hex string; otherwise validates with `rich.style.Style.parse()`. Returns `None` if invalid, preventing simulation crashes due to malformed color tags.

---

### `main.py` — Application Entrypoint

* Reads map filepath from `sys.argv`.
* Coordinates: `Parser` $\rightarrow$ `Simulation` $\rightarrow$ `print_colored_simulation`.
* Wraps execution in top-level `try/except` for user-friendly error messages.

---

## 3. Map File Format Specification

A map file defines the fleet count, zones, and connections.

```ini
# Drone count (must be the first declaration)
nb_drones: 4

# Zones: <type>: <name> <x> <y> [metadata]
start_hub: start 0 0 [color=green]
hub: waypoint1 1 0 [zone=restricted max_drones=2 color=blue]
hub: waypoint2 1 1 [zone=priority color=cyan]
hub: obstacle 2 0 [zone=blocked]
end_hub: goal 3 0 [color=red]

# Connections: connection: <zone1>-<zone2> [max_link_capacity=N]
connection: start-waypoint1 [max_link_capacity=2]
connection: start-waypoint2 [max_link_capacity=1]
connection: waypoint1-goal
connection: waypoint2-goal
```

### Zone Types Reference

| Zone Type | Traversal Cost | Behavior |
| :--- | :---: | :--- |
| **`normal`** | 1 turn | Default zone. Traversed in 1 turn. |
| **`restricted`** | 2 turns | Requires 2 turns to enter. Drones enter an intermediate `in_transit` state on turn 1 and land on turn 2. |
| **`priority`** | 1 turn | Preferred by Dijkstra during tie-breaking over normal zones. |
| **`blocked`** | $\infty$ (None) | Impassable obstacle. Dijkstra will never route through it. |

---

## 4. How to Run

### Requirements
* Python 3.10+
* Dependencies: `rich`

### Run with Makefile
```bash
make run MAP=maps/easy/01_linear_path.txt
```

### Run Directly with Python
```bash
python3 main.py maps/easy/01_linear_path.txt
python3 main.py maps/medium/03_priority_puzzle.txt
python3 main.py maps/hard/01_maze_nightmare.txt
```

### Clean Artifacts
```bash
make clean
```
