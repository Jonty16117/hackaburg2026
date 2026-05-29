# Graph Report - .  (2026-05-29)

## Corpus Check
- Corpus is ~22,847 words - fits in a single context window. You may not need a graph.

## Summary
- 213 nodes · 389 edges · 20 communities detected
- Extraction: 75% EXTRACTED · 25% INFERRED · 0% AMBIGUOUS · INFERRED: 97 edges (avg confidence: 0.75)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Hardware Components & BOM|Hardware Components & BOM]]
- [[_COMMUNITY_Simulation Engine|Simulation Engine]]
- [[_COMMUNITY_Motor Drive Layer|Motor Drive Layer]]
- [[_COMMUNITY_Navigation Controller|Navigation Controller]]
- [[_COMMUNITY_Dashboard REST API|Dashboard REST API]]
- [[_COMMUNITY_Brain FSM Core|Brain FSM Core]]
- [[_COMMUNITY_Autopilot Command Handlers|Autopilot Command Handlers]]
- [[_COMMUNITY_Agent Workflows & Testing|Agent Workflows & Testing]]
- [[_COMMUNITY_Duck Pose & Reset|Duck Pose & Reset]]
- [[_COMMUNITY_Configuration Management|Configuration Management]]
- [[_COMMUNITY_Navigation Config|Navigation Config]]
- [[_COMMUNITY_Obstacle Deletion|Obstacle Deletion]]
- [[_COMMUNITY_Obstacle Creation|Obstacle Creation]]
- [[_COMMUNITY_Goal & Markers|Goal & Markers]]
- [[_COMMUNITY_Sonar Override|Sonar Override]]
- [[_COMMUNITY_sensors pkg|sensors pkg]]
- [[_COMMUNITY_motors pkg|motors pkg]]
- [[_COMMUNITY_navigation pkg|navigation pkg]]
- [[_COMMUNITY_dashboard pkg|dashboard pkg]]
- [[_COMMUNITY_scripts pkg|scripts pkg]]

## God Nodes (most connected - your core abstractions)
1. `_ensure_sim()` - 28 edges
2. `SimEngine` - 28 edges
3. `DuckDrive` - 18 edges
4. `run_navigation()` - 16 edges
5. `Perimeter` - 11 edges
6. `Brain` - 11 edges
7. `Sonar` - 7 edges
8. `Motor` - 7 edges
9. `DuckBot autonomous navigation controller.  Reads sonar, dead-reckons position, r` - 7 edges
10. `Run the sense→think→act navigation loop at ~20 Hz.      Args:         on_cycle:` - 7 edges

## Surprising Connections (you probably didn't know these)
- `navigation/brain.py — 4-State FSM` --semantically_similar_to--> `Simulation Autopilot FSM (GO→REVERSE→SCAN→FACE_BEST→COOLDOWN/PERIM)`  [INFERRED] [semantically similar]
  README.md → docs/api.md
- `DuckBot autonomous navigation controller.  Reads sonar, dead-reckons position, r` --uses--> `DuckDrive`  [INFERRED]
  navigation/controller.py → motors/drive.py
- `Run the sense→think→act navigation loop at ~20 Hz.      Args:         on_cycle:` --uses--> `DuckDrive`  [INFERRED]
  navigation/controller.py → motors/drive.py
- `Scripted / interactive motor control for DuckBot.  Usage:     python -m scripts.` --uses--> `DuckDrive`  [INFERRED]
  scripts/run.py → motors/drive.py
- `run_navigation()` --calls--> `DuckDrive`  [INFERRED]
  navigation/controller.py → motors/drive.py

## Hyperedges (group relationships)
- **Autonomous Navigation Pipeline (odometry + perimeter + brain + controller)** — odometry_py, perimeter_py, brain_py, controller_py, config_py [EXTRACTED 1.00]
- **Hardware Safety Constraints (BEC isolation, propeller removal, LiPo handling, waterproofing)** — bec_5v_warning, optimal_voltage_rationale, perimeter_margin_rationale, cw_ccw_pair_rationale [INFERRED 0.75]
- **Hardware Control Stack (GPIO pins → ESC → Thruster / RCWL Sensor)** — gpio_pin_12, gpio_pin_13, gpio_pin_23, gpio_pin_24, esc_py, sonar_py, apisqueen_30a_esc, rcwl_1655, voltage_divider [EXTRACTED 1.00]

## Communities

### Community 0 - "Hardware Components & BOM"
Cohesion: 0.07
Nodes (42): APISQUEEN 30A Bi-Directional ESC, APISQUEEN — Underwater Thruster Manufacturer, APISQUEEN U01 Brushless Thrusters, BEC 5V Safety Warning — Do Not Connect to Pi, navigation/brain.py — 4-State FSM, Camera (TBD) — Object Detection, CCPM 3-Channel Servo Tester, navigation/config.py — Tunable Constants (+34 more)

### Community 1 - "Simulation Engine"
Cohesion: 0.1
Nodes (14): api_apply_scenario(), api_get_obstacles(), api_set_speeds(), api_sim_step(), _sim_loop(), _ws_set_speeds(), _ws_sim_step(), Server-side simulation engine for DuckBot.  Ports the browser JS physics + autop (+6 more)

### Community 2 - "Motor Drive Layer"
Cohesion: 0.18
Nodes (8): DuckDrive, Motor, _us_to_duty(), auto_sequence(), calibrate(), main(), manual_control(), Scripted / interactive motor control for DuckBot.  Usage:     python -m scripts.

### Community 3 - "Navigation Controller"
Cohesion: 0.12
Nodes (12): _AvoidPhase, Finite state machine: sonar + odometry + perimeter → motor speed commands., State, main(), DuckBot autonomous navigation controller.  Reads sonar, dead-reckons position, r, Run the sense→think→act navigation loop at ~20 Hz.      Args:         on_cycle:, run_navigation(), Enum (+4 more)

### Community 4 - "Dashboard REST API"
Cohesion: 0.12
Nodes (10): fastapi (Web Framework), api_get_debug(), api_get_state(), api_get_state_history(), get_live(), DuckBot dashboard server — FastAPI + SSE + WebSocket + REST API.  Runs a server-, _read_state(), _startup() (+2 more)

### Community 5 - "Brain FSM Core"
Cohesion: 0.2
Nodes (4): Brain, Perimeter, Perimeter awareness: point-in-polygon, distance-to-edge, bearing-to-center., heading_error()

### Community 6 - "Autopilot Command Handlers"
Cohesion: 0.25
Nodes (9): api_autopilot_start(), api_autopilot_stop(), api_clear_obstacles(), api_sim_toggle(), _ensure_sim(), _ws_autopilot_start(), _ws_autopilot_stop(), _ws_clear_obs() (+1 more)

### Community 7 - "Agent Workflows & Testing"
Cohesion: 0.38
Nodes (7): Parameter Sweep Testing Workflow, DuckBot REST API, Scenario-Based Reproducible Testing, Sensor Injection Testing Technique, Simulation Physics Engine (~20Hz, 50ms tick), dashboard/simulator.html — Browser Dashboard UI, WebSocket Real-Time Telemetry

### Community 8 - "Duck Pose & Reset"
Cohesion: 0.33
Nodes (4): api_reset(), api_set_pose(), _ws_reset(), _ws_set_pose()

### Community 9 - "Configuration Management"
Cohesion: 0.4
Nodes (4): api_get_config(), api_update_config(), get_config(), _ws_set_config()

### Community 10 - "Navigation Config"
Cohesion: 0.67
Nodes (1): DuckBot navigation configuration — tune these values for your setup.

### Community 11 - "Obstacle Deletion"
Cohesion: 0.67
Nodes (2): api_remove_obstacle(), _ws_rm_obs()

### Community 12 - "Obstacle Creation"
Cohesion: 0.67
Nodes (2): api_add_obstacle(), _ws_add_obs()

### Community 13 - "Goal & Markers"
Cohesion: 0.67
Nodes (2): api_set_goal(), _ws_set_goal()

### Community 14 - "Sonar Override"
Cohesion: 0.67
Nodes (2): api_set_sonar(), _ws_set_sonar()

### Community 15 - "sensors pkg"
Cohesion: 1.0
Nodes (0): 

### Community 16 - "motors pkg"
Cohesion: 1.0
Nodes (0): 

### Community 17 - "navigation pkg"
Cohesion: 1.0
Nodes (0): 

### Community 18 - "dashboard pkg"
Cohesion: 1.0
Nodes (0): 

### Community 19 - "scripts pkg"
Cohesion: 1.0
Nodes (0): 

## Knowledge Gaps
- **21 isolated node(s):** `DuckBot navigation configuration — tune these values for your setup.`, `Perimeter awareness: point-in-polygon, distance-to-edge, bearing-to-center.`, `Dead reckoning: integrates motor speed commands into pose estimate.`, `Finite state machine: sonar + odometry + perimeter → motor speed commands.`, `Shared math utilities used across navigation and simulation.` (+16 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **Thin community `sensors pkg`** (1 nodes): `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `motors pkg`** (1 nodes): `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `navigation pkg`** (1 nodes): `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `dashboard pkg`** (1 nodes): `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `scripts pkg`** (1 nodes): `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `run_navigation()` connect `Navigation Controller` to `Motor Drive Layer`, `Brain FSM Core`?**
  _High betweenness centrality (0.342) - this node is a cross-community bridge._
- **Why does `_nav_loop()` connect `Navigation Controller` to `Dashboard REST API`?**
  _High betweenness centrality (0.291) - this node is a cross-community bridge._
- **Why does `navigation/controller.py — Main 20Hz Control Loop` connect `Hardware Components & BOM` to `Dashboard REST API`?**
  _High betweenness centrality (0.176) - this node is a cross-community bridge._
- **Are the 6 inferred relationships involving `DuckDrive` (e.g. with `Motor` and `DuckBot autonomous navigation controller.  Reads sonar, dead-reckons position, r`) actually correct?**
  _`DuckDrive` has 6 INFERRED edges - model-reasoned connections that need verification._
- **Are the 13 inferred relationships involving `run_navigation()` (e.g. with `DuckDrive` and `Sonar`) actually correct?**
  _`run_navigation()` has 13 INFERRED edges - model-reasoned connections that need verification._
- **Are the 3 inferred relationships involving `Perimeter` (e.g. with `DuckBot autonomous navigation controller.  Reads sonar, dead-reckons position, r` and `Run the sense→think→act navigation loop at ~20 Hz.      Args:         on_cycle:`) actually correct?**
  _`Perimeter` has 3 INFERRED edges - model-reasoned connections that need verification._
- **What connects `DuckBot navigation configuration — tune these values for your setup.`, `Perimeter awareness: point-in-polygon, distance-to-edge, bearing-to-center.`, `Dead reckoning: integrates motor speed commands into pose estimate.` to the rest of the system?**
  _21 weakly-connected nodes found - possible documentation gaps or missing edges._