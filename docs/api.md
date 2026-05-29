# DuckBot Simulation API

The DuckBot dashboard exposes every simulation action as a REST API endpoint
so AI agents and CLI tools can programmatically control, observe, and debug
the duck without a browser.

## Quick Start

```bash
# Start the server
uvicorn dashboard.server:app --host 0.0.0.0 --port 8080

# Start autopilot (duck navigates toward goal marker autonomously)
curl -X POST http://localhost:8080/api/autopilot/start

# Check current state
curl http://localhost:8080/api/state | jq .

# Place an obstacle in the pool
curl -X POST http://localhost:8080/api/obstacles \
  -H 'Content-Type: application/json' \
  -d '{"x": 500, "y": 80, "r": 25}'

# Override the front sonar reading (simulate a close obstacle)
curl -X PUT http://localhost:8080/api/sensors/sonar \
  -H 'Content-Type: application/json' \
  -d '{"front": 30}'

# Step simulation one frame
curl -X POST http://localhost:8080/api/sim/step \
  -H 'Content-Type: application/json' \
  -d '{}'

# Save a scenario
curl -X POST http://localhost:8080/api/scenarios \
  -H 'Content-Type: application/json' \
  -d '{"name": "obstacle_course", "obstacles": [{"x": 500, "y": 80, "r": 20}]}'

# Load a scenario
curl -X POST http://localhost:8080/api/scenarios/obstacle_course/apply

# Stream real-time state via WebSocket
wscat -c ws://localhost:8080/ws
```

---

## State & History

### `GET /api/state`

Full duck state as a JSON object.

**Response fields:**

| Field | Type | Description |
|-------|------|-------------|
| `x_cm` | float | X position in cm (0-1000) |
| `y_cm` | float | Y position in cm (0-200) |
| `theta_rad` | float | Heading in radians (-π to π) |
| `left_speed` | float | Left motor speed (-1.0 to 1.0) |
| `right_speed` | float | Right motor speed (-1.0 to 1.0) |
| `sonar_front` | float? | Front sonar reading in cm, or null |
| `sonar_left` | float? | Left sonar reading, or null |
| `sonar_right` | float? | Right sonar reading, or null |
| `autopilot` | bool | Whether autopilot FSM is active |
| `brain_state` | string | Current state label: `IDLE`, `GO`, `REVERSE`, `SCAN`, `FACE_BEST`, `COOLDOWN`, `PERIM`, `ARRIVED` |
| `avoid_state` | string | Low-level FSM phase: `none`, `reverse`, `scan`, `face_best`, `cooldown` |
| `arrived` | bool | Whether duck has reached the goal |
| `inside` | bool | Whether duck is inside perimeter bounds |
| `edge_cm` | float | Distance to nearest perimeter edge |
| `frame` | int | Frame counter (increments each simulation tick) |
| `obstacles` | list | `[{id, x, y, r}, ...]` |
| `start` | object | `{x, y}` start marker position |
| `end` | object | `{x, y}` goal marker position |
| `trail` | list | `[{x, y}, ...]` last 40 positions (for trail rendering) |
| `config` | object | Current simulation config values |

**Example:**

```json
{
  "x_cm": 520.3,
  "y_cm": 100.0,
  "theta_rad": 0.15,
  "left_speed": 0.4,
  "right_speed": 0.5,
  "sonar_front": 200.0,
  "sonar_left": 300.0,
  "sonar_right": 400.0,
  "autopilot": true,
  "brain_state": "GO",
  "avoid_state": "none",
  "arrived": false,
  "inside": true,
  "edge_cm": 30.0,
  "frame": 42,
  "obstacles": [{"id": 0, "x": 500, "y": 80, "r": 20}],
  "start": {"x": 500, "y": 100},
  "end": {"x": 900, "y": 100},
  "trail": [{"x": 500, "y": 100}, {"x": 500.3, "y": 100}, ...],
  "config": {
    "PW": 1000, "PH": 200, "DUCK_R": 15, "MARGIN": 30,
    "OBST_TH": 25, "TURN_SP": 0.5, "MAX_SPD": 100, "WB": 30,
    "HDG_TOL": 0.02, "max_speed": 0.6, ...
  }
}
```

### `GET /api/state/history?limit=50`

Latest debug frames from the simulation buffer.

**Response:**
```json
{
  "frames": [
    {
      "t": "0.05", "x": "500.0", "y": "100.0", "th": "0.0",
      "ls": "0.40", "rs": "0.50",
      "f": "200", "l": "300", "r": "400",
      "near": "30", "perimEscaping": false,
      "avoidState": "none", "perimCooldown": "0.00"
    }
  ]
}
```

---

## Duck Control

### `PUT /api/duck/pose`

Teleport the duck to a specific position. Resets autopilot and clears stuck/avoid state.

**Request:**
```json
{"x": 500, "y": 100, "theta": 0}
```
`theta` is optional (defaults to 0).

**Response:** Full state after the move.

### `POST /api/duck/reset`

Reset duck to the start marker position. Clears trail, debug buffer, autopilot state, and zeroes speeds.

**Response:** Full state after reset.

### `PUT /api/duck/speeds`

Set motor speeds directly (manual control). Disables autopilot.

**Request:**
```json
{"left": 0.5, "right": 0.5}
```
Values are clamped to -1.0 to 1.0.

**Response:** Full state.

---

## Sensors

### `PUT /api/sensors/sonar`

Override one or more sonar readings. Overrides persist until cleared (set to `null`).

- `null` = use simulated raycasting
- number = fixed value

**Request:**
```json
{"front": 30, "left": null, "right": null}
```

**Response:** Full state.

---

## Autopilot

### `POST /api/autopilot/start`

Enable the autonomous navigation FSM. The duck will:
1. Head toward the goal (end marker)
2. Reverse + scan when it detects an obstacle (sonar < OBST_TH + DUCK_R)
3. Escape from perimeter edges
4. Detect when it's stuck (hasn't moved much in 1.5s)
5. Stop when it reaches the goal (ARRIVAL distance)

**Response:** Full state (autopilot = true, speeds zeroed).

### `POST /api/autopilot/stop`

Disable autopilot and zero motor speeds.

**Response:** Full state (autopilot = false, speeds = 0).

---

## Obstacles

### `GET /api/obstacles`

List all obstacles.

**Response:** `[{"id": 0, "x": 500, "y": 80, "r": 20}]`

### `POST /api/obstacles`

Add a circular obstacle. `r` defaults to 10 if not specified.

**Request:**
```json
{"x": 500, "y": 80, "r": 20}
```

**Response:** `{"id": 1}`

### `DELETE /api/obstacles/{id}`

Remove an obstacle by ID.

**Response:** `{"ok": true}` or 404

### `DELETE /api/obstacles`

Remove all obstacles.

**Response:** `{"ok": true}`

---

## Configuration

### `GET /api/config`

Current simulation configuration.

**Response:**
```json
{
  "PW": 1000, "PH": 200, "DUCK_R": 15, "MARGIN": 30,
  "ARRIVAL": 20, "OBST_TH": 25, "TURN_SP": 0.5,
  "REVERSE_SPD": 0.5, "MAX_SPD": 100, "WB": 30,
  "HDG_TOL": 0.02, "AVOID_REVERSE_S": 0.5,
  "AVOID_COOLDOWN_S": 0.3, "STUCK_DIST_CM": 8,
  "STUCK_WINDOW_S": 1.5, "max_speed": 0.6
}
```

### `PUT /api/config`

Update configuration values. Only provided keys are changed (partial update).

**Request:**
```json
{"max_speed": 0.8, "OBST_TH": 40}
```

**Response:** Full config dict after merge.

---

## Simulation Control

### `POST /api/sim/step`

Advance simulation by one physics tick (~20Hz, 50ms default). Only available in sim mode.

**Request:**
```json
{"dt": 0.05}
```
`dt` is optional (defaults to 0.05).

**Response:** Full state after the step.

### `POST /api/sim/toggle`

Toggle the continuous simulation loop on/off. When on (default), the sim runs at ~20Hz automatically. When off, only explicit `POST /api/sim/step` calls advance the simulation.

**Response:** `{"continuous": true}` or `{"continuous": false}`

### `PUT /api/sim/goal`

Move the start and/or end marker positions.

- Setting `start` also teleports the duck to the new start position.
- Setting `end` only moves the goal marker.

**Request:**
```json
{"start": {"x": 100, "y": 100}, "end": {"x": 900, "y": 100}}
```
Either `start` or `end` can be omitted.

**Response:** Full state.

### `GET /api/sim/debug`

Full debug buffer (last 500 frames) as downloadable JSON.

**Response:** `{"frames": [...]}`

---

## Scenarios

Scenarios let you save and restore full simulation setups (config, obstacles, markers) for reproducible testing. Scenarios are stored in memory and lost on server restart.

### `GET /api/scenarios`

List saved scenarios.

**Response:**
```json
[{"name": "obstacle_course", "obstacles_count": 3, "created_at": "2026-05-29T12:00:00"}]
```

### `POST /api/scenarios`

Save the current (or custom) setup as a named scenario.

**Request:**
```json
{
  "name": "obstacle_course",
  "config": {"max_speed": 0.8, "OBST_TH": 40},
  "obstacles": [{"x": 500, "y": 80, "r": 20}, {"x": 300, "y": 120, "r": 15}],
  "start": {"x": 100, "y": 100},
  "end": {"x": 900, "y": 100}
}
```
All fields except `name` are optional.

**Response:** `{"name": "obstacle_course"}`

### `GET /api/scenarios/{name}`

Get a saved scenario's data.

**Response:** The full scenario dict, or 404.

### `POST /api/scenarios/{name}/apply`

Load a scenario into the simulation engine. This:
- Updates config parameters
- Replaces all obstacles
- Moves start/end markers
- Resets the duck to start position
- Clears trail and debug buffer

**Response:** Full state after loading.

### `DELETE /api/scenarios/{name}`

Delete a saved scenario.

**Response:** `{"ok": true}` or 404.

---

## Mode

### `GET /api/mode`

**Response:**
```json
{"mode": "sim", "hardware_available": false}
```

### `PUT /api/mode`

Switch between simulation and real hardware mode. Only available if hardware (RPi.GPIO) is detected.

**Request:**
```json
{"mode": "real"}
```

**Response:** `{"mode": "real"}`

---

## Legacy Endpoints (kept for backward compatibility)

| Method | Path | Notes |
|--------|------|-------|
| `GET` | `/` | Serves the dashboard HTML |
| `GET` | `/config` | Perimeter + brain config from `navigation/config.py` |
| `POST` | `/push` | Deprecated no-op |
| `GET` | `/live` | Latest state + buffer (same as `/api/state` + `/api/state/history`) |
| `GET` | `/stream` | SSE — pushes state changes as they happen |

---

## WebSocket (`ws://localhost:8080/ws`)

Bidirectional real-time communication.

### Server → Client (20Hz push)

```json
{"type": "state", "data": { /* full state object */ }}
```

### Client → Server (commands)

| `type` | Fields | Equivalent REST |
|--------|--------|-----------------|
| `set_pose` | `x`, `y`, `theta?` | `PUT /api/duck/pose` |
| `set_speeds` | `left`, `right` | `PUT /api/duck/speeds` |
| `set_sonar` | `front?`, `left?`, `right?` | `PUT /api/sensors/sonar` |
| `autopilot_start` | — | `POST /api/autopilot/start` |
| `autopilot_stop` | — | `POST /api/autopilot/stop` |
| `reset` | — | `POST /api/duck/reset` |
| `add_obstacle` | `x`, `y`, `r?` | `POST /api/obstacles` |
| `remove_obstacle` | `id` | `DELETE /api/obstacles/{id}` |
| `clear_obstacles` | — | `DELETE /api/obstacles` |
| `set_goal` | `start?`, `end?` | `PUT /api/sim/goal` |
| `sim_step` | `dt?` | `POST /api/sim/step` |
| `sim_toggle` | — | `POST /api/sim/toggle` |
| `set_config` | `{key: value, ...}` | `PUT /api/config` |

**Example** — send a command via WebSocket with `wscat`:
```
wscat -c ws://localhost:8080/ws
{"type": "set_pose", "x": 500, "y": 50}
{"type": "autopilot_start"}
```

---

## Autopilot FSM Reference

The simulation autopilot is a finite state machine ported from the browser UI.
It mirrors this behaviour:

```
                     ┌──────────┐
              ┌─────▶│    GO    │
              │      │  (goal-  │
              │      │ seeking) │
              │      └────┬─────┘
              │           │
              │     sonar < threshold    │ near edge
              │           ▼                     ▼
              │     ┌──────────┐      ┌──────────────┐
              │     │ REVERSE  │      │   PERIMETER  │
              │     │  (0.5s)  │      │    ESCAPE    │
              │     └────┬─────┘      └──────┬───────┘
              │          │                    │
              │          ▼                    │  far enough
              │     ┌──────────┐              │  from edge
              │     │   SCAN   │              │
              │     │ (sweep,  │              │
              │     │  pick    │              │
              │     │  best)   │              │
              │     └────┬─────┘              │
              │          ▼                    │
              │     ┌──────────┐              │
              │     │ FACE_BEST│              │
              │     └────┬─────┘              │
              │          ▼                    │
              │     ┌──────────┐              │
              │     │ COOLDOWN │              │
              │     │  (0.3s)  │──────────────┘
              │     └──────────┘
              │
              │  stuck (moved < 8cm in 1.5s) → REVERSE
              │  arrived at goal → stop
              └──────────────────────────────────────
```

### Config parameters that control the FSM:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `OBST_TH` | 25 | Sonar distance (cm) below which obstacle is detected |
| `TURN_SP` | 0.5 | Turn-in-place speed |
| `REVERSE_SPD` | 0.5 | Reverse speed during avoidance |
| `MAX_SPD` | 100 | Max linear velocity (cm/s) |
| `WB` | 30 | Wheel base (cm) — affects turn rate |
| `HDG_TOL` | 0.02 | Heading tolerance (rad) before goal-seeking drives straight |
| `AVOID_REVERSE_S` | 0.5 | How long to reverse before scanning |
| `AVOID_COOLDOWN_S` | 0.3 | Cooldown after facing best direction |
| `STUCK_DIST_CM` | 8 | Distance threshold for stuck detection |
| `STUCK_WINDOW_S` | 1.5 | Time window for stuck detection |
| `ARRIVAL` | 20 | Distance (cm) from goal to trigger arrival |
| `MARGIN` | 30 | Distance from pool edge to trigger perimeter escape |
| `DUCK_R` | 15 | Duck collision radius (cm) |
| `max_speed` | 0.6 | Goal-seeking forward speed (-1.0 to 1.0) (Note: lowercase, Python attribute name) |

---

## Agent Workflows

### Workflow 1: Run a predefined test

```bash
# 1. Load a scenario
curl -X POST http://localhost:8080/api/scenarios/obstacle_test/apply

# 2. Start autopilot
curl -X POST http://localhost:8080/api/autopilot/start

# 3. Poll until arrived or stuck
while true; do
  state=$(curl -s http://localhost:8080/api/state)
  echo "$state" | jq '{x: .x_cm, y: .y_cm, state: .brain_state, frame: .frame}'
  if echo "$state" | jq -e '.arrived' > /dev/null; then
    echo "DUCK ARRIVED!"
    break
  fi
  sleep 0.5
done

# 4. Dump debug data
curl http://localhost:8080/api/sim/debug > test-run-1.json
```

### Workflow 2: Step-by-step debugging

```bash
# 1. Pause continuous simulation
curl -X POST http://localhost:8080/api/sim/toggle

# 2. Place duck and obstacle
curl -X PUT http://localhost:8080/api/duck/pose \
  -H 'Content-Type: application/json' -d '{"x": 200, "y": 100, "theta": 0}'
curl -X POST http://localhost:8080/api/obstacles \
  -H 'Content-Type: application/json' -d '{"x": 400, "y": 100, "r": 25}'

# 3. Start autopilot
curl -X POST http://localhost:8080/api/autopilot/start

# 4. Step frame by frame, observing state
for i in $(seq 1 100); do
  curl -s -X POST http://localhost:8080/api/sim/step \
    -H 'Content-Type: application/json' -d '{}' \
    | jq '{x: .x_cm, y: .y_cm, state: .brain_state, avoid: .avoid_state, f: .sonar_front}'
  sleep 0.1
done
```

### Workflow 3: Sensor injection attack

```bash
# Force the duck to think there's always a clear path ahead
curl -X PUT http://localhost:8080/api/sensors/sonar \
  -H 'Content-Type: application/json' \
  -d '{"front": 999, "left": 999, "right": 999}'

# Or force it to think it's about to hit something
curl -X PUT http://localhost:8080/api/sensors/sonar \
  -H 'Content-Type: application/json' \
  -d '{"front": 20}'
```

### Workflow 4: Parameter sweep

```bash
for speed in 0.3 0.5 0.7 1.0; do
  curl -X PUT http://localhost:8080/api/config \
    -H 'Content-Type: application/json' \
    -d "{\"max_speed\": $speed, \"OBST_TH\": 40}"
  curl -X POST http://localhost:8080/api/duck/reset
  curl -X POST http://localhost:8080/api/autopilot/start
  sleep 5
  curl -X POST http://localhost:8080/api/autopilot/stop
  state=$(curl -s http://localhost:8080/api/state)
  echo "speed=$speed x=$(echo $state | jq -r '.x_cm') y=$(echo $state | jq -r '.y_cm') arrived=$(echo $state | jq -r '.arrived')"
done
```

---

## Integration with the Real Robot

When hardware (Raspberry Pi with RPi.GPIO) is detected, the server runs in
REAL mode by default. In REAL mode:

- Sim commands return 400 errors
- State is read from the real navigation controller
- Use `GET /stream` for SSE telemetry from the real robot
- To test scenarios, switch to SIM mode: `PUT /api/mode {"mode": "sim"}`

---

## Simulation Engine Notes

- Physics runs at ~20 Hz (50ms per tick)
- The sim loop starts on server boot (can be paused with `POST /api/sim/toggle`)
- Obstacle collision uses duck radius + obstacle radius
- Position is clamped to pool bounds (DUCK_R margin from each edge)
- Sonar raycasting checks perimeter walls and obstacles
- Debug buffer stores up to 500 frames (oldest discarded)
- Trail stores up to 40 positions for visual rendering
