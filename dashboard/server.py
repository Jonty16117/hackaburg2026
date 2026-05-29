"""DuckBot dashboard server — FastAPI + SSE + WebSocket + REST API.

Runs a server-side simulation engine (SimEngine) in a background thread
and exposes every dashboard action as a REST API endpoint for agents.

Usage:
    cd hackaburg2026
    uvicorn dashboard.server:app --host 0.0.0.0 --port 8080 --reload
"""

import asyncio
import json
import pathlib
import threading
import time
from datetime import datetime

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, StreamingResponse

HERE = pathlib.Path(__file__).parent

# ---------------------------------------------------------------------------
# Config — fallback when navigation package not importable
# ---------------------------------------------------------------------------
try:
    from navigation.config import (
        LEFT_PIN, RIGHT_PIN, SONAR_TRIG, SONAR_ECHO,
        PERIMETER_CM, START_X_CM, START_Y_CM, START_HEADING_RAD,
        END_X_CM, END_Y_CM, BRAIN_CFG,
    )
except ImportError:
    LEFT_PIN = 12; RIGHT_PIN = 13
    SONAR_TRIG = 23; SONAR_ECHO = 24
    PERIMETER_CM = [(0, 0), (1000, 0), (1000, 200), (0, 200)]
    START_X_CM = 500; START_Y_CM = 100; START_HEADING_RAD = 0.0
    END_X_CM = 900; END_Y_CM = 100
    BRAIN_CFG = {
        "OBSTACLE_THRESHOLD_CM": 50, "PERIMETER_MARGIN_CM": 30,
        "EXPLORE_SPEED": 0.4, "TURN_SPEED": 0.5,
        "MAX_SPEED_CM_S": 100, "WHEEL_BASE_CM": 30,
        "HEADING_TOLERANCE_RAD": 0.26, "LOOP_HZ": 20,
    }

try:
    import RPi.GPIO  # noqa: F401
    HAS_HARDWARE = True
except ImportError:
    HAS_HARDWARE = False

# ---------------------------------------------------------------------------
# App + CORS
# ---------------------------------------------------------------------------
app = FastAPI(title="DuckBot Dashboard")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---------------------------------------------------------------------------
# SimEngine
# ---------------------------------------------------------------------------
from dashboard.sim_engine import SimEngine

_engine = SimEngine()
_engine_lock = threading.Lock()

# ---------------------------------------------------------------------------
# Real-hardware state (used when mode == "real")
# ---------------------------------------------------------------------------
_real_state = {
    "x_cm": float(START_X_CM),
    "y_cm": float(START_Y_CM),
    "theta_rad": float(START_HEADING_RAD),
    "left_speed": 0.0, "right_speed": 0.0,
    "sonar_front": None, "sonar_left": None, "sonar_right": None,
    "brain_state": "IDLE", "avoid_phase": None,
    "inside": True, "edge_cm": 100.0,
    "autopilot": False, "avoid_state": "none", "arrived": False,
    "frame": 0, "obstacles": [], "start": {"x": 500, "y": 100},
    "end": {"x": 900, "y": 100},
    "config": {},
}
_real_lock = threading.Lock()

_mode = "real" if HAS_HARDWARE else "sim"

# ---------------------------------------------------------------------------
# Scenarios (in-memory)
# ---------------------------------------------------------------------------
_scenarios: dict[str, dict] = {}

# ---------------------------------------------------------------------------
# WebSocket clients
# ---------------------------------------------------------------------------
_ws_clients: set[WebSocket] = set()

# ---------------------------------------------------------------------------
# Continuous simulation loop
# ---------------------------------------------------------------------------
_continuous = True

def _sim_loop():
    while True:
        if _continuous:
            with _engine_lock:
                _engine.step(0.05)
        time.sleep(1.0 / 20)

_t = threading.Thread(target=_sim_loop, daemon=True)
_t.start()

# ---------------------------------------------------------------------------
# Real hardware nav loop
# ---------------------------------------------------------------------------
def _nav_loop():
    if not HAS_HARDWARE:
        return
    from navigation.controller import run_navigation
    from navigation.brain import State

    def on_cycle(data):
        d = data["d"]
        brain = data["brain"]
        perim = data["perim"]
        bstate = "IDLE"
        ap = None
        if hasattr(brain, "state"):
            bstate = brain.state.name
        if bstate == "AVOID" and hasattr(brain, "_avoid_phase"):
            ap = brain._avoid_phase.name if brain._avoid_phase else None

        with _real_lock:
            _real_state.update({
                "x_cm": round(data["x"], 1),
                "y_cm": round(data["y"], 1),
                "theta_rad": round(data["theta"], 4),
                "left_speed": round(data["ls"], 4),
                "right_speed": round(data["rs"], 4),
                "sonar_front": round(d, 1) if d is not None else None,
                "brain_state": bstate,
                "avoid_phase": ap,
                "inside": perim.is_inside(data["x"], data["y"]),
                "edge_cm": round(perim.distance_to_edge(data["x"], data["y"]), 1),
            })

    run_navigation(on_cycle=on_cycle)

if HAS_HARDWARE:
    _nav_t = threading.Thread(target=_nav_loop, daemon=True)
    _nav_t.start()

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _read_state():
    if _mode == "real":
        with _real_lock:
            return dict(_real_state)
    with _engine_lock:
        return _engine.get_state()


def _get_config():
    with _engine_lock:
        return _engine.get_config()


def _sim_cmd(method):
    if _mode == "real":
        raise HTTPException(400, "Cannot run sim commands in REAL mode")
    with _engine_lock:
        return method()


def _ensure_sim():
    if _mode == "real":
        raise HTTPException(400, "Cannot run sim commands in REAL mode")


# ---------------------------------------------------------------------------
# WebSocket broadcast
# ---------------------------------------------------------------------------
async def _ws_broadcast():
    while True:
        if _ws_clients:
            state = _read_state()
            msg = json.dumps({"type": "state", "data": state})
            stale = set()
            for ws in _ws_clients:
                try:
                    await ws.send_text(msg)
                except Exception:
                    stale.add(ws)
            for s in stale:
                _ws_clients.discard(s)
        await asyncio.sleep(0.05)


@app.on_event("startup")
async def _startup():
    asyncio.create_task(_ws_broadcast())


# =========================================================================
# LEGACY ENDPOINTS (backward compatible)
# =========================================================================

@app.get("/")
async def index():
    return FileResponse(HERE / "simulator.html")


@app.get("/config")
async def get_config():
    cfg = _get_config()
    return {
        "perimeter_cm": PERIMETER_CM,
        "brain_cfg": BRAIN_CFG,
        "start": {"x": START_X_CM, "y": START_Y_CM, "theta": START_HEADING_RAD},
        "end": {"x": END_X_CM, "y": END_Y_CM},
        "sim": cfg,
    }


@app.post("/push")
async def push_telemetry(request: Request):
    """Deprecated — kept for backward compatibility."""
    await request.json()
    return {"ok": True}


@app.get("/live")
async def get_live():
    state = _read_state()
    if _mode == "sim":
        with _engine_lock:
            debug = _engine.get_debug(50)
        return {"latest": state, "frame_count": len(debug["frames"]), "buffer": debug["frames"]}
    return {"latest": state, "frame_count": 0, "buffer": []}


@app.get("/stream")
async def stream(request: Request):
    async def gen():
        last = {}
        while True:
            if await request.is_disconnected():
                break
            state = _read_state()
            if state != last:
                yield f"data: {json.dumps(state)}\n\n"
                last = state
            await asyncio.sleep(0.05)

    return StreamingResponse(
        gen(), media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


# =========================================================================
# WEB SOCKET
# =========================================================================

_COMMAND_HANDLERS = {}


def _ws_handler(cmd_type):
    def wrapper(fn):
        _COMMAND_HANDLERS[cmd_type] = fn
        return fn
    return wrapper


@_ws_handler("set_pose")
async def _ws_set_pose(ws, data):
    _ensure_sim()
    with _engine_lock:
        return _engine.set_pose(data.get("x"), data.get("y"), data.get("theta"))


@_ws_handler("set_speeds")
async def _ws_set_speeds(ws, data):
    _ensure_sim()
    with _engine_lock:
        return _engine.set_speeds(data.get("left", 0), data.get("right", 0))


@_ws_handler("set_sonar")
async def _ws_set_sonar(ws, data):
    _ensure_sim()
    with _engine_lock:
        return _engine.set_sonar_override(data.get("front"), data.get("left"), data.get("right"))


@_ws_handler("autopilot_start")
async def _ws_autopilot_start(ws, data):
    _ensure_sim()
    with _engine_lock:
        return _engine.set_autopilot(True)


@_ws_handler("autopilot_stop")
async def _ws_autopilot_stop(ws, data):
    _ensure_sim()
    with _engine_lock:
        return _engine.set_autopilot(False)


@_ws_handler("reset")
async def _ws_reset(ws, data):
    _ensure_sim()
    with _engine_lock:
        return _engine.reset()


@_ws_handler("add_obstacle")
async def _ws_add_obs(ws, data):
    _ensure_sim()
    with _engine_lock:
        oid = _engine.add_obstacle(data.get("x", 0), data.get("y", 0), data.get("r"))
        return {"id": oid}


@_ws_handler("remove_obstacle")
async def _ws_rm_obs(ws, data):
    _ensure_sim()
    with _engine_lock:
        ok = _engine.remove_obstacle(data.get("id"))
        return {"ok": ok}


@_ws_handler("clear_obstacles")
async def _ws_clear_obs(ws, data):
    _ensure_sim()
    with _engine_lock:
        _engine.clear_obstacles()
        return {"ok": True}


@_ws_handler("set_goal")
async def _ws_set_goal(ws, data):
    _ensure_sim()
    with _engine_lock:
        return _engine.set_goal(data.get("start"), data.get("end"))


@_ws_handler("sim_step")
async def _ws_sim_step(ws, data):
    _ensure_sim()
    with _engine_lock:
        _engine.step(data.get("dt", 0.05))
        return _engine._build_state()


@_ws_handler("sim_toggle")
async def _ws_sim_toggle(ws, data):
    global _continuous
    _ensure_sim()
    _continuous = not _continuous
    return {"continuous": _continuous}


@_ws_handler("set_config")
async def _ws_set_config(ws, data):
    _ensure_sim()
    with _engine_lock:
        _engine.update_config(data)
        return _engine._build_config()


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    _ws_clients.add(websocket)
    try:
        while True:
            raw = await websocket.receive_text()
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                await websocket.send_json({"type": "error", "detail": "invalid JSON"})
                continue

            cmd = msg.get("type", "")
            handler = _COMMAND_HANDLERS.get(cmd)
            if handler is None:
                await websocket.send_json({"type": "error", "detail": f"unknown command: {cmd}"})
                continue

            try:
                result = await handler(websocket, msg)
                await websocket.send_json({"type": "ack", "command": cmd, "data": result})
            except HTTPException as e:
                await websocket.send_json({"type": "error", "command": cmd, "detail": e.detail})
            except Exception as e:
                await websocket.send_json({"type": "error", "command": cmd, "detail": str(e)})
    except WebSocketDisconnect:
        pass
    finally:
        _ws_clients.discard(websocket)


# =========================================================================
# REST API — State
# =========================================================================

@app.get("/api/state")
def api_get_state():
    return _read_state()


@app.get("/api/state/history")
def api_get_state_history(limit: int = 50):
    if _mode == "real":
        return {"frames": []}
    with _engine_lock:
        return _engine.get_debug(limit)


# =========================================================================
# REST API — Duck Control
# =========================================================================

@app.put("/api/duck/pose")
def api_set_pose(data: dict):
    _ensure_sim()
    with _engine_lock:
        return _engine.set_pose(data.get("x"), data.get("y"), data.get("theta"))


@app.post("/api/duck/reset")
def api_reset():
    _ensure_sim()
    with _engine_lock:
        return _engine.reset()


@app.put("/api/duck/speeds")
def api_set_speeds(data: dict):
    _ensure_sim()
    with _engine_lock:
        return _engine.set_speeds(data.get("left", 0), data.get("right", 0))


# =========================================================================
# REST API — Sensors
# =========================================================================

@app.put("/api/sensors/sonar")
def api_set_sonar(data: dict):
    _ensure_sim()
    with _engine_lock:
        return _engine.set_sonar_override(
            data.get("front"), data.get("left"), data.get("right"),
        )


# =========================================================================
# REST API — Autopilot
# =========================================================================

@app.post("/api/autopilot/start")
def api_autopilot_start():
    _ensure_sim()
    with _engine_lock:
        return _engine.set_autopilot(True)


@app.post("/api/autopilot/stop")
def api_autopilot_stop():
    _ensure_sim()
    with _engine_lock:
        return _engine.set_autopilot(False)


# =========================================================================
# REST API — Obstacles
# =========================================================================

@app.get("/api/obstacles")
def api_get_obstacles():
    if _mode == "real":
        return []
    with _engine_lock:
        return list(_engine.obstacles)


@app.post("/api/obstacles")
def api_add_obstacle(data: dict):
    _ensure_sim()
    with _engine_lock:
        oid = _engine.add_obstacle(data.get("x", 0), data.get("y", 0), data.get("r"))
        return {"id": oid}


@app.delete("/api/obstacles/{oid}")
def api_remove_obstacle(oid: int):
    _ensure_sim()
    with _engine_lock:
        if not _engine.remove_obstacle(oid):
            raise HTTPException(404, f"Obstacle {oid} not found")
        return {"ok": True}


@app.delete("/api/obstacles")
def api_clear_obstacles():
    _ensure_sim()
    with _engine_lock:
        _engine.clear_obstacles()
        return {"ok": True}


# =========================================================================
# REST API — Configuration
# =========================================================================

@app.get("/api/config")
def api_get_config():
    return _get_config()


@app.put("/api/config")
def api_update_config(data: dict):
    _ensure_sim()
    with _engine_lock:
        _engine.update_config(data)
        return _engine._build_config()


# =========================================================================
# REST API — Simulation Control
# =========================================================================

@app.post("/api/sim/step")
def api_sim_step(data: dict = {}):
    _ensure_sim()
    with _engine_lock:
        _engine.step(data.get("dt", 0.05))
        return _engine._build_state()


@app.post("/api/sim/toggle")
def api_sim_toggle():
    global _continuous
    _ensure_sim()
    _continuous = not _continuous
    return {"continuous": _continuous}


@app.put("/api/sim/goal")
def api_set_goal(data: dict):
    _ensure_sim()
    with _engine_lock:
        return _engine.set_goal(data.get("start"), data.get("end"))


@app.get("/api/sim/debug")
def api_get_debug():
    if _mode == "real":
        return {"frames": []}
    with _engine_lock:
        return _engine.get_debug(500)


# =========================================================================
# REST API — Scenarios
# =========================================================================

@app.get("/api/scenarios")
def api_list_scenarios():
    return [
        {
            "name": name,
            "obstacles_count": len(s.get("obstacles", [])),
            "created_at": s.get("created_at"),
        }
        for name, s in _scenarios.items()
    ]


@app.post("/api/scenarios")
def api_save_scenario(data: dict):
    name = data.get("name")
    if not name:
        raise HTTPException(400, "Scenario name is required")
    scenario = {
        "name": name,
        "config": data.get("config"),
        "obstacles": data.get("obstacles"),
        "start": data.get("start"),
        "end": data.get("end"),
        "created_at": datetime.utcnow().isoformat(),
    }
    _scenarios[name] = scenario
    return {"name": name}


@app.get("/api/scenarios/{name}")
def api_get_scenario(name: str):
    s = _scenarios.get(name)
    if s is None:
        raise HTTPException(404, f"Scenario '{name}' not found")
    return s


@app.delete("/api/scenarios/{name}")
def api_delete_scenario(name: str):
    if name not in _scenarios:
        raise HTTPException(404, f"Scenario '{name}' not found")
    del _scenarios[name]
    return {"ok": True}


@app.post("/api/scenarios/{name}/apply")
def api_apply_scenario(name: str):
    _ensure_sim()
    s = _scenarios.get(name)
    if s is None:
        raise HTTPException(404, f"Scenario '{name}' not found")
    with _engine_lock:
        return _engine.load_scenario(s)


# =========================================================================
# REST API — Mode
# =========================================================================

@app.get("/api/mode")
def api_get_mode():
    return {"mode": _mode, "hardware_available": HAS_HARDWARE}


@app.put("/api/mode")
def api_set_mode(data: dict):
    global _mode
    new_mode = data.get("mode", "sim")
    if new_mode not in ("sim", "real"):
        raise HTTPException(400, "mode must be 'sim' or 'real'")
    if new_mode == "real" and not HAS_HARDWARE:
        raise HTTPException(400, "Hardware not available")
    _mode = new_mode
    return {"mode": _mode}
