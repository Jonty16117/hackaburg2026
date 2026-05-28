"""DuckBot dashboard server — FastAPI + SSE telemetry stream.

Runs navigation loop (sonar → brain → motors) in a background thread
and streams live state to browsers via Server-Sent Events.

Usage (on Pi):
    cd hackaburg2026
    source .venv/bin/activate
    uvicorn dashboard.server:app --host 0.0.0.0 --port 8080

Simulation mode: open http://<pi-ip>:8080 in any browser.
No hardware needed — just pick "Simulation" mode.
"""

import asyncio
import json
import pathlib
import threading
import time

from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, StreamingResponse

HERE = pathlib.Path(__file__).parent

try:
    from navigation.config import (
        LEFT_PIN, RIGHT_PIN, SONAR_TRIG, SONAR_ECHO,
        PERIMETER_CM, START_X_CM, START_Y_CM, START_HEADING_RAD,
        END_X_CM, END_Y_CM, BRAIN_CFG,
    )
    from navigation.odometry import Odometry
    from navigation.perimeter import Perimeter
    from navigation.brain import Brain, State
except ImportError:
    LEFT_PIN = 12; RIGHT_PIN = 13
    SONAR_TRIG = 23; SONAR_ECHO = 24
    PERIMETER_CM = [(0, 0), (1000, 0), (1000, 200), (0, 200)]
    START_X_CM = 500; START_Y_CM = 100; START_HEADING_RAD = 0.0
    END_X_CM = 900; END_Y_CM = 100
    BRAIN_CFG = {"OBSTACLE_THRESHOLD_CM": 50, "PERIMETER_MARGIN_CM": 30,
                 "EXPLORE_SPEED": 0.4, "TURN_SPEED": 0.5,
                 "MAX_SPEED_CM_S": 100, "WHEEL_BASE_CM": 30,
                 "HEADING_TOLERANCE_RAD": 0.26, "LOOP_HZ": 20}
    Odometry = None; Perimeter = None; Brain = None; State = None

try:
    import RPi.GPIO
    HAS_HARDWARE = True
except ImportError:
    HAS_HARDWARE = False

app = FastAPI(title="DuckBot Dashboard")

_state = {
    "x_cm": START_X_CM,
    "y_cm": START_Y_CM,
    "theta_rad": START_HEADING_RAD,
    "left_speed": 0.0, "right_speed": 0.0,
    "sonar_front": None, "sonar_left": None, "sonar_right": None,
    "brain_state": "IDLE", "avoid_phase": None,
    "inside": True, "edge_cm": 100.0,
}
_lock = threading.Lock()


@app.get("/")
async def index():
    return FileResponse(HERE / "simulator.html")


@app.get("/config")
async def get_config():
    return {
        "perimeter_cm": PERIMETER_CM,
        "brain_cfg": BRAIN_CFG,
        "start": {"x": START_X_CM, "y": START_Y_CM,
                  "theta": START_HEADING_RAD},
        "end": {"x": END_X_CM, "y": END_Y_CM},
    }


@app.get("/stream")
async def stream(request: Request):
    async def gen():
        last = dict(_state)
        while True:
            if await request.is_disconnected():
                break
            with _lock:
                data = dict(_state)
            if data != last:
                yield f"data: {json.dumps(data)}\n\n"
                last = data
            await asyncio.sleep(0.05)
    return StreamingResponse(
        gen(), media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


def _nav_loop():
    if not HAS_HARDWARE:
        return
    from navigation.controller import run_navigation

    def on_cycle(data):
        d = data["d"]
        brain = data["brain"]
        perim = data["perim"]
        ap = brain._avoid_phase.name if brain.state == State else None
        with _lock:
            _state.update({
                "x_cm": round(data["x"], 1),
                "y_cm": round(data["y"], 1),
                "theta_rad": round(data["theta"], 4),
                "left_speed": round(data["ls"], 4),
                "right_speed": round(data["rs"], 4),
                "sonar_front": round(d, 1) if d else None,
                "brain_state": brain.state.name,
                "avoid_phase": ap,
                "inside": perim.is_inside(data["x"], data["y"]),
                "edge_cm": round(perim.distance_to_edge(data["x"], data["y"]), 1),
            })

    run_navigation(on_cycle=on_cycle)


if HAS_HARDWARE:
    _t = threading.Thread(target=_nav_loop, daemon=True)
    _t.start()
