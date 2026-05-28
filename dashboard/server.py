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
    import RPi.GPIO as GPIO
    from motors.drive import DuckDrive
    from sensors.sonar import Sonar
    from navigation.config import (
        LEFT_PIN, RIGHT_PIN, SONAR_TRIG, SONAR_ECHO,
        PERIMETER_CM, START_X_CM, START_Y_CM, START_HEADING_RAD, BRAIN_CFG,
    )
    from navigation.odometry import Odometry
    from navigation.perimeter import Perimeter
    from navigation.brain import Brain, State
    HAS_HARDWARE = True
except ImportError:
    HAS_HARDWARE = False

app = FastAPI(title="DuckBot Dashboard")

_state = {
    "x_cm": START_X_CM if HAS_HARDWARE else 500,
    "y_cm": START_Y_CM if HAS_HARDWARE else 100,
    "theta_rad": START_HEADING_RAD if HAS_HARDWARE else 0.0,
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
        "perimeter_cm": PERIMETER_CM if HAS_HARDWARE else [[0, 0], [1000, 0], [1000, 200], [0, 200]],
        "brain_cfg": BRAIN_CFG if HAS_HARDWARE else {
            "OBSTACLE_THRESHOLD_CM": 50, "PERIMETER_MARGIN_CM": 30,
            "EXPLORE_SPEED": 0.4, "TURN_SPEED": 0.5,
            "MAX_SPEED_CM_S": 100, "WHEEL_BASE_CM": 30,
            "HEADING_TOLERANCE_RAD": 0.26,
        },
        "start": {
            "x": START_X_CM if HAS_HARDWARE else 500,
            "y": START_Y_CM if HAS_HARDWARE else 100,
            "theta": START_HEADING_RAD if HAS_HARDWARE else 0,
        },
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
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    drive = DuckDrive(LEFT_PIN, RIGHT_PIN)
    sonar = Sonar(SONAR_TRIG, SONAR_ECHO)
    odom = Odometry(START_X_CM, START_Y_CM, START_HEADING_RAD,
                    BRAIN_CFG["WHEEL_BASE_CM"], BRAIN_CFG["MAX_SPEED_CM_S"])
    perim = Perimeter(PERIMETER_CM)
    brain = Brain(perim, BRAIN_CFG)
    ll, lr = 0.0, 0.0
    lt = time.time()
    try:
        while True:
            now = time.time()
            dt = max(now - lt, 0.001)
            lt = now
            odom.update(ll, lr, dt)
            d = sonar.distance_cm()
            x, y, theta = odom.position()
            ls, rs = brain.decide(d, x, y, theta, dt)
            drive.drive_speeds(ls, rs)
            ll, lr = ls, rs
            ap = brain._avoid_phase.name if brain.state == State.AVOID else None
            with _lock:
                _state.update({
                    "x_cm": round(x, 1), "y_cm": round(y, 1),
                    "theta_rad": round(theta, 4),
                    "left_speed": round(ls, 4), "right_speed": round(rs, 4),
                    "sonar_front": round(d, 1) if d else None,
                    "brain_state": brain.state.name, "avoid_phase": ap,
                    "inside": perim.is_inside(x, y),
                    "edge_cm": round(perim.distance_to_edge(x, y), 1),
                })
            time.sleep(1.0 / BRAIN_CFG["LOOP_HZ"])
    except Exception as e:
        print(f"Nav error: {e}")
    finally:
        drive.stop()
        drive.cleanup()
        sonar.cleanup()
        GPIO.cleanup()


if HAS_HARDWARE:
    _t = threading.Thread(target=_nav_loop, daemon=True)
    _t.start()
