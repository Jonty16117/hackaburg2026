#!/usr/bin/env python3
"""Run navigation scenarios against a live sim server and report results.

Usage:
    python scripts/live_test.py              # real-time, all scenarios
    python scripts/live_test.py --fast       # no delays, batch steps
    python scripts/live_test.py --only dense # filter by name
"""

import json
import sys
import time
import urllib.request
import urllib.error
import argparse

SCENARIOS = [
    ("1. No obstacles", [], 100),
    ("2. Single r=30", [(400, 100, 30)], 100),
    ("3. Three staggered",
     [(300, 60, 10), (400, 140, 10), (600, 100, 10)], 100),
    ("4. Two above+below", [(400, 130, 15), (400, 70, 15)], 100),
    ("5. 7 wall cluster",
     [(500, 70, 15), (500, 90, 15), (500, 110, 15), (500, 130, 15),
      (520, 80, 12), (520, 100, 12), (520, 120, 12)], 100),
    ("6. 8 wall test",
     [(400, 60, 12), (400, 140, 12), (500, 80, 12), (500, 120, 12),
      (600, 100, 15), (650, 70, 10), (650, 130, 10), (400, 100, 20)], 100),
    ("7. Dense 16 L+R",
     [(641, 125, 8), (641, 92, 9), (642, 82, 13), (644, 111, 10),
      (646, 10, 8), (646, 52, 10), (651, 42, 5), (651, 101, 14),
      (733, 77, 8), (733, 186, 11), (735, 97, 9), (737, 169, 9),
      (740, 100, 13), (740, 172, 12), (745, 129, 12), (749, 152, 15)], 500),
    ("8. Dense 16 S->E",
     [(641, 125, 8), (641, 92, 9), (642, 82, 13), (644, 111, 10),
      (646, 10, 8), (646, 52, 10), (651, 42, 5), (651, 101, 14),
      (733, 77, 8), (733, 186, 11), (735, 97, 9), (737, 169, 9),
      (740, 100, 13), (740, 172, 12), (745, 129, 12), (749, 152, 15)], 100),
    ("9. User config",
     [(567, 172, 5), (577, 122, 5), (593, 153, 13), (618, 155, 10),
      (635, 86, 14), (619, 117, 8), (719, 17, 13), (719, 44, 7),
      (719, 77, 8), (719, 112, 6), (719, 130, 9)], 500),
    ("10. Maze easy",
     [(250, 70, 10), (250, 90, 10), (250, 110, 10), (250, 130, 10), (250, 150, 10), (250, 170, 10), (250, 190, 10),
      (450, 10, 10), (450, 30, 10), (450, 50, 10), (450, 70, 10), (450, 90, 10), (450, 110, 10), (450, 130, 10),
      (650, 70, 10), (650, 90, 10), (650, 110, 10), (650, 130, 10), (650, 150, 10), (650, 170, 10), (650, 190, 10)], 30),
    ("11. Maze standard",
     [(250, 60, 10), (250, 80, 10), (250, 100, 10), (250, 120, 10), (250, 140, 10), (250, 160, 10), (250, 180, 10), (250, 200, 10),
      (450, 0, 10), (450, 20, 10), (450, 40, 10), (450, 60, 10), (450, 80, 10), (450, 100, 10), (450, 120, 10), (450, 140, 10),
      (650, 60, 10), (650, 80, 10), (650, 100, 10), (650, 120, 10), (650, 140, 10), (650, 160, 10), (650, 180, 10), (650, 200, 10)], 30),
    ("12. Maze hard",
     [(250, 50, 10), (250, 70, 10), (250, 90, 10), (250, 110, 10), (250, 130, 10), (250, 150, 10), (250, 170, 10), (250, 190, 10),
      (450, 10, 10), (450, 30, 10), (450, 50, 10), (450, 70, 10), (450, 90, 10), (450, 110, 10), (450, 130, 10), (450, 150, 10),
      (650, 50, 10), (650, 70, 10), (650, 90, 10), (650, 110, 10), (650, 130, 10), (650, 150, 10), (650, 170, 10), (650, 190, 10)], 30),
]

MAX_STEPS = 6000
DT = 0.05
BATCH = 20   # steps per request in fast mode


class LiveClient:
    def __init__(self, base_url):
        self.base = base_url.rstrip("/")

    def _api(self, method, path, data=None):
        url = f"{self.base}{path}"
        body = json.dumps(data).encode() if data is not None else None
        req = urllib.request.Request(url, data=body, method=method)
        req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                if resp.status == 200:
                    return json.loads(resp.read())
        except (urllib.error.URLError, urllib.error.HTTPError, json.JSONDecodeError):
            pass
        return None

    def pause_continuous(self):
        self._api("POST", "/api/sim/toggle")

    def resume_continuous(self):
        self._api("POST", "/api/sim/toggle")

    def set_label(self, label):
        self._api("PUT", "/api/config", {"scenario_label": label})

    def apply(self, start_x, obstacles):
        """Reset and load a scenario via load_scenario to restart frame counter."""
        self._api("POST", "/api/duck/reset")
        self._api("DELETE", "/api/obstacles")
        self._api("PUT", "/api/sim/goal",
                  {"start": {"x": start_x, "y": 100}, "end": {"x": 960, "y": 100}})
        for ox, oy, r in obstacles:
            self._api("POST", "/api/obstacles", {"x": ox, "y": oy, "r": r})
        self._api("POST", "/api/autopilot/start")

    def multi_step(self, n):
        """Send multiple step commands as fast as possible."""
        for _ in range(n):
            self._api("POST", "/api/sim/step")

    def state(self):
        s = self._api("GET", "/api/state")
        if s is None:
            return None
        return {
            "frame": s["frame"],
            "x": s["x_cm"],
            "y": s["y_cm"],
            "arrived": s["arrived"],
            "brain_state": s["brain_state"],
        }


def run_scenario(client, desc, obstacles, start_x, fast=False):
    client.pause_continuous()
    client.apply(start_x, obstacles)
    client.set_label(desc)
    s0 = client.state()
    if s0 is None:
        print(f"  {desc:>24s}  ERROR   server unreachable")
        return False, 0, 0.0
    start_frame = s0["frame"]

    step_fn = client.multi_step

    for i in range(0, MAX_STEPS, BATCH if fast else 1):
        step_fn(BATCH if fast else 1)
        if i % (100 if fast else 20) == 0:
            s = client.state()
            if s and s["arrived"]:
                d_frames = s["frame"] - start_frame
                t = d_frames * DT
                print(f"  {desc:>24s}  ARRIVED  {d_frames:>5d}f  {t:>6.1f}s  x={s['x']:.0f}")
                client.resume_continuous()
                return True, d_frames, t
        if not fast:
            time.sleep(DT * 0.8)

    s = client.state()
    d_frames = MAX_STEPS
    print(f"  {desc:>24s}  FAILED   {d_frames:>5d}f  timeout  x={s['x']:.0f}" if s else
          f"  {desc:>24s}  ERROR    state unavailable")
    client.resume_continuous()
    return False, d_frames, d_frames * DT


def main():
    parser = argparse.ArgumentParser(description="Live navigation scenario runner")
    parser.add_argument("--server", default="http://127.0.0.1:8081")
    parser.add_argument("--fast", action="store_true",
                        help="Batch steps (no delays, for quick results)")
    parser.add_argument("--only", type=str,
                        help="Filter scenarios by name substring")
    args = parser.parse_args()

    client = LiveClient(args.server)

    s = client.state()
    if s is None:
        print(f"Server not reachable at {args.server}")
        sys.exit(1)

    filtered = SCENARIOS
    if args.only:
        filtered = [(d, o, sx) for d, o, sx in SCENARIOS
                    if args.only.lower() in d.lower()]
        if not filtered:
            print(f"No scenarios matching '{args.only}'")
            sys.exit(1)

    mode = "fast" if args.fast else "real-time"
    print(f"Live test — {args.server} ({mode})  |  {len(filtered)} scenarios\n")

    passed = 0
    failed = 0
    total_frames = 0
    total_time = 0.0

    t_start = time.monotonic()
    for desc, obstacles, start_x in filtered:
        ok, frames, secs = run_scenario(
            client, desc, obstacles, start_x, fast=args.fast)
        total_frames += frames
        total_time += secs
        if ok:
            passed += 1
        else:
            failed += 1
        if not args.fast:
            time.sleep(1.0)
    t_elapsed = time.monotonic() - t_start

    print(f"\n{passed}/{passed + failed} passed  |  {total_frames}f / {total_time:.1f}s sim-time  |  {t_elapsed:.1f}s wall-clock")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
