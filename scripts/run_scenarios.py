"""Run benchmark scenarios one at a time via REST API.
Open http://localhost:8080 in your browser to watch."""

import json, sys, time, urllib.request

HOST = "http://localhost:8080"
API = f"{HOST}/api"

def req(method, path, body=None):
    url = f"{API}{path}"
    data = json.dumps(body).encode() if body else None
    r = urllib.request.Request(url, data=data, method=method)
    r.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(r, timeout=600) as resp:
        return json.loads(resp.read())

def pause():
    """Ensure sim loop is paused, return previous state."""
    cur = req("POST", "/sim/get_continuous")
    if cur.get("continuous"):
        req("POST", "/sim/toggle")
    return True

def resume():
    """Ensure sim loop is running."""
    cur = req("POST", "/sim/get_continuous")
    if not cur.get("continuous"):
        req("POST", "/sim/toggle")
    return True

def run(desc, obstacles, start=(30, 100), end=(960, 100), timeout_s=300):
    pause()
    req("PUT", "/sim/label", {"label": desc})
    req("POST", "/duck/reset")
    req("DELETE", "/obstacles")
    req("PUT", "/sim/goal", {"start": {"x": start[0], "y": start[1]}, "end": {"x": end[0], "y": end[1]}})
    for ox, oy, r in obstacles:
        req("POST", "/obstacles", {"x": ox, "y": oy, "r": r})
    req("POST", "/autopilot/start")
    resume()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        s = req("GET", "/state")
        if s.get("arrived"):
            print(f"  {desc}: {s['frame']*0.05:.1f}s OK")
            return True
        time.sleep(1)
    print(f"  {desc}: {timeout_s}s TIMEOUT (x={s['x_cm']:.0f} y={s['y_cm']:.0f})")
    return False

STANDARD = [
    ("no obstacles", []),
    ("single r=30", [(400,100,30)]),
    ("8 wall cluster", [(400,60,12),(400,140,12),(500,80,12),(500,120,12),(600,100,15),(650,70,10),(650,130,10),(400,100,20)]),
    ("dense 16 S->E", [(641,125,8),(641,92,9),(642,82,13),(644,111,10),(646,10,8),(646,52,10),(651,42,5),(651,101,14),(733,77,8),(733,186,11),(735,97,9),(737,169,9),(740,100,13),(740,172,12),(745,129,12),(749,152,15)]),
    ("user config", [(567,172,5),(577,122,5),(593,153,13),(618,155,10),(635,86,14),(619,117,8),(719,17,13),(719,44,7),(719,77,8),(719,112,6),(719,130,9)]),
    ("maze standard", [(250,60,10),(250,80,10),(250,100,10),(250,120,10),(250,140,10),(250,160,10),(250,180,10),(250,200,10),(450,0,10),(450,20,10),(450,40,10),(450,60,10),(450,80,10),(450,100,10),(450,120,10),(450,140,10),(650,60,10),(650,80,10),(650,100,10),(650,120,10),(650,140,10),(650,160,10),(650,180,10),(650,200,10)]),
    ("maze hard", [(250,50,10),(250,70,10),(250,90,10),(250,110,10),(250,130,10),(250,150,10),(250,170,10),(250,190,10),(450,10,10),(450,30,10),(450,50,10),(450,70,10),(450,90,10),(450,110,10),(450,130,10),(450,150,10),(650,50,10),(650,70,10),(650,90,10),(650,110,10),(650,130,10),(650,150,10),(650,170,10),(650,190,10)]),
]

GAP = [
    ("chicane", [(350,50,10),(350,60,10),(350,70,10),(350,80,10),(350,90,10),(350,100,10),(350,110,10),(350,120,10),(350,130,10),(350,140,10),(550,80,10),(550,90,10),(550,100,10),(550,110,10),(550,120,10),(550,130,10),(550,140,10),(550,150,10),(550,160,10),(550,170,10)]),
    ("off-gap mid", [(350,50,10),(350,60,10),(350,70,10),(350,80,10),(350,90,10),(350,110,10),(350,120,10),(350,130,10),(350,140,10),(350,150,10),(350,160,10),(350,170,10),(350,180,10),(350,190,10),(550,50,10),(550,60,10),(550,70,10),(550,80,10),(550,90,10),(550,110,10),(550,120,10),(550,130,10),(550,140,10),(550,150,10),(550,160,10),(550,170,10),(550,180,10),(550,190,10)]),
    ("alley 80cm", [(400,60,10),(400,70,10),(400,80,10),(400,90,10),(400,130,10),(400,140,10),(400,150,10),(400,160,10)]),
    ("barrier top gap", [(250,50,10),(250,70,10),(250,90,10),(250,110,10),(250,130,10),(250,150,10),(250,170,10),(250,190,10)]),
]

print(f"Open {HOST} in your browser to watch\n")
ok = 0; fail = 0
for desc, obs in STANDARD + GAP:
    try:
        if run(desc, obs):
            ok += 1
        else:
            fail += 1
    except Exception as e:
        print(f"  {desc}: ERROR {e}")
        fail += 1
    sys.stdout.flush()
print(f"\n{'='*60}\nDone: {ok} OK, {fail} FAIL")
