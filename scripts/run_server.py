"""Run scenarios directly on the server engine singleton.
Open http://localhost:8080 and watch each one complete."""

import sys, time, os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("OBJC_DISABLE_INITIALIZE_FORK_SAFETY", "YES")

import dashboard.server as srv

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
    ("off-gap mid (impassable)", [(350,50,10),(350,60,10),(350,70,10),(350,80,10),(350,90,10),(350,110,10),(350,120,10),(350,130,10),(350,140,10),(350,150,10),(350,160,10),(350,170,10),(350,180,10),(350,190,10),(550,50,10),(550,60,10),(550,70,10),(550,80,10),(550,90,10),(550,110,10),(550,120,10),(550,130,10),(550,140,10),(550,150,10),(550,160,10),(550,170,10),(550,180,10),(550,190,10)]),
    ("alley 80cm", [(400,60,10),(400,70,10),(400,80,10),(400,90,10),(400,130,10),(400,140,10),(400,150,10),(400,160,10)]),
    ("barrier top gap", [(250,50,10),(250,70,10),(250,90,10),(250,110,10),(250,130,10),(250,150,10),(250,170,10),(250,190,10)]),
]

def wait(timeout_s=300):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        st = srv._engine.get_state()
        if st["arrived"]:
            return st["frame"] * 0.05, True
    return timeout_s, False

def run(desc, obstacles, start=(30, 100), end=(960, 100), timeout_s=300):
    srv._continuous = False
    time.sleep(0.1)
    srv._engine.scenario_label = desc
    srv._engine.reset()
    srv._engine.clear_obstacles()
    srv._engine.set_goal(start={"x": start[0], "y": start[1]}, end={"x": end[0], "y": end[1]})
    for ox, oy, r in obstacles:
        srv._engine.add_obstacle(ox, oy, r)
    srv._engine.set_autopilot(True)
    srv._continuous = True
    elapsed, ok = wait(timeout_s)
    status = "OK" if ok else "TIMEOUT"
    print(f"  {desc:>20}: {elapsed:>5.1f}s {status}")
    sys.stdout.flush()
    return ok, elapsed

print(f"Open http://localhost:8080 in your browser to watch\n")
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
print(f"\nDone: {ok} OK, {fail} FAIL")
