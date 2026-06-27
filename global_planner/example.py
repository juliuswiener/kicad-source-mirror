#!/usr/bin/env python3
"""
Example: drive the standalone planner core from Python, with the full
plan -> route -> feedback loop. The PNS part is mocked here so this runs
without KiCad; replace `route_and_check` with your real PNS bridge.

Build the module first:
    pip install pybind11
    cmake -B build -DBUILD_PYTHON=ON . && cmake --build build
    PYTHONPATH=build python3 example.py
"""

import math
import gplan


def box(cx, cy, w, h):
    hw, hh = w / 2, h / 2
    return [gplan.Point(cx - hw, cy - hh), gplan.Point(cx + hw, cy - hh),
            gplan.Point(cx + hw, cy + hh), gplan.Point(cx - hw, cy + hh)]


# ---- The adapter you implement on the KiCad side ---------------------------
# Returns (ok, blocking_point_or_None). Mock standing in for PNS: it samples
# ALONG each segment (as the real router would see the whole trace) and "fails"
# if the line passes through the central gap — pretend PNS couldn't shove enough.
def route_and_check(waypoints):
    for a, b in zip(waypoints, waypoints[1:]):
        for k in range(21):
            t = k / 20.0
            x = a.p.x + (b.p.x - a.p.x) * t
            y = a.p.y + (b.p.y - a.p.y) * t
            if abs(x - 5.0) < 1.0 and abs(y) < 0.8:
                return False, gplan.Point(5.0, 0.0)   # stuck in the gap
    return True, None
# ---------------------------------------------------------------------------


def main():
    obstacles = [
        gplan.Obstacle(box(5,  1.6, 3, 2), True),    # fixed component C
        gplan.Obstacle(box(5, -1.6, 3, 2), True),    # fixed component D
        gplan.Obstacle(box(5,  0.0, 1, 0.2), False), # movable track in the gap
    ]

    params = gplan.PlannerParams()
    params.trackWidth = 0.2
    params.clearance = 0.2
    params.kPaths = 5

    planner = gplan.Planner(obstacles, params)
    start, target = gplan.Point(0, 0), gplan.Point(10, 0)

    # plan -> try candidates -> on failure, bump congestion and replan
    for attempt in range(4):
        paths = planner.plan(start, target)
        if not paths:
            print("no path found"); return

        print(f"attempt {attempt}: {len(paths)} candidate(s), "
              f"best cost {paths[0].cost:.2f}")

        routed = False
        for p in paths:
            wps = [(round(w.p.x, 2), round(w.p.y, 2), w.layer) for w in p.waypoints]
            ok, blk = route_and_check(p.waypoints)
            print(f"   try {wps} -> {'OK' if ok else 'FAIL'}")
            if ok:
                print(f"   >>> routed via {wps}")
                routed = True
                break
            planner.bump_congestion(blk, 1.5, 5.0)   # learn from the failure

        if routed:
            return
    print("gave up after retries")


if __name__ == "__main__":
    main()
