# gplan — standalone global-routing path planner

A dependency-free C++ core that turns *(obstacles, start, target)* into a small
set of ranked **candidate paths** (which side of each obstacle to pass), as
polyline waypoints. It is the **global-routing** brain that sits on top of a
detailed router (KiCad PNS) — the half KiCad never had.

It does **not** route or shove. You feed the waypoints to PNS; PNS does the
exact geometry, clearance and push-and-shove. On failure you report back and the
planner re-ranks (negotiated-congestion / PathFinder style).

## What it does

1. **Inflated visibility graph** over the *fixed* obstacles only (offset each
   convex hull outward by `clearance + trackWidth/2`; nodes = inflated corners).
2. **Hard vs soft** baked into the graph: fixed obstacles delete edges; movable
   copper never blocks — it only raises edge **cost** (capacity / congestion).
   That is the "wiggle room": a full-looking gap stays usable because PNS can
   shove the movable tracks aside.
3. **k distinct paths** (A* + edge-reuse penalty) → different homotopies.
4. **PathFinder feedback**: `bump_congestion(point, r, f)` after a PNS failure
   makes the next plan avoid that area.

## Build & test

```bash
# C++ self-test (no Python needed):
g++ -std=c++17 -O2 planner_core.cpp selftest.cpp -o selftest && ./selftest

# Python module:
pip install pybind11
cmake -B build -DBUILD_PYTHON=ON . && cmake --build build
PYTHONPATH=build python3 example.py
```

## The two adapters YOU provide (KiCad-linked, not in this core)

| Adapter | Direction | What it does |
|---|---|---|
| obstacle source | board → core | After `SyncWorld`, walk the PNS world; emit each item as `Obstacle{ poly = Hull(), fixed = !item->IsMovable() }`. Fixed hulls must be convex. |
| route verifier   | core → board | Feed a candidate's waypoints to PNS (`StartRouting` → `Move` per waypoint → `Move(target)`), then `QueryColliding` on the placed trace. Return `(ok, blockingPoint)`. |

Orchestrate the `plan → verify → bump_congestion → replan` loop in Python
(see `example.py`).

## API

```cpp
Planner planner( obstacles, params );          // obstacles: vector<Obstacle>
std::vector<Path> paths = planner.plan(start, target);   // ascending cost
planner.bumpCongestion( where, radius, factor );         // after a PNS failure
```

## v1 limitations / roadmap

- **Single layer, no vias.** Start and target must be on one layer. v2: stack a
  graph per layer and add via-edges (with a via cost) between them — this is the
  only globally-aware place to plan layer changes.
- Fixed hulls assumed **convex** (PNS `Hull()` is). Non-convex fixed shapes would
  need decomposition.
- `corridorClear`/capacity use distance approximations, not swept-polygon
  clipping — fine because PNS is the ground truth; the planner only proposes.
- Congestion `capacity = gap / pitch` is an estimate; tune `wCongestion`,
  `wTightness`, `reusePenalty` per board.
