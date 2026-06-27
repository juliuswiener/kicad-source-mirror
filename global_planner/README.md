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
# Core tests (ctest) + sanitizers — the standalone core is fully covered:
cmake -B build -DGPLAN_SANITIZE=ON .
cmake --build build && ( cd build && ctest --output-on-failure )

# Python module + end-to-end demo of the plan->verify->bump->replan loop:
pip install pybind11
cmake -B build -DBUILD_PYTHON=ON -Dpybind11_DIR=$(python3 -m pybind11 --cmakedir) .
cmake --build build
PYTHONPATH=build python3 example.py
```

`tests.cpp` covers: open field, start==target, two-homotopy enumeration with a
geometric margin-clearance assertion, movable-doesn't-block + congestion
re-ranking, routing from inside a pad, fully-enclosed/unreachable, non-convex
fixed input, multi-layer vias, and dedup. All pass clean under ASan/UBSan.

## The KiCad-side bridge (`pns_bridge.{h,cpp}`)

The two adapters are implemented in `pns_bridge.cpp` (KiCad-linked, **builds in
the KiCad tree only** — not part of the dependency-free core):

| Adapter | Direction | What it does |
|---|---|---|
| `getObstacles(layer)` | board → core | Walks the BOARD; emits each item as `Obstacle{ poly, fixed = pad/locked/keepout, layer }`. v1 uses bounding boxes for fixed shapes (convex, conservative). |
| `routeAndCheck(path)` | core → board | Loads board+rules, drives PNS shove along the waypoints (`StartRouting` → `Move` per waypoint, layer change → via), then `QueryColliding` on the placed trace. Returns `(ok, collided, blocking)`. |

It is modelled on the verified headless harness `qa/tools/pns/pns_log_player.cpp`
and the collision read-out in `pcbnew/router/pns_router.cpp::markViolations()`.
`load()` uses `BOARD_LOADER::Load`, which attaches `.kicad_pro` and runs
`InitEngine()` on `.kicad_dru` automatically (filename convention — same stem).

Orchestrate the `plan → verify → bump_congestion → replan` loop yourself
(see `example.py` for the loop with a mocked router).

## API

```cpp
Planner planner( obstacles, params );          // obstacles: vector<Obstacle>
std::vector<Path> paths = planner.plan(start, target);   // ascending cost
planner.bumpCongestion( where, radius, factor );         // after a PNS failure
```

## Status / limitations

- **Multi-layer + vias: done.** `params.layers` lists the copper stack; the
  planner adds via-edges (cost `viaCost`) between adjacent layers at clear
  landing sites. A waypoint layer change == "drop a via". `plan(start, sL,
  target, tL)`; a single-layer `plan(start, target)` convenience remains.
- Fixed hulls must be **convex** (bounding boxes in the bridge are; PNS `Hull()`
  is too). Non-convex fixed shapes would need decomposition.
- Via sites are restricted to graph columns (obstacle corners + start/target).
  Fine for "last bits"; a denser via-candidate set can be added if needed.
- Bridge via handling (`ToggleViaPlacement`/`SwitchLayer`) is a first cut —
  verify against your PNS version.
- Capacity/tightness use distance approximations, not swept-polygon clipping —
  fine because PNS is the ground truth; the planner only proposes. Tune
  `wCongestion`, `wTightness`, `reusePenalty`, `viaCost` per board.

## Performance & scaling

Graph build is ~O(n³) in the obstacle count n (n=20 → ~0.9 ms, n=100 → ~60 ms,
n=200 → ~370 ms per `plan()`, single core). For the intended use — *local*
"last-bits" routing — feed only obstacles in the route's neighbourhood (the
bridge's `getObstacles` can be bounded to a bbox around start/target); n then
stays in the tens and each plan is sub-10 ms. For a 100-target evaluation loop,
reuse one `Planner` and run targets across worker processes (the core is
stateless between `plan()` calls except for `bumpCongestion`).

## Bridge build status

The pure core + Python bindings are built, tested (ctest, ASan/UBSan) and run
end-to-end here. `pns_bridge.{h,cpp}` is **KiCad-linked and cannot be compiled
outside the KiCad tree**; every one of its ~60 PNS/BOARD API calls was verified
against the repository headers (signatures match). Build it inside the KiCad
tree to validate at runtime; the via-placement sequence
(`ToggleViaPlacement`/`SwitchLayer`) is a first cut to confirm there.
