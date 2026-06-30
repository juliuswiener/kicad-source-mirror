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

## Bridge smoketest (real link + runtime against KiCad/PNS)

`bridge_smoketest.cpp` loads a board via the qa helper, `attach()`es the bridge,
extracts obstacles from the live PNS world, runs the core, and exercises
`routeAndCheck`. To build it, add this target to `qa/tools/pns/CMakeLists.txt`
(mirrors `qa_pns_regressions`) and configure with `-DKICAD_BUILD_PNS_DEBUG_TOOL=ON`:

```cmake
add_executable( gplan_bridge_smoketest
  ${COMMON_SRCS}
  ../../qa_utils/pcb_test_frame.cpp ../../qa_utils/pcb_test_selection_tool.cpp
  ../../qa_utils/test_app_main.cpp ../../qa_utils/utility_program.cpp
  ../../qa_utils/mocks.cpp
  ../../../global_planner/planner_core.cpp
  ../../../global_planner/pns_bridge.cpp
  ../../../global_planner/bridge_smoketest.cpp )
target_compile_definitions( gplan_bridge_smoketest PRIVATE PCBNEW TEST_APP_NO_MAIN )
add_dependencies( gplan_bridge_smoketest pcbnew )
target_include_directories( gplan_bridge_smoketest PRIVATE ${CMAKE_SOURCE_DIR}/global_planner )
target_link_libraries( gplan_bridge_smoketest
  qa_pcbnew_utils connectivity pcbcommon pnsrouter gal common gal qa_utils
  dxflib_qcad tinyspline_lib nanosvg idf3 pcbcommon markdown_lib 3d-viewer
  ${PCBNEW_IO_LIBRARIES} ${wxWidgets_LIBRARIES} ${GDI_PLUS_LIBRARIES}
  Boost::headers ${PCBNEW_EXTRA_LIBS} )
```

```bash
ninja -C <kicad-build> gplan_bridge_smoketest
<kicad-build>/qa/tools/pns/gplan_bridge_smoketest complex_hierarchy
```

The bridge splits `load()` (uses `BOARD_LOADER`, i.e. the pcbnew kiface) from
`attach(BOARD*)` (routing setup only) so a host with its own board-loading path
links without the kiface — the smoketest uses `attach()`.

## Bridge build status — verified link + runtime

The pure core + Python bindings are built, tested (ctest, ASan/UBSan) and run
end-to-end. `pns_bridge.{h,cpp}` is **KiCad-linked** (builds in the KiCad tree).
It has been **compiled, linked and run against a real KiCad/PNS build** via
`bridge_smoketest` on `qa/data/pcbnew/complex_hierarchy.kicad_pcb`:

```
board loaded: 361 tracks, 72 footprints
bridge attached, PNS world synced
obstacles extracted: 739 (fixed=378, movable=361)
gplan candidates for the net: 3
routeAndCheck: ok=1 placed=1 vias=0 collided=0           <-- real PNS shove route, clean
multilayer routeAndCheck: ok=1 placed=1 vias=1 collided=0 <-- F.Cu->via->B.Cu, clean
SMOKETEST OK
```

The multi-layer leg drops a real via (`vias=1`): the via commits on a mid-route
`FixRoute` (kept speculative with `forceCommit=false`), after which the head
continues on the new layer.

Bugs found and fixed during runtime bring-up (all in the bridge, none in the core):
- split `load()` (needs `BOARD_LOADER`/kiface) from `attach(BOARD*)` so hosts
  with their own board loading link without the kiface;
- `attach()` must `LoadSettings()` a `ROUTING_SETTINGS` before `SyncWorld()`/
  `SetMode()` (else `Settings()` derefs null);
- evaluate the routed **head** via `Traces()` (the head exists after `Move()`),
  not `HasPlacedAnything()` (only true after a fix/commit);
- item pick falls back to a small slop radius;
- via sequence rewritten: a layer change does `Move` → arm via → `SwitchLayer`
  → `FixRoute` (speculative), which actually places the via — verified `vias=1`.
