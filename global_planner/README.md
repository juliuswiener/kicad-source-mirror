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

## The KiCad-side bridge (`pns_bridge.{h,cpp}`, Python module `gplan_kicad`)

KiCad-linked, **builds in the KiCad tree only** (not part of the dependency-free
core). `load()`/`attach()` set up the PNS world; everything else reads the world
or drives PNS. `load()` uses the kiface-free `PCB_IO_KICAD_SEXPR` + a hand-built
`DRC_ENGINE` (picks up `.kicad_pro` / `.kicad_dru` by filename stem); `attach()`
is the same minus loading, for hosts with their own BOARD.

| Method | What it does |
|---|---|
| `load(path)` / `attach(board)` | build PNS world; `attach()` is **re-callable** (T10) — `cleanup()` then re-attach to route another board in one process |
| `setMode(mode)` | `RouteMode` Shove / Walkaround / MarkObstacles (re-asserted before each `StartRouting`) |
| `getObstacles(layer)` / `getAllObstacles()` | BOARD → `gplan::Obstacle`. Pads use their **real effective-shape hull** (not bbox); board outline added as a thin fixed boundary |
| `nearestUnconnected(x,y,layer)` | from a start point, the nearest **unrouted** ratsnest anchor — feed gplan real endpoints, not guessed pad centres |
| `probeTarget(x,y,layer,net,clr,otherLayer)` | `TargetProbe{ seedable, congested, nearestForeign, foreignNet, nearestOther }` — flag a congested pad-centre target and re-aim at the escape/drop point |
| `routeAndCheck(waypoints)` | drive PNS shove along the waypoints; speculative (commits nothing). `RouteResult{ ok, placed, vias, collided, blocking, reason }` |
| `routeAndExtract(waypoints)` | speculative, but returns the routed geometry (`RouteGeom`) for the host to apply itself |
| `routeAndCommit(waypoints)` | **production path**: route AND commit to the PNS world (route-order). Returns PNS's **lossless parent-matched** change stream (`RouteChange`): `added*` (new copper), `mod*` (shoved neighbours — **modify the existing board item by UUID in place**, via links intact), `removed_uuids`. Next net shoves against this copper. |

Modelled on `qa/tools/pns/pns_log_player.cpp` and `pns_router.cpp` (markViolations,
CommitRouting). `routeAndCommit` overrides the iface's `AddItem/UpdateItem/
RemoveItem` to capture PNS's own commit stream — so shoved neighbours are
modified in place rather than delete+re-added (which would break connectivity).

### Production host loop (commit-to-world)

```python
import gplan, gplan_kicad
br = gplan_kicad.PnsBridge(); br.load("board.kicad_pcb")
br.set_mode(gplan_kicad.PnsBridge.RouteMode.WALKAROUND)
for net in route_order:                          # order matters less w/ commit-to-world
    t = br.probe_target(tx, ty, fcu, net, clearance, bcu)
    if t.congested or not t.seedable:
        tx, ty, layer = escape_point(net)        # B.Cu drop, not the congested pad centre
    paths = planner.plan(start, fcu, gplan.Point(tx, ty), layer)
    c = br.route_and_commit(paths[0].waypoints)  # committed into the world
    for s in c.added_segs:                         board.add_track(s)
    for u, g in zip(c.mod_seg_uuids, c.mod_segs):  board.modify(u, g)   # in-place, keeps vias
    for u in c.removed_uuids:                       board.delete(u)
```

The speculative `plan → routeAndCheck → bump_congestion → replan` loop
(`example.py`, mocked router) is still available for candidate evaluation.

## API

```cpp
Planner planner( obstacles, params );          // obstacles: vector<Obstacle>
std::vector<Path> paths = planner.plan(start, target);   // ascending cost
planner.bumpCongestion( where, radius, factor );         // after a PNS failure
```

## Status / limitations

Done (roadmap T1–T12, each gate-verified): thread-safe `Planner`, **Yen's
k-distinct paths**, exact convex edge-blocking, AABB spatial cull (~10× the graph
build), multi-layer + vias, real-`Hull()` obstacle extraction, board outline,
ratsnest endpoints, reloadable bridge, route modes, **lossless commit-to-world**,
target congestion probe, and a board-level **negotiated-congestion multi-net
loop** (`pathfinder.h`). See `ROADMAP.md` for the remaining backlog (diff pairs,
length/skew tuning, optimizer pass, CDT free-space backend, ML net-order ranker).

Caveats:
- Fixed hulls must be **convex** — the core convex-hulls them defensively; a
  genuinely concave keepout should be split.
- Via sites are graph columns (obstacle corners + start/target). Fine for
  "last bits"; a denser via grid can be added.
- Capacity/tightness use distance approximations, not swept-polygon clipping —
  fine because PNS is the ground truth; the planner only proposes. Tune
  `wCongestion`, `wTightness`, `viaCost` per board.
- `routeAndCommit`'s shove-capture (`mod*` by UUID) and `probeTarget`'s
  `congested` path both run and compile against real KiCad, but the regression
  fixture (`complex_hierarchy`) didn't trigger a shove or a fine-pitch neighbour,
  so those two branches are mechanism-verified rather than gated. A fine-pitch +
  shove fixture would close that.

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
obstacles extracted: 747 (fixed=386, movable=361)   <-- real hulls + board outline
routeAndCheck: ok=1 placed=1 vias=0 collided=0           <-- real PNS shove route, clean
multilayer routeAndCheck: ok=1 placed=1 vias=1 collided=0 <-- F.Cu->via->B.Cu, clean
routeAndExtract: ok=1 placed=1 net=1 segs=9               <-- host applies geometry
T12 walkaround routeAndCheck: ok=1 placed=1               <-- route mode switch
T10 re-attach routeAndCheck: ok=1 placed=1               <-- reloadable bridge
routeAndCommit: ok=1 placed=1 net=1 added=9 mod=0 removed=0 <-- lossless commit-to-world
probeTarget net-1 pad: seedable=1 congested=0            <-- target probe
T9 ratsnest target: (123825000,68326000); routed ok=1 placed=1  <-- unrouted-net endpoint
SMOKETEST OK   (EXIT=0)
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
