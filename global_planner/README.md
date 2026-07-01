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
| `routeAndCommit(waypoints)` | **production path**: route AND commit to the PNS world (route-order). Returns PNS's **lossless parent-matched** change stream (`RouteChange`): `added*` (new copper), `mod*` (shoved neighbours — **modify the existing board item by UUID in place**, via links intact), `removed_uuids`. Only commits if the route actually **reaches** the target; otherwise `ok=false, reached=false` + `blocking` = farthest point reached. |
| `routeLongHaul(waypoints, maxInserts)` | long single legs that PNS can only fragment: retry `routeAndCommit`, inserting the `blocking` point as a **checkpoint** waypoint each round (stable sub-goals, fresh budgets). Honest `ok=false` if it still can't reach. |
| `dragComponent(x,y,newX,newY)` | shove a footprint + its connected tracks (PNS `COMPONENT_DRAGGER`). Commits only if the move is **clean** (never endangers connections). Refuses a **locked** footprint. Returns the same `RouteChange` (shoved tracks as `mod`-by-UUID). |
| `probeDrag(x,y,newX,newY)` | **speculative** component drag — evaluate `{clean, cost, shoved}` then discard (commits nothing). The primitive for a placement search. |
| `shoveComponentSearch(x,y,candidates)` | router-driven placement: `probeDrag` each candidate `{nx,ny}`, keep the cleanest/cheapest, commit it. `ShoveResult{ committed, x, y, cost, change }`. |
| `routeDiffPairAndCommit(waypoints)` | route a differential pair: click **one** point of either net — `DIFF_PAIR_PLACER::FindDpPrimitivePair` auto-finds the coupled net (matches `+/-` / `_P/_N` naming). Both legs routed together, gap from netclass rules (auto-import, same as `routeAndCommit`). Same `RouteChange` contract; router placer-mode is switched and **restored** on every exit path. |
| `tuneLength(x,y,endX,endY,layer,targetLengthNm)` | length-tune an **existing** routed run (`MEANDER_PLACER`) between two points on the same trace. `TuneResult{ change, status (0=TOO_SHORT/1=TOO_LONG/2=TUNED), currentLength, targetLength }`. Commits only on `status==TUNED`. |
| `probeViaMove(x,y,newX,newY)` / `moveVia(x,y,newX,newY)` | relocate a single **via** (PNS `DM_VIA` drag) — e.g. a GND-stitch via blocking a pin's escape corridor. Same speculative/commit split as `probeDrag`/`dragComponent`; refuses a **locked** via. |
| `shoveViaSearch(x,y,candidates)` | router-driven via placement: `probeViaMove` each candidate, commit the cleanest/cheapest. Same `ShoveResult` as `shoveComponentSearch`. |

Modelled on `qa/tools/pns/pns_log_player.cpp` and `pns_router.cpp` (markViolations,
CommitRouting). `routeAndCommit`/`dragComponent` override the iface's `AddItem/
UpdateItem/RemoveItem` to capture PNS's own commit stream — so shoved neighbours
are modified in place rather than delete+re-added (which would break connectivity).

**Locking (host-controlled "do not move"):** lock copper with `track.SetLocked(true)`
before `attach()` → PNS marks it `MK_LOCKED` and never shoves it while routing;
lock a footprint with `fp.SetLocked(true)` → `dragComponent`/`probeDrag` refuse it
(`ok=false, reason="component locked"`); lock a via with `via.SetLocked(true)` →
`moveVia`/`probeViaMove` refuse it (`reason="via locked"`). NPTH pads are auto
non-routable.

**⚠ One KiCad runtime per process.** `gplan_kicad` links its own `wxWidgets`/
`PGM_BASE`/kiface singletons (from the tree it was built against). Loading
`gplan_kicad` in the **same process** as another KiCad/pcbnew instance (e.g. a
KiCad plugin host, or a second `gplan_kicad` built against a different KiCad
version) collides on those singletons → SIGABRT/segfault. If your agent needs
both gplan routing *and* a live pcbnew session, run them as **separate
processes** and hand geometry across via a file/socket (route in one process →
serialize the `RouteChange`/`RouteGeom` to JSON → apply in the other). Locking
one `Planner`/`PnsBridge` instance to one thread is fine; the process-level
singleton is the actual boundary.

**Locking everything to force a detour is a planner-scale trap, not a fix.**
Locking ALL copper (not just the footprint/via you actually need fixed) turns
every trace into a `fixed` obstacle for the core — `getAllObstacles()` returns
hundreds of hulls, and `plan()`'s O(n³) graph build (§ Performance & scaling)
chokes/times out, *and* the "wiggle room" (movable copper only costs, never
blocks — see §1 in `AGENT_GUIDE.md`) is gone, so there's no slack left to route
through. Lock only the specific footprint/via you want held still; leave
everything else movable so PNS can shove it.

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

### Long-haul net + opening a corridor by shoving a component

```python
# Cross-board net PNS can't reach in one leg -> checkpoint-retry driver:
r = br.route_long_haul(planner.plan(start, target)[0].waypoints, max_inserts=6)
if not r.ok:                                   # honest: still couldn't reach
    # r.blocking = farthest point; nudge a blocking component out of the way:
    cands = [(bx + dx, by) for dx in range(-200_000, 200_001, 50_000)]   # nm grid
    sr = br.shove_component_search(cx, cy, cands)   # probe each, commit cheapest-clean
    if sr.committed:
        board.move_footprint(ref, sr.x - cx, sr.y - cy)
        for u, g in zip(sr.change.mod_seg_uuids, sr.change.mod_segs): board.modify(u, g)
        r = br.route_long_haul(...)            # retry the net in the freed corridor
```

### Relocating a via blocking a pin's escape corridor

```python
# GND-stitch via sitting in Pin 21 (RST_N)'s escape route:
cands = [(vx + dx, vy + dy) for dx in (-100_000, 0, 100_000)
                             for dy in (-100_000, 0, 100_000) if dx or dy]  # nm grid
sr = br.shove_via_search(vx, vy, cands)   # probe each, commit cheapest-clean
if sr.committed:
    board.move_via(via_uuid, sr.x - vx, sr.y - vy)
    for u, g in zip(sr.change.mod_seg_uuids, sr.change.mod_segs): board.modify(u, g)
# or, for one specific candidate: pv = br.probe_via_move(vx, vy, nx, ny)
#                                 if pv.clean: br.move_via(vx, vy, nx, ny)
```

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
loop** (`pathfinder.h`).

Also done, verified end-to-end against real KiCad/PNS: **differential pair
routing** (`routeDiffPairAndCommit`), **length tuning** (`tuneLength`, via
`MEANDER_PLACER` — ground-truthed against `pcb_tuning_pattern.cpp`), and
**single-via relocation** (`moveVia`/`probeViaMove`/`shoveViaSearch`, for cases
like a stitching via blocking a pin's escape route). See `ROADMAP.md` for the
remaining backlog (skew tuning, optimizer pass, CDT free-space backend, ML
net-order ranker).

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
routeAndCommit: ok=1 placed=1 reached=1 net=1 added=9     <-- lossless, honest-reached
probeTarget net-1 pad: seedable=1 congested=0            <-- target probe
dragComponent (+0.05mm): ok=1 placed=1                   <-- clean component shove
dragComponent on LOCKED footprint: ok=0 'component locked' <-- lock respected
probeDrag(zero move): clean=1 cost=0                     <-- speculative, no commit
shoveComponentSearch: committed=1 chosen=(95915000,56896000) <-- candidate search
routeLongHaul: ok=1 reached=1                            <-- checkpoint-retry driver
routeDiffPairAndCommit: ok=0 (no diff-pair-named nets on this board — honest);
  mode correctly RESTORED after (confirmed by a routeAndCheck right after)
tuneLength: ok=0 status=TOO_LONG curLen=46075907 target=10295663  <-- ran clean, honest non-commit
T9 ratsnest target: (123825000,68326000); routed ok=1 placed=1  <-- unrouted-net endpoint
SMOKETEST OK   (EXIT=0)
```

**Via relocation, verified on `qa/data/pcbnew/issue7325.kicad_pcb`** (327 real
vias — `complex_hierarchy` has none, so this needs a via-bearing fixture):

```
T-VIA: via at (208788000,108458000)
probeViaMove(+0.2mm): clean=1 cost=0 shoved=0
moveVia(+0.2mm): ok=1 placed=1 modVias=1                 <-- a REAL via relocated, committed
moveVia on LOCKED via: ok=0 reason='via locked'          <-- lock respected
shoveViaSearch: committed=1 chosen=(209088000,108458000) <-- candidate search, committed
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
- `tuneLength`: `MEANDER_PLACER::doMove()` unconditionally computes
  `origPathDelay()` even in length-only mode, dereferencing
  `MEANDER_SETTINGS::m_netClass` — crashed until populated from the real net's
  `NETCLASS` before `UpdateSettings()`;
- `tuneLength`: a manual `QueryColliding()` on the meander geometry after
  `Move()` corrupted the R-tree traversal (deep crash in
  `KIRTREE::COW_RTREE::searchImpl`) — removed; the real UI driver
  (`pcb_tuning_pattern.cpp`) never runs this check either, since meanders
  self-enforce DRC via `TuningStatus()`;
- `ROUTER::FailureReason()` is **sticky** across `StartRouting()` calls (not
  reset) — `tuneLength`/`routeDiffPairAndCommit` only surface it when nothing
  was actually placed, else it leaks the previous call's error message.
