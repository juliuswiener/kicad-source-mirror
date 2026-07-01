# gplan — Integration Guide for a PCB-Builder Agent

This document tells an autonomous PCB-builder agent how to drive **gplan** (the
standalone global-routing planner) together with the **PNS bridge** (KiCad's
push-and-shove detailed router) to route nets on a board.

Read this end-to-end before wiring gplan into your agent loop. It covers the
mental model, the full API, units, the canonical routing loop, the multi-target
evaluation pattern, tuning, failure interpretation, and the known limits.

---

## 1. Mental model — two routers, one job

PCB routing is solved in two stages. KiCad ships only the second one.

| Stage | What it decides | Component |
|---|---|---|
| **Global routing** | *Which corridor / which side of each obstacle / which layer* a net takes — the **homotopy**. Coarse. | **gplan** (this project) |
| **Detailed routing** | Exact geometry, clearances, and pushing existing copper aside — the **shove**. | **KiCad PNS**, via the bridge |

gplan does **not** route copper. It proposes a small set of ranked candidate
**waypoint paths**. The bridge feeds each candidate to PNS, which lays the actual
track (shoving neighbours as needed) and reports whether it worked. On failure
the agent tells gplan *where* it failed; gplan re-ranks and proposes again.

> The waypoints only need to be **topologically right** (correct side of each
> obstacle, correct layer transitions). PNS fixes exact geometry and clearance.
> This is why a coarse plan is enough — and why you must not treat gplan output
> as DRC-correct on its own. **PNS is the ground truth.**

This split is the standard global→detailed autorouter architecture (what
Freerouting fuses into one engine); gplan supplies the global-planning layer
KiCad never had.

---

## 2. Components & files

```
global_planner/
  planner_core.{h,cpp}   PURE C++ core (no KiCad dep). The planner.   [tested]
  bindings.cpp           pybind11 -> Python module `gplan`.           [tested]
  pns_bridge.{h,cpp}     KiCad-linked: BOARD <-> PNS. attach/extract/route.
  pns_bridge_load.cpp    Optional load() helper (pulls BOARD_LOADER/kiface).
  bridge_bindings.cpp    pybind11 -> Python module `gplan_kicad`.
  bridge_smoketest.cpp   Real link+runtime test (qa harness).         [passing]
  tests.cpp / selftest.cpp  Core unit tests (ctest, ASan/UBSan).      [passing]
```

Two layers, deliberately separated:

- **Core** (`gplan`): pure geometry. Knows only points, polygons, layers. Builds
  anywhere, binds to Python trivially. This is where planning happens.
- **Bridge** (`gplan_kicad`): links KiCad/PNS. Turns a `BOARD` into obstacles and
  runs PNS. Only this half needs KiCad.

Your agent orchestrates the loop across the two. Plan in the core, verify in the
bridge.

---

## 3. Units & coordinate conventions

- **Geometry**: integers in **KiCad internal units = nanometres (nm)**.
  `0.2 mm = 200000`. The core uses `double` but you feed it nm from the board so
  everything stays consistent. (The core is unit-agnostic; just be consistent.)
- **Layers**: gplan uses **PNS layer indices** (0 = F.Cu, 1 = next inner, … ).
  Convert from KiCad `PCB_LAYER_ID` with `bridge.pnsLayer(F_Cu)` etc. Always tag
  obstacles and waypoints with PNS layer ids, and list your stack in
  `PlannerParams.layers` in physical order.
- **A waypoint's layer change == a via.** Two consecutive waypoints at the same
  (x,y) on different layers mean "drop a via here".

---

## 4. API reference — core (`gplan`)

### Data types

```cpp
struct Point   { double x, y; };
using  Polygon = std::vector<Point>;          // closed; first/last not repeated

struct Obstacle {
    Polygon poly;
    bool    fixed = true;   // true  = pad / locked track / keepout / board edge (HARD)
                            // false = movable copper (existing routed track/via) (SOFT)
    int     layer = 0;      // PNS copper layer. Multi-layer items: add once PER layer.
};

struct Waypoint { Point p; int layer = 0; };
struct Path     { std::vector<Waypoint> waypoints; double cost; };  // cost is planner-internal
```

> **Convexity:** `fixed` polygons must be **convex** (the core convex-hulls them
> defensively, so a slightly-off hull is fine; a genuinely concave keepout should
> be split). `movable` polygons may be any shape — they only add cost.

### Parameters — every field

```cpp
struct PlannerParams {
    double trackWidth   = 0.15;   // net being routed (nm on a real board)
    double clearance    = 0.15;   // required clearance to neighbours
    int    kPaths       = 5;      // how many distinct candidates to return

    double wCongestion  = 1.0;    // weight: "this channel is filling up"
    double wTightness   = 1.0;    // weight: "this gap is narrow"
    double reusePenalty = 3.0;    // edge-cost multiplier forcing later paths to differ

    std::vector<int> layers = {0};// copper stack in PHYSICAL order; vias connect adjacent entries
    double viaCost      = 5.0;    // cost per layer transition (raise to discourage vias)
    double viaClearance = 0.15;   // clearance around a via landing site
    double viaDiameter  = 0.45;   // via pad diameter (landing-site clear test)
};
```

The geometry margin used for blocking and node placement is
`margin = clearance + trackWidth/2`. Channel capacity is estimated as
`gap / (trackWidth + clearance)`.

### Planner

```cpp
Planner( std::vector<Obstacle> obstacles, PlannerParams params );

// Multi-layer: start/target each on a PNS layer. Returns candidates, ascending cost.
std::vector<Path> plan( Point start, int startLayer, Point target, int targetLayer );
// Single-layer convenience (both on params.layers[0]):
std::vector<Path> plan( Point start, Point target );

// PathFinder feedback after a router failure near `where` (any layer).
void bumpCongestion( Point where, double radius, double factor );
void clearCongestion();
```

`plan()` returns up to `kPaths` **distinct** candidate paths (different
homotopies), cheapest first. Empty if start/target are unreachable through the
**fixed** obstacles (movable copper never blocks — it only raises cost).

### Python (`gplan`)

```python
import gplan
p  = gplan.PlannerParams(); p.trackWidth=200000; p.clearance=200000; p.kPaths=5
p.layers=[0,1]; p.viaCost=200000
obs = [gplan.Obstacle(poly, fixed, layer), ...]   # poly = [gplan.Point(x,y), ...]
pl  = gplan.Planner(obs, p)
paths = pl.plan(gplan.Point(x0,y0), 0, gplan.Point(x1,y1), 0)   # or pl.plan(a, b)
for path in paths:
    for w in path.waypoints:   # w.p.x, w.p.y, w.layer
        ...
pl.bump_congestion(gplan.Point(x,y), radius, factor)
pl.clear_congestion()

# Waypoint has a real constructor — build/concatenate waypoint lists by hand
# (e.g. splicing legs from two different planner.plan() calls, or a manual
# escape leg) without going through Path:
manual = [gplan.Waypoint(gplan.Point(0,0), 0), gplan.Waypoint(gplan.Point(5,5), 1)]
```

---

## 5. API reference — bridge (`gplan_kicad`)

```cpp
struct RouteResult {
    bool        ok;        // placed AND reached the target AND no residual collisions
    bool        collided;  // placed but obstacles remain
    bool        placed;    // PNS produced a head trace at all
    int         vias;      // vias the route placed (layer changes realized)
    gplan::Point blocking; // where it got stuck (feed to bumpCongestion)
    std::string reason;    // router->FailureReason()
};

class PnsBridge {
    bool load( const std::string& pcbPath );  // C++/Python: load pcb+pro+dru, build DRC, sync
    bool attach( BOARD* board );              // C++ only: routing setup on an existing BOARD
    int  pnsLayer( int boardLayer ) const;    // PCB_LAYER_ID -> PNS layer index
    std::vector<gplan::Obstacle> getObstacles( int pnsLayer ) const;
    std::vector<gplan::Obstacle> getAllObstacles() const;   // all copper layers, tagged
    RouteResult routeAndCheck( const std::vector<gplan::Waypoint>& waypoints );
};
```

- **`load(path)`** is the standalone entry. It loads `<board>.kicad_pcb` and, by
  filename convention, `<board>.kicad_pro` (netclasses, design rules) and
  `<board>.kicad_dru` (custom rules) — **all three must share the stem and dir**.
  It builds the DRC engine and syncs the PNS world. Links `BOARD_LOADER` (the
  pcbnew kiface).
- **`attach(BOARD*)`** (C++ only) is for hosts that already have a loaded `BOARD`
  with its DRC engine initialized (e.g. inside a KiCad plugin/kiface). Avoids the
  kiface dependency. `load()` is just `BOARD_LOADER::Load` + `attach`.
- **`getObstacles` / `getAllObstacles`** walk the live PNS world: pads / locked
  tracks / keepouts → `fixed`; unlocked copper → `movable`. v1 emits **bounding
  boxes** for fixed shapes (convex, slightly conservative).
- **`routeAndCheck(waypoints)`** drives PNS shove along the waypoints (layer
  change → via), then reports the verdict. **Speculative** — it commits nothing
  to the board (you re-run it freely per candidate).

### Python (`gplan_kicad`)

```python
import gplan, gplan_kicad
br = gplan_kicad.PnsBridge()
br.load("path/to/board.kicad_pcb")          # .kicad_pro/.kicad_dru auto-picked by stem
obs = br.get_all_obstacles()                # -> [gplan.Obstacle]
fcu = br.pns_layer(0)                       # pass the board PCB_LAYER_ID int for F_Cu
r = br.route_and_check(path.waypoints)      # -> RouteResult: r.ok, r.placed, r.vias, r.blocking, r.reason
```

---

## 6. The canonical agent loop (plan → verify → bump → replan)

This is the heart of the integration. For one connection (start, target):

```python
import gplan, gplan_kicad

def route_connection(br, obs, params, start, s_layer, target, t_layer,
                     max_rounds=4):
    planner = gplan.Planner(obs, params)
    for _ in range(max_rounds):
        cands = planner.plan(start, s_layer, target, t_layer)
        if not cands:
            return None                       # unreachable through fixed copper
        for path in cands:                    # cheapest first
            r = br.route_and_check(path.waypoints)
            if r.ok:
                return path                   # PNS routed it cleanly
            # learn from the failure and try the next candidate / replan
            planner.bump_congestion(r.blocking, params.clearance * 8, 5.0)
    return None                               # gave up
```

Key points:
- `route_and_check` is **speculative** — none of the attempts modify the board.
  Only when you decide a path is final do you re-route it for real and commit
  (see §11).
- `bump_congestion` makes gplan avoid the exact spot PNS got stuck, so the next
  `plan()` proposes a genuinely different route — this is negotiated-congestion
  (PathFinder) and is what stops the loop from re-proposing dead ends.
- `r.blocking` is the obstacle/intersection point PNS reported.

---

## 7. Routing a real net end-to-end

```python
br = gplan_kicad.PnsBridge(); br.load(board_path)

params = gplan.PlannerParams()
params.trackWidth = 200000      # 0.2 mm
params.clearance  = 200000
params.layers     = [br.pns_layer(F_Cu), br.pns_layer(B_Cu)]   # board layer ids in
params.viaCost    = 1000000     # discourage vias unless needed
params.kPaths     = 5

fcu = br.pns_layer(F_Cu)
obs = [o for o in br.get_all_obstacles() if o.layer in params.layers]

# Pick endpoints: pad centres of the net's two unconnected ends (use the ratsnest
# / your netlist to find what still needs connecting).
start  = gplan.Point(*pad_a_xy_nm)
target = gplan.Point(*pad_b_xy_nm)

path = route_connection(br, obs, params, start, fcu, target, fcu)
```

Endpoint selection matters: feed the **unconnected** ends (from the ratsnest /
airwires). Routing between already-connected pads gives PNS nothing to place
(`placed=0`). The pad centre reliably yields a PNS start item (the bridge falls
back to a small slop radius if the exact point misses).

---

## 8. The 100-candidate / multi-target evaluation pattern

Your agent often must choose among many possible targets (e.g. which pin to
connect next). Evaluate them speculatively and rank by what actually routes:

```python
def evaluate_targets(br, obs, params, start, s_layer, targets):
    results = []
    for tgt in targets:                       # tgt = (Point, layer)
        planner = gplan.Planner(obs, params)  # fresh; or reuse + clear_congestion()
        cands = planner.plan(start, s_layer, tgt[0], tgt[1])
        best = None
        for path in cands:
            r = br.route_and_check(path.waypoints)
            if r.ok:
                best = (path, r); break
        results.append((tgt, best))
    return results                            # then pick the cheapest routable target
```

Optimisations:
- **Pre-screen cheaply**: a target whose straight corridor is wide-open routes
  trivially; one buried in copper can be deprioritised before the full attempt.
- **Parallelise across processes**: one `BOARD`+`PnsBridge` per worker (the
  bridge is single-threaded and stateful — do not share one across threads).
- The core is stateless between `plan()` calls except `bumpCongestion` state;
  `clear_congestion()` to reset.

---

## 9. Waypoint strategy — why the path you feed matters

PNS is a **greedy, local** engine with **per-Move budgets** (shove: 250
iterations / 1000 ms; walkaround: 40 iterations). A single jump straight to a far
target is the router's *weakest* mode: no directional hint, one budget, one giant
head, no checkpoints. Feeding waypoints fixes all of that:

- each leg gets its own iteration/time budget;
- good sub-decisions lock into the stable "tail";
- the cursor trail gives PNS a winding/posture hint (which side to go around).

gplan's candidate `waypoints` already provide this — the bridge walks them leg by
leg. **Do not collapse a candidate to just (start, target)**; pass the full
waypoint list. For trivial open routes a 2-point path is fine and faster.

---

## 10. Multi-layer & vias

- List the stack in `params.layers` in physical order. gplan adds via-edges
  (cost `viaCost`) between **adjacent** layers at landing sites clear of fixed
  copper on both layers.
- A candidate that changes layer has waypoints like
  `[(p, L0), (mid, L0), (mid, L1), (q, L1)]` — the `(mid,L0)→(mid,L1)` step at the
  same (x,y) is the via.
- The bridge realises a layer change as: `Move` to the via site → arm via →
  `SwitchLayer` → speculative `FixRoute` (commits the segment+via in the session,
  not the board) → continue on the new layer. `RouteResult.vias` counts what was
  placed (verified `vias=1` on a real F.Cu→B.Cu route).
- Raise `viaCost` to keep routes on one layer; lower it where layer changes are
  cheap/expected.
- **Via crossing a plane zone (inner-layer via, GND/power pour): expect a
  rejection with `vias=0`/`added=0` the first time.** The plane has no antipad
  for a via that doesn't exist yet — KiCad only carves clearance holes at zone-
  fill time, and a freshly loaded board's fill predates your via. `gplan_zone_refill`
  (`qa/tools/pns/gplan_zone_refill board.kicad_pcb --add-via x y drill dia net
  top bottom`, units = nm) inserts the via and runs the real zone filler in one
  pass, then re-`load()` the board — the via now collision-checks clean. See
  README § "Placing an inner-layer via that crosses a plane zone".

---

## 11. Committing a final route

`routeAndCheck` is speculative and commits nothing. When the agent has chosen the
final path for a net, route it for real through the standard PNS commit path
(`StartRouting` → `Move`/`FixRoute` with `forceCommit=true` → `CommitRouting`),
or extend the bridge with a `commit=true` variant. Keep speculation and commit
separate so evaluation never mutates the board.

---

## 12. Interpreting `RouteResult`

| Fields | Meaning | Agent action |
|---|---|---|
| `placed=0` | PNS produced no trace (bad/again-connected start, unroutable) | check `reason`; pick different endpoint; this candidate is dead |
| `placed=1, collided=1` | routed but residual DRC violations (couldn't shove enough) | `bump_congestion(blocking, …)`, try next candidate |
| `placed=1, ok=0, collided=0` | placed but didn't reach target (stopped short) | `bump_congestion`, try next |
| `ok=1` | placed, reached, clean | accept; optionally compare cost/vias across candidates |
| `vias=N` | layer changes realised | use to prefer fewer-via routes |

`reason` is `ROUTER::FailureReason()` — surface it in logs.

---

## 13. Tuning

| Goal | Knob |
|---|---|
| More/fewer candidates | `kPaths` |
| Avoid tight channels | raise `wTightness` |
| Spread load off busy channels | raise `wCongestion` |
| More distinct alternatives | raise `reusePenalty` |
| Fewer vias / stay on layer | raise `viaCost` |
| Wider keep-away | raise `clearance` (also affects PNS via the DRC rules) |

Defaults are sane for dense "last-bits" routing. Tune per board; the planner only
proposes, so over-tuning is low-risk.

---

## 14. Performance & scaling

Two structural fixes now in place: **T-GRID** (per-layer spatial hash, makes
the per-edge obstacle scan ~O(k) instead of ~O(m)) and **T-NEIGHBOR**
(`buildEdges()` no longer enumerates all-pairs — each node queries an
expanding ring of ≥24 spatial neighbours instead; start/target stay exempt —
always tested against every other node, so a free direct sightline is never
missed). Combined: n=200 obstacles ~10 ms (was ~370 ms brute force, **~37×**);
n=885 (a real large-session count) ~86 ms (was untested/would time out).
Scaling is now close to **linear**, not cubic. For local last-bits routing
(tens of obstacles) this was already a non-issue either way.

**T-NEIGHBOR trade-off**: unlike T-GRID (provably bit-identical — it only
narrows *which hulls get tested*), T-NEIGHBOR narrows *which node pairs are
even considered as edges*. On dense real boards this can pick a different
(still valid, still verified-clear) waypoint sequence for the same route. All
correctness tests still pass; if a specific board seems to be missing an
obviously-better route, `MIN_NEIGHBORS` (currently 24, in `buildEdges()`) is
the tunable floor.

To keep it fast on dense boards regardless:

- **Feed region-local obstacles**: filter `get_all_obstacles()` to a bbox around
  (start, target) before constructing the `Planner`. n stays in the tens →
  sub-10 ms.
- **Reuse the PNS world**: `load()`/`attach()` once, route many nets against it
  (PNS folds committed copper into the world). Do **not** reload per net.
- **Parallelise at the board-copy level** (one bridge per worker process).

---

## 15. Build & run

```bash
# Core: tests (ctest) + sanitizers — fully covered, passing.
cmake -B build -DGPLAN_SANITIZE=ON .
cmake --build build && ( cd build && ctest --output-on-failure )

# Python core module + demo of the full loop (mocked router):
pip install pybind11
cmake -B build -DBUILD_PYTHON=ON -Dpybind11_DIR=$(python3 -m pybind11 --cmakedir) .
cmake --build build && PYTHONPATH=build python3 example.py

# Bridge: builds inside the KiCad tree. Real link+runtime smoketest:
cmake -S <kicad-src> -B kbuild -DKICAD_BUILD_PNS_DEBUG_TOOL=ON
ninja -C kbuild gplan_bridge_smoketest
./kbuild/qa/tools/pns/gplan_bridge_smoketest qa/data/pcbnew/complex_hierarchy.kicad_pcb
```

For the bridge as a Python module (`gplan_kicad`), build `bridge_bindings.cpp` +
`pns_bridge.cpp` + `planner_core.cpp` into a pybind module linked against the same
libs as `gplan_bridge_smoketest` (see `qa/tools/pns/CMakeLists.txt`).

**Verified on real KiCad/PNS** (`complex_hierarchy`):
```
obstacles extracted: 739 (fixed=378, movable=361)
routeAndCheck:            ok=1 placed=1 vias=0 collided=0   single-layer shove, clean
multilayer routeAndCheck: ok=1 placed=1 vias=1 collided=0   F.Cu->via->B.Cu, clean
```

---

## 16. Strategy notes for the agent

- **Route order matters.** Earlier nets grab the good channels. Route critical /
  constrained nets first and treat them as fixed (locked) obstacles for the rest
  — this is "pre-routing". The congestion-feedback loop mitigates, doesn't
  eliminate, order sensitivity.
- **gplan replaces blind exploration, not perception.** You already have exact
  geometry via the collision oracle. Prefer geometric candidate generation over
  asking a vision model for coordinates (it can't measure clearance and can't be
  trusted for nm-precise points). If you use a vision model, use it to *rank*
  gplan's grounded candidates, never as the coordinate source.
- **Mode choice** (set in the bridge / PNS): shove (push copper aside) is the
  default; walkaround (bend new track only) is cheaper when placement is loose;
  mark-obstacles (no avoidance) only for "what's in the way" queries.

---

## 17. Known limitations (v1)

- Fixed obstacles are emitted as **bounding boxes** by the extractor (convex,
  conservative). Replace with real `Hull()` polygons for tighter packing.
- Via candidate sites are restricted to graph columns (obstacle corners +
  start/target). Fine for last-bits; add a denser grid if needed.
- Capacity/tightness are distance **approximations**, not swept-polygon clipping
  — intentional; PNS is the ground truth.
- `routeAndCheck` is speculative-only; a committing variant is a small addition.
- The bridge runs headless via a minimal `Pgm()` bootstrap; KiCad's global
  teardown is unreliable headless (the smoketest hard-exits after success). In a
  long-lived host process this is irrelevant.
- **One KiCad runtime per process.** `gplan_kicad` links its own wx/`PGM_BASE`/
  kiface singletons. Loading it alongside another KiCad/pcbnew instance in the
  same process (a plugin host, a second differently-built `gplan_kicad`, …)
  collides on those singletons → crash. Run gplan routing and a live pcbnew
  session as separate processes; hand geometry across as JSON
  (`RouteChange`/`RouteGeom` fields are all plain numbers/strings).
- **Don't lock everything to force a detour.** Locking ALL copper (instead of
  just the one footprint/via you need fixed) turns every trace into a `fixed`
  obstacle → `plan()`'s O(n³) build chokes on hundreds of hulls, and the
  movable "wiggle room" (§1) is gone, so PNS has no slack to shove through.
  Lock only what must not move.
- **`PnsBridge::load()`'s relative-path cwd trap.** A successful `load()` calls
  KiCad's `SETTINGS_MANAGER::LoadProject()`, which chdir's the process into the
  project's own directory as a side effect (not gated off in this headless
  bootstrap). `load()` resolves its OWN argument to absolute first, so a single
  relative path always works — but if you pass a SECOND relative path to a
  later `load()` call (same or a new `PnsBridge`, same process), it resolves
  against the drifted cwd, not the one your script started in. Always pass
  absolute paths to `load()` from a long-lived host/Python process.

---

## 18. Cheat sheet

```python
import gplan, gplan_kicad

br = gplan_kicad.PnsBridge(); br.load("board.kicad_pcb")        # board+rules+world
params = gplan.PlannerParams()
params.trackWidth = 200000; params.clearance = 200000
params.layers = [br.pns_layer(F_Cu), br.pns_layer(B_Cu)]; params.viaCost = 1_000_000
obs = br.get_all_obstacles()

planner = gplan.Planner(obs, params)
for path in planner.plan(start, s_layer, target, t_layer):     # cheapest first
    r = br.route_and_check(path.waypoints)
    if r.ok: break                                              # routed clean
    planner.bump_congestion(r.blocking, params.clearance*8, 5)  # learn, retry
```

**Rule of thumb:** geometry (gplan) proposes the *side and layer*; PNS lays the
*copper and shoves*; your loop *learns* from PNS failures via `bump_congestion`.
Never trust gplan output as DRC-final — `routeAndCheck`/PNS is the verdict.
