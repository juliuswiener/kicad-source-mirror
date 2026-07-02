# gplan — Improvement & Expansion Roadmap

Result of a multi-agent research pass: our own code (`global_planner/`, `pcbnew/router/`)
cross-referenced with the global-routing literature, open-source routers, and
ML-for-EDA work. Items are **Improvement** (make the current system better) or
**Expansion** (new capability), each with what/why/how (file:line) + effort/impact
+ references. References are listed in §8.

Legend: effort S/M/L, impact ★–★★★.

---

## 0. Where we stand

gplan today: inflated visibility graph → A* → k-paths by edge-reuse penalty →
PathFinder-style `bumpCongestion`, feeding KiCad PNS (shove) via a bridge that
extracts obstacles as **bounding boxes** and routes one net speculatively.
Architecturally this is the textbook global→detailed split [McMurchie95, Cheng05].
The gaps below are mostly *depth*, not direction.

---

## 1. Tier 0 — correctness & robustness quick wins (do first)

| # | Item | Why | How (file:line) | E | I |
|---|---|---|---|---|---|
| 0.1 | **Thread-safe `m_bumps`** | blocks the parallel multi-target eval we documented | `planner_core.cpp` `bumpCongestion` ~214; add mutex OR pass bumps into `plan()` (immutable) | S | ★★ |
| 0.2 | **Via-site interior check** | `viaSiteClear` only tests edge distance; a point *inside* a hull passes | `planner_core.cpp:293` add `pointInPolygon(p,hull)` | S | ★★ |
| 0.3 | **Input validation** | non-convex fixed silently hulled; bad layer dropped; `viaCost<0` breaks A* | `Planner` ctor; assert/ warn on convexity, unknown layer, neg costs | S | ★ |
| 0.4 | **`routeAndCommit` variant** | `routeAndCheck` is speculative-only → must re-route to commit | `pns_bridge.cpp:177` add `commit` flag → `CommitRouting()` | M | ★★ |
| 0.5 | **Reloadable bridge / teardown safety** | `_Exit()` hack means one board per process | wrap teardown; add `cleanup()`; test load→close→load | M | ★★ |
| 0.6 | **Error detail on `load()`** | bool hides file-vs-parse-vs-DRC failure | `pns_bridge_load.cpp:39`; add `std::string* err` | S | ★ |

---

## 2. Tier 1 — core algorithm upgrades (Improvement)

### 2.1 Yen's k-distinct paths ★★★ (M) — `planner_core.cpp` (T4) — **DONE**
Implemented as Yen's k-shortest loopless paths (commit `15679932bc`, tagged T4
in-code). Verified via ctest.

### 2.2 Spatial index for graph build ★★★ (L) — `planner_core.cpp:336` — **DONE**
`buildEdges` was O(n²·m) ≈ O(n³): every candidate edge tested every hull, AND
every node pair was enumerated as a candidate in the first place. Both fixed,
no Boost (core stays dependency-free):
- **T-GRID**: a per-layer uniform grid (`SpatialGrid`) answers "hulls near this
  edge/point" in ~O(k) instead of ~O(m). Provably bit-identical (the exact
  AABB/polygon test still runs on every grid-query result — the grid only
  narrows *which hulls get tested*). Verified bit-identical on ctest + both
  real-board smoketests.
- **T-NEIGHBOR**: `buildEdges`'s outer loop no longer enumerates all-pairs.
  Each non-start/target node queries an expanding ring (grid over node XY) for
  ≥24 neighbours instead of testing all n−1 others; start/target stay exempt
  (tested against every other node, O(n) extra) so a free direct sightline is
  never missed regardless of distance. **Not** bit-identical — narrows which
  node pairs are even considered as edges, so a dense real board can pick a
  different (still valid, still verified-clear) waypoint sequence. All
  correctness tests still pass.

Combined result (`perf_tests.cpp`, cumulative vs. brute force): n=200:
370ms→~10ms (**~37×**); n=885 (real large-session obstacle count): would time
out → **~86ms**; n=1500: would time out → ~200ms. Scaling from n=200→1500
(7.5×) now costs only ~20× the time — close to linear, not the prior
quadratic-to-cubic growth. See README § Performance & scaling for the full
before/after table and the T-NEIGHBOR trade-off writeup.

### 2.3 Swept-polygon edge blocking ★★ (M) — `planner_core.cpp` (T6, `segHitsConvex`) — **DONE**
Implemented as an exact Liang-Barsky/Cyrus-Beck half-plane clip against convex
CCW hulls (commit `208a26ee22`) — resolution-independent, replaces the old
24-point sampling. Verified via ctest.

### 2.4 Rigorous channel capacity ★★ (M) — `planner_core.cpp` (`edgeWeight`) — **DONE**
`edgeWeight` now computes a geometric corridor width: the nearest fixed
obstacle on EACH side of the a→b line (side judged by obstacle bbox center,
reusing the existing ring-doubling `fixedGrid` query with a per-side T5 prune),
so `gap = dLeft + dRight` instead of the old `2·dFix` single-obstacle proxy; an
empty side falls back to the old 10·pitch cap. Still `capacity =
floor(gap/pitch)` and still a congestion-RANKING heuristic (PNS is ground
truth); `dFix = min(dLeft, dRight)` keeps the tightness term unchanged.
Verified via ctest + both bridge smoketests + ASan/UBSan build.

### 2.5 Region culling + per-plan caching ★★ (M) — `planner_core.cpp` (T-CACHE/T-REGION) — **DONE**
`plan()` now splits the graph into a cached corner-only part (inflated-hull
nodes + edges, independent of start/target) and a tiny per-plan start/target
overlay (appended nodes, connected exhaustively — the old src/dst exemption,
generalised). The cache is rebuilt only when its inputs change
(`bumpCongestion`/`clearCongestion`, region change); obstacles are
ctor-immutable, so a multi-target loop over one obstacle set costs ONE build
(`graphBuildCount()` exposes this; ctest asserts 3 plans = 1 build and that
cache-hit results are byte-identical to a cold, uncached instance).
`setRegion(bbox)` / `plan(start,target,bbox)` cull corner nodes to the bbox;
re-setting the same region keeps the cache warm. Validation also surfaced a
latent heap-layout-dependent use-after-free in `PnsBridge::load()` reload
(old board's `BOARD_DESIGN_SETTINGS` outliving its project's
`JSON_SETTINGS` parent) — fixed via `ClearProject()` before swapping
`m_settings` (`pns_bridge_load.cpp`).

### 2.6 A* improvements ★ (S–M) — `planner_core.cpp` (T-TIEBREAK) — **DONE**
Tie-breaking (secondary key) implemented: the `aStar` priority-queue entry is now
`(f, -g, node)`, so equal-f ties prefer the HIGHER-g (deeper) node — on a
visibility graph this walks straight down an optimal corridor instead of
expanding the whole equal-cost front — with node index as a final deterministic
key; carrying g in the entry also makes the stale-entry check exact (no
`f - h` roundoff). Verified via ctest + bridge smoketests + ASan/UBSan build.
**Bidirectional A*** (≈√ frontier) and the **congestion-aware heuristic**
(precomputed congestion term in `h`) [Liu22, Adya98] are explicitly DEFERRED —
not implemented this pass.

---

## 3. Tier 2 — routing methodology (Improvement, from literature)

### 3.1 Real multi-net PathFinder loop ★★★ (L) — `global_planner/pathfinder.h` (T11) — **DONE**
Implemented as a header-only, KiCad-free `gplan::routeNets()` (commit
`b067dd5447`): each committed net becomes a FIXED obstacle for the others
(order-independence), deadlock detection rips up + congestion-bumps a blocking
net when no net makes progress in a round. Own ctest target
(`pathfinder_tests`, part of the standard 4/4 suite). History-cost
reset/troublemakers-last seeding from the original writeup are NOT yet
present — the deadlock-driven rip-up covers the core negotiated-congestion
loop but not that refinement.

### 3.2 Two-phase pattern→maze ★★ (M)
Fast first pass with L/Z/monotone "pattern" paths on the visibility graph for all
nets; fall back to full A*/shove only on nets that overflow [FastRoute]. Big
speedup on dense boards.

### 3.3 CDT free-space backend (alt graph) ★★★ (L) — research bet
Replace/augment the inflated-corner visibility graph with a **constrained Delaunay
triangulation** of free space; route on the dual graph and pull taut with the
**funnel / string-pulling** algorithm to get minimal-length arbitrary-angle paths
with far fewer nodes [Dayan97, Ozdal08, Guibas-Hershberger89, Lee82]. This is the
"rubber-band/topological" model real topological routers use; it also gives
rigorous capacity. Largest architectural change; highest ceiling.

### 3.4 Funnel taut-path post-step ★★ (M) — cheap subset of 3.3
Even on the current graph, run **string-pulling** on each candidate to shorten and
remove needless corners before handing waypoints to PNS [Guibas-Hershberger89].
Better waypoints → fewer PNS shove iterations.

---

## 4. Tier 3 — feature expansions via PNS (Expansion)

The bridge uses ~5% of PNS. High-value capabilities already implemented in
`pcbnew/router/`, just not exposed:

| # | Feature | PNS entry (file) | Why | E | I |
|---|---|---|---|---|---|
| 4.1 | **Differential pairs** | `DIFF_PAIR_PLACER` (pns_diff_pair_placer.h:52), `SetMode(PNS_MODE_ROUTE_DIFF_PAIR)` (pns_router.h:157) | USB/PCIe/LVDS — 30-50% of modern nets; auto-pair detection via `FindDpPrimitivePair` | L | ★★★ |
| 4.2 | **Length / delay / skew tuning** | `MEANDER_PLACER` / `DP_MEANDER_PLACER` / `MEANDER_SKEW_PLACER`, `PNS_MODE_TUNE_*`; `ROUTER_IFACE::CalculateRoutedPathLength/Delay` (pns_router.h:124) | high-speed timing closure (DDR/clocks) | L | ★★★ |
| 4.3 | **Per-net class width/clearance** | netclasses already in `BOARD`; thread through bridge | correct constraints per net instead of global params | M | ★★ |
| 4.4 | **Blind/buried vias** | relax adjacency in `planner_core.cpp:368`; config via-connectivity | escape routing on HDI boards (currently through-only adjacent) | M | ★★ |
| 4.5 | **Multi-terminal / bus / Steiner** — **DONE** | `Planner::planMultiTerminal` (`planner_core.{h,cpp}`) | nets with >2 pads; coordinated bus routing reduces order sensitivity | L | ★★ |
| 4.6 | **Post-route optimizer pass** — **DONE** | `PnsBridge::optimizeRoute` (`pns_bridge.{h,cpp}`), `OPTIMIZER::Optimize` (pns_optimizer.h:116): MERGE_SEGMENTS+SMART_PADS | 10-20% shorter, cleaner traces | M | ★★ |
| 4.7 | **Dragging / fixup pass** | `DRAGGER`/`MULTI_DRAGGER`/`COMPONENT_DRAGGER`, `StartDragging(...,DM_*)` (pns_router.h:208) | density recovery without full re-route | M | ★ |
| 4.8 | **Mode strategy (walkaround→shove)** — **DONE** | `PnsBridge::routeWithStrategy` (`pns_bridge.{h,cpp}`): `ROUTING_SETTINGS` RM_Walkaround/RM_Shove via `setMode` | multi-pass: polite first, shove second | S | ★★ |
| 4.9 | **Ratsnest-driven endpoints** | `GetNearestRatnestAnchor` (pns_router.h:247), `TOPOLOGY::NearestUnconnectedAnchorPoint` (pns_topology.h:63) | pick the *unrouted* ends automatically (fixes our placed=0 case) | M | ★★ |
| 4.10 | **Board outline** — **DONE** | `board->GetBoardPolygonOutlines()` (T8, `pns_bridge.cpp` `getObstacles`) | thin FIXED wall around the board edge on every layer so the planner never proposes off-board routes | S | ★★ |
| 4.11 | **Real `Hull()` not bbox** — **DONE** | `pad->GetEffectivePolygon(bl)` (T7) for pads; `zone->Outline()` (this pass) for keepout zones — both replace `bboxPoly`. Only vias still use bbox (acceptable: near-circular already) | tighter packing; less false congestion | M | ★★★ |
| 4.12 | **LOGGER replay** | `LOGGER` (pns_logger.h:48), `FormatLogFileAsJSON` | regression capture, audit, deterministic replay | S | ★ |
| 4.13 | **Zone-antipad pre-carve** — **DONE** | `gplan_zone_refill` (`global_planner/zone_refill_tool.cpp`), real `ZONE_FILLER::Fill()` | breaks the plane-via chicken-egg (via has no antipad until a fill runs WITH it present) found on a real board (RST_N/XVF3800 case study); a separate binary since real `ZONE_FILLER` only lives in `pcbnew_kiface_objects`, which conflicts symbol-for-symbol with the QA mocks the bridge/smoketest link for a light build | S | ★★★ |
| 4.14 | **clearEscapeCorridor** — **DONE** | `PnsBridge::clearEscapeCorridor` (`pns_bridge.{h,cpp}`) | automates the manual "probe → find nearest blocker → shove_via/component_search → reprobe" cascade for a fanout-saturated pin escape (RST_N/XVF3800 case study, Wall A); composes existing `routeAndCheck`/`shoveViaSearch`/`shoveComponentSearch`, no new PNS surface | S | ★★ |

(4.11 is listed here too because it's the bridge data-quality fix with the widest downstream effect.)

### 4.5 Multi-terminal / bus / Steiner ★★ (L) — `planner_core.{h,cpp}` (`planMultiTerminal`) — **DONE**
`Planner::planMultiTerminal(terminals)` is core-side (KiCad-free), a greedy
Steiner-tree heuristic over the existing point-to-point `plan()`: starting
from `terminals[0]`, it repeatedly connects the cheapest (unconnected
terminal, already-connected point) pair, growing a tree one edge at a time
(Prim's-algorithm structure, reusing `plan()` as the edge-cost oracle instead
of a new graph primitive). This is what "reduces order sensitivity" means in
practice: on a graph with distinct edge costs the resulting tree — and its
total cost — is the same regardless of which permutation of terminals is
passed in, unlike routing a caller-supplied chain start->t1->t2->... where a
badly-ordered chain pays for detours a cost-driven choice would avoid.
Returns one `Path` per newly-connected terminal (`terminals.size() - 1` on
success). Verified by `T-MULTI` in `tests.cpp`: two different permutations of
the same 4-terminal set (one deliberately bad) produce the same total tree
cost. `SHOVE::AddHeads`/`ShoveMultiLines` (bridge-side coordinated multi-line
shove) remains open — this closes the core-side planning gap only.

### 4.6 Post-route optimizer pass ★★ (M) — `pns_bridge.{h,cpp}` (`optimizeRoute`) — **DONE**
`PnsBridge::optimizeRoute(x, y, pnsLayer)` assembles the joint-to-joint LINE
under the probe point from the committed PNS world (`NODE::AssembleLine`) and
runs the real `PNS::OPTIMIZER` on it with MERGE_SEGMENTS + SMART_PADS — the
same effects the interactive router applies post-shove (pattern verified
against `SHOVE::runOptimizer` / `LINE_PLACER`). An improvement is applied on a
`NODE::Branch` (`Replace`), collision-checked, and committed via
`ROUTER::CommitRouting(NODE*)` — the same lossless parent-matched change
stream as `routeAndCommit` (removed-by-uuid + added copper), so the host can
mirror the cleanup onto the board. Returns before/after length + corner count
(`OptimizeResult`); "no improvement" is an honest ok=true no-op, a colliding
result is refused. Python: `optimize_route`. Verified via a bridge_smoketest
block that routes+commits a real net, optimizes at a committed-segment
midpoint, and prints length before/after (improvement is board-dependent, not
asserted; found+collision-free is).

### 4.8 Mode strategy (walkaround→shove) ★★ (S) — `pns_bridge.{h,cpp}` (`routeWithStrategy`) — **DONE**
`setMode`/`RouteMode` (walkaround/shove/mark-obstacles) already existed (T12)
as manual primitives; there was no automated multi-pass STRATEGY.
`PnsBridge::routeWithStrategy(waypoints)` tries the SAME waypoints under
RM_Walkaround first (polite — never shoves existing copper) via the ordinary
`routeAndCommit`, and only if that pass fails to reach the target, retries
once under RM_Shove (the aggressive fallback). `routeAndCommit` only commits a
reached+collision-free result, so a failed walkaround pass leaves the world
clean for the shove retry. The bridge's configured mode is restored on every
exit path, mirroring the placer-mode restore already used by
`routeDiffPairAndCommit`. Python: `route_with_strategy`. Verified via a
bridge_smoketest block that un-routes net 1 (T9's pattern), then routes it
through `routeWithStrategy` and asserts the commit reaches the target.

---

## 5. Tier 4 — ML, selectively (Expansion)

Bottom line from the survey: **ML helps as a guide, not as the router.** Keep PNS
+ geometry as the engine.

| # | Item | Why | E | I |
|---|---|---|---|---|
| 5.1 | **Learned net-ordering ranker** | cheap (<1 ms/net), 5-15% overflow reduction; "hardest/troublemakers-last" [Ma21/GRIP, MARouter/Zhu23] | M | ★★ |
| 5.2 | **Congestion/routability prediction (GNN/CNN)** | predict hot regions → cost map for the planner; most mature ML-EDA area [Xie20, Tang22] | L | ★★ |
| 5.3 | **Learned A* heuristic** | 20-40% fewer node expansions [Marcano22, NeuralA*/Yonetani21]; only if A* becomes the bottleneck | L | ★ |
| 5.4 | **LLM = orchestrator only** | parse intent, pick params/strategy, drive rip-up/retry; never generate geometry [ChatEDA/Wu24, CircuitRouter/Bae24] | S | ★ |
| — | **Skip:** end-to-end RL routing & LLM layout generation — poor generalization; analytical/geometric methods win [Cheng-Yan21, Agnesina23] | — | — |

Train data is free: log your own router's successes/failures (4.12) and learn from them.

---

## 6. Tier 5 — testing & infra (Improvement)

- Adversarial geometry: near-touching hulls (2% eps band), sub-trackWidth channels,
  collinear/degenerate vertices, scale extremes (±1e9, ±1e-3).
- Bridge integration: DRC-violation detection, locked copper shoves as soft,
  multi-layer via on complex geometry, RM_Walkaround vs RM_Shove, unreachable/timeout.
- Perf regression harness (assert n=100 < 100 ms), Python binding round-trip tests,
  property/fuzz tests (non-negative cost, distinct waypoints, reachability).

---

## 7. Recommended sequence

1. **Tier 0** (all) — **done**.
2. **4.11 real hulls + 4.9 ratsnest endpoints + 4.10 board outline** — **done**.
3. **2.1 Yen** (done) **+ 2.2 spatial index** (done — T-GRID + T-NEIGHBOR)
   **+ 2.5 region/caching** (done — T-CACHE + T-REGION) — planner quality + scale.
4. **3.1 board-level PathFinder loop** — **done** (`pathfinder.h`, T11).
5. **4.5 multi-terminal** — **done** (`planMultiTerminal`); **4.6 optimizer** — **done**
   (`optimizeRoute`); **4.8 mode strategy** — **done** (`routeWithStrategy`).
6. **4.1 diff pairs + 4.2 length tuning** — **done**.
7. **4.13 zone-antipad pre-carve + 4.14 clearEscapeCorridor** — **done** (found via a
   real-board case study, not the original research pass).
8. **3.3 CDT backend** and **5.1/5.2 ML guides** — research bets once the above is
   solid; still open.

Genuinely open as of this writing: 3.2, 3.3, 3.4, 4.3,
4.7, 4.12, all of Tier 4 (ML), Tier 5 (testing/infra hardening).

---

## 8. References

Algorithms / global routing:
- McMurchie & Ebeling, "PathFinder," FPGA'95 — negotiated congestion. doi:10.1145/214564.214594
- Pan et al., "FastRoute," IEEE TCAD 2012 — pattern routing. doi:10.1109/TCAD.2011.2177081
- Wen et al., "NCTU-GR 2.0," IEEE TCAD 2013. doi:10.1109/TCAD.2013.2249556
- Ao et al., "CUGR," ISPD'14. doi:10.1145/2591720.2591733
- GLSVLSI'19, history-cost adjustment. doi:10.1145/3299901.3319814
- Adya et al., "Hierarchical Global Routing," IEEE TCAD 1998.
- Liu et al., "A* with Adaptive Heuristics," DAC'22. doi:10.1145/3489517.3530471

Topological / homotopy / geometry:
- Dayan, "Rubber-band based topological router," PhD UCSC 1997.
- Ozdal & Wong, "Obstacle-Avoiding Steiner Trees," IEEE TCAD 2008. doi:10.1109/TCAD.2008.923256
- Bhattacharya et al., "Path planning with homotopy class constraints," ICRA 2010. arXiv:1004.2982
- Guibas & Hershberger, "Shortest path queries in a simple polygon" (funnel), JCSS 1989.
- Yen, "k shortest loopless paths," Mgmt Sci 1971. doi:10.1287/mnsc.17.11.712
- Cheng et al., "Theories and Algorithms for PCB Layout," IEEE CAS Mag 2005.

ML for EDA:
- Mirhoseini et al., chip placement RL, Nature 2021 — and critiques Cheng-Yan (arXiv:2111.11734), Agnesina (arXiv:2301.11280).
- Xie et al., "Routability-Driven Analytical Placement via GNN," ICCAD 2020.
- Tang et al., "PCB Routing Congestion Prediction via GNN," DATE 2022.
- Marcano et al., "Learn-to-Route," ICCAD 2022; Yonetani et al., "Neural A*," ICLR 2021.
- Ma et al., "GRIP," ICCAD 2021; Zhu & Chen, "MARouter," DAC 2023.
- Wu et al., "ChatEDA," DAC 2024; Bae et al., "CircuitRouter-Agent," arXiv:2409.04512.

Open-source / libraries:
- Freerouting — github.com/freerouting/freerouting (maze + rip-up; no global congestion view).
- nextpnr — github.com/YosysHQ/nextpnr (PathFinder).
- TritonRoute / OpenROAD; CUGR — github.com/jshsun/CUGR.
- Clipper2 — github.com/AngusJohnson/Clipper2 (offset/clip).
- boost.geometry R-tree; boost.polygon (Voronoi); DREAMPlace.
