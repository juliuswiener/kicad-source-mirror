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

### 2.1 Yen's k-distinct paths ★★★ (M) — `planner_core.cpp:462`
Current k-paths re-runs full A* k× with an edge-penalty; not principled, can
return same-homotopy duplicates, O(k·n²log n). Replace with **Yen's algorithm**
(shortest-path tree once, then ranked deviations) [Yen71], and make distinctness
**homotopy-aware** via an **h-signature** so candidates are genuinely different
ways around obstacles, not just cost variants [Bhattacharya10]. Add a
diversity/dissimilarity filter so the k returned paths spread across homotopy
classes.

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

### 2.3 Swept-polygon edge blocking ★★ (M) — `planner_core.cpp:270`
24-point sampling can miss thin obstacles and is wasteful elsewhere. Build the
edge's swept rectangle (track width) once and test polygon–polygon intersection
via **Clipper2** or a Minkowski test. Correct and resolution-independent. [Clipper2]

### 2.4 Rigorous channel capacity ★★ (M) — `planner_core.cpp:305`
`capacity = floor(2·dist / pitch)` is a crude proxy. Compute true gap density
between the two bounding fixed obstacles (geometric corridor width / max-flow on
the channel) [Cheng05]. Sharpens congestion ranking; still a heuristic (PNS is
ground truth).

### 2.5 Region culling + per-plan caching ★★ (M) — `planner_core.cpp:445`
Add `plan(start,target,bbox)` / `setRegion(bbox)`; cache node/edge adjacency and
rebuild only when obstacles/bumps change (the 100-target loop currently rebuilds
the whole graph each call). Pairs with 2.2.

### 2.6 A* improvements ★ (S–M) — `planner_core.cpp:379`
Tie-breaking (secondary key) to cut equal-cost expansion; optional **bidirectional
A*** (≈√ frontier) and a **congestion-aware heuristic** (add a precomputed
congestion term to `h`) [Liu22, Adya98].

---

## 3. Tier 2 — routing methodology (Improvement, from literature)

### 3.1 Real multi-net PathFinder loop ★★★ (L)
Today `bumpCongestion` is per-connection. Lift it to a **board-level negotiated-
congestion loop**: route all nets, find over-capacity graph edges, raise their
*history cost*, rip-up & reroute only affected nets, iterate to convergence
[McMurchie95, nextpnr]. This is the single biggest quality lever — it makes the
result largely **net-order-independent**, the classic failure of greedy routers
(incl. Freerouting). Add **history-cost reset** and **troublemakers-last** seeding
[GLSVLSI19, GRIP/Ma21].

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
| 4.5 | **Multi-terminal / bus / Steiner** | `plan(vector<targets>)`; `SHOVE::AddHeads`/`ShoveMultiLines` (pns_shove.h:80) | nets with >2 pads; coordinated bus routing reduces order sensitivity | L | ★★ |
| 4.6 | **Post-route optimizer pass** | `OPTIMIZER::Optimize` (pns_optimizer.h:116): MERGE_SEGMENTS/SMART_PADS | 10-20% shorter, cleaner traces | M | ★★ |
| 4.7 | **Dragging / fixup pass** | `DRAGGER`/`MULTI_DRAGGER`/`COMPONENT_DRAGGER`, `StartDragging(...,DM_*)` (pns_router.h:208) | density recovery without full re-route | M | ★ |
| 4.8 | **Mode strategy (walkaround→shove)** | `ROUTING_SETTINGS` (pns_routing_settings.h): RM_Walkaround/RM_Shove, ShoveVias, JumpOver, free-angle, corner mode | multi-pass: polite first, shove second | S | ★★ |
| 4.9 | **Ratsnest-driven endpoints** | `GetNearestRatnestAnchor` (pns_router.h:247), `TOPOLOGY::NearestUnconnectedAnchorPoint` (pns_topology.h:63) | pick the *unrouted* ends automatically (fixes our placed=0 case) | M | ★★ |
| 4.10 | **Board outline + soft keepouts** | `board->GetBoardPolygonOutlines()`; zone priority | stop routing off-board; soft keepouts as cost not hard block | S | ★★ |
| 4.11 | **Real `Hull()` not bbox** | `pns_bridge.cpp:92` `bboxPoly` → `pad->GetEffectivePolygon`/PNS `Hull()` | tighter packing; less false congestion | M | ★★★ |
| 4.12 | **LOGGER replay** | `LOGGER` (pns_logger.h:48), `FormatLogFileAsJSON` | regression capture, audit, deterministic replay | S | ★ |
| 4.13 | **Zone-antipad pre-carve** — **DONE** | `gplan_zone_refill` (`global_planner/zone_refill_tool.cpp`), real `ZONE_FILLER::Fill()` | breaks the plane-via chicken-egg (via has no antipad until a fill runs WITH it present) found on a real board (RST_N/XVF3800 case study); a separate binary since real `ZONE_FILLER` only lives in `pcbnew_kiface_objects`, which conflicts symbol-for-symbol with the QA mocks the bridge/smoketest link for a light build | S | ★★★ |

(4.11 is listed here too because it's the bridge data-quality fix with the widest downstream effect.)

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

1. **Tier 0** (all) — small, unblocks parallel eval + real commits + reload.
2. **4.11 real hulls + 4.9 ratsnest endpoints + 4.10 board outline** — bridge data
   quality; biggest correctness gain for least code; makes routes realistic.
3. **2.1 Yen** (done) **+ 2.2 spatial index** (done — T-GRID + T-NEIGHBOR)
   **+ 2.5 region/caching** — planner quality + scale.
4. **3.1 board-level PathFinder loop** — order-independence; the quality unlock.
5. **4.6 optimizer + 4.8 mode strategy** — output cleanliness, cheap.
6. **4.1 diff pairs + 4.2 length tuning** — opens high-speed boards (large, high value).
7. **3.3 CDT backend** and **5.1/5.2 ML guides** — research bets once the above is solid.

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
