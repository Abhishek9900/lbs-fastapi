# Navigation.py — Complete Technical Documentation

> **File:** `Navigation.py`  
> **Algorithm:** Dijkstra’s algorithm and Abraham et al. [2013] — *Alternative Routes in Road Networks* (simplified)

---

## Table of Contents

1. [Module Overview](#1-module-overview)
2. [Dependencies & Graph Model](#2-dependencies--graph-model)
3. [Module-Level Constant](#3-module-level-constant)
4. [Public API](#4-public-api)
5. [Private Core Functions](#5-private-core-functions)
6. [Helper Functions](#6-helper-functions)
7. [Algorithm Deep-Dive: `_alternative_paths()`](#7-algorithm-deep-dive-_alternative_paths)
8. [Parameters: α, γ, ε](#8-parameters-α-γ-ε)
9. [fast_mode: Speedup Heuristic](#9-fast_mode-speedup-heuristic)
10. [Walking vs. Wheelchair Mode](#10-walking-vs-wheelchair-mode)
11. [Error Handling](#11-error-handling)
12. [Quick-Reference Table](#12-quick-reference-table)

---

## 1. Module Overview

`Navigation.py` is the **routing engine** of the map navigation web application. It exposes four public functions that compute shortest paths and meaningful alternative routes for both walking and wheelchair users, using road graph data loaded from OpenStreetMap (OSM).

The core algorithm is a simplified implementation of the **Abraham et al. [2013]** *via-node* approach: instead of the paper's true bidirectional Dijkstra search, two independent Dijkstra runs (one forward from the source, one backward from the target on the reversed graph) are combined at every candidate via-node to form alternative paths.

### What the module does

```
User clicks two points on the map
           │
           ▼
   lat/lon coordinates
           │
     ┌─────┴──────┐
     ▼            ▼
shortest_path   alternative_paths
  (walking /      (walking /
  wheelchair)     wheelchair)
     │            │
     │            ▼
     │          fast_mode
     │            │
     │      ┌─────┴──────┐
     │      ▼            ▼
     │     True         False
     │      └─────┬──────┘
     │            ▼
     │            │
     └─────┬──────┘
           ▼
  List of coordinate dicts
  [{"lat": ..., "lon": ...}, ...]
```

---

## 2. Dependencies & Graph Model

| Dependency | Role |
|---|---|
| `networkx` | Dijkstra's algorithm, graph reversal, path queries |
| `GraphHolder` | Singleton that provides the loaded OSM `MultiDiGraph` |
| `Util` | `findClosestNode()`, `pathsToCoordinates()`, `updateGraphWeights()` |

### Graph type

The underlying graph is a **`networkx.MultiDiGraph`** — directed, with potentially multiple parallel edges between the same pair of nodes (e.g. two lane directions on a dual carriageway). All edge-weight lookups use the minimum weight across parallel edges.

### Edge weight attributes

| Attribute | Used for | Set by |
|---|---|---|
| `"length"` | Walking — geographic distance in metres | OSM data at load time |
| `"weight"` | Wheelchair — custom accessibility cost | `updateGraphWeights()` before each query |

The entire algorithm is **weight-agnostic**: swapping the weight string between the two modes requires no algorithmic changes.

---

## 3. Module-Level Constant

```python
_GRID_STEP = 0.002
```

Grid cell size in latitude/longitude degrees used by the spatial clustering speedup in `fast_mode`. At mid-European latitudes this corresponds to approximately **200 metres**. Via-nodes within the same grid cell produce near-identical paths; only the cheapest is kept. Increase this value to cluster more aggressively (fewer T-test calls, slightly lower diversity); decrease it for finer clustering.

---

## 4. Public API

All four public functions follow the same contract:

- **Input:** a `request` object carrying coordinate and parameter attributes.
- **Output:** a list of coordinate lists `[[{"lat": float, "lon": float}, ...], ...]`.
- **Error:** raises `Exception` (shortest-path functions) or returns `[]` (alternative-path functions) on failure.

---

### `shortest_path_by_walking(request)`

Computes the single shortest walking path between two coordinates using geographic distance.

**Request attributes used:**

| Attribute | Type | Description |
|---|---|---|
| `lat1`, `lon1` | float | Start coordinate |
| `lat2`, `lon2` | float | End coordinate |

**Returns:** `[{"lat": float, "lon": float}, ...]` — one coordinate list.

**Raises:** `Exception` if source == target, a node is not in the graph, or no path exists.

**Example:**
```python
path = shortest_path_by_walking(request)
# path == [{"lat": 50.73, "lon": 7.10}, {"lat": 50.731, "lon": 7.101}, ...]
```

---

### `shortest_path_by_wheelchair(request)`

Computes the single shortest path optimised for wheelchair accessibility. Before running Dijkstra, calls `updateGraphWeights(graph, request.distance_weight)` to re-score all edges according to slope, surface type, kerb cuts, etc.

**Request attributes used:**

| Attribute | Type | Description |
|---|---|---|
| `lat1`, `lon1` | float | Start coordinate |
| `lat2`, `lon2` | float | End coordinate |
| `distance_weight` | float | Passed to `updateGraphWeights()` to control the accessibility penalty weighting |

**Returns:** `[{"lat": float, "lon": float}, ...]` — one coordinate list.

**Raises:** Same as `shortest_path_by_walking`.

---

### `alternative_paths_by_walking(request, via_path_cutoff=1.4, fast_mode=False)`

Finds the shortest walking path plus up to two meaningfully different alternatives, all satisfying the three Abraham et al. quality constraints.

**Request attributes used:**

| Attribute | Type | Description |
|---|---|---|
| `lat1`, `lon1`, `lat2`, `lon2` | float | Coordinates |
| `epsilon` | float | Stretch tolerance — alternatives may be at most `(1+ε)×` the optimal length |
| `gamma` | float | Sharing bound — shared edge cost must be `≤ γ × l_opt` |
| `alpha` | float | Local-optimality radius — `T = α × l_opt` |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `via_path_cutoff` | `1.4` | Ignored — overridden internally to `1 + request.epsilon` |
| `fast_mode` | `False` | If `True`, enables spatial clustering + T-test bypass (~2× speedup) |

**Returns:** `[[opt_coords], [alt1_coords]?, [alt2_coords]?]` — 1 to 3 coordinate lists.

---

### `alternative_paths_by_wheelchair(request, via_path_cutoff=1.4, fast_mode=False)`

Same as `alternative_paths_by_walking` but applies `updateGraphWeights()` first to score edges by accessibility before running the algorithm with `weight="weight"`.

**Additional request attribute:**

| Attribute | Type | Description |
|---|---|---|
| `distance_weight` | float | Passed to `updateGraphWeights()` |

**Returns:** `[[opt_coords], [alt1_coords]?, [alt2_coords]?]`

---

## 5. Private Core Functions

### `_shortest_path(lat1, lon1, lat2, lon2, weight="length")`

Shared engine for both public shortest-path functions.

**Steps:**
1. Fetch graph from `GraphHolder.get_graph()`
2. Snap coordinates to nearest graph nodes via `findClosestNode()`
3. Guard against: same source/target, node not in graph, disconnected graph
4. Run `nx.dijkstra_path(graph, source, target, weight=weight)`
5. Return `pathsToCoordinates(graph, [path], weight)`

---

### `_cluster_candidates_by_grid(graph, candidates_sorted, max_candidates)`

**Purpose:** Speedup heuristic for `fast_mode`. Reduces the candidate via-node list before the expensive inner constraint loop runs.

**How it works:**
- Reads each node's `y` (latitude) and `x` (longitude) attributes from the OSM graph
- Assigns each node to a grid cell: `cell = (round(lat / _GRID_STEP), round(lon / _GRID_STEP))`
- Keeps only the **first (cheapest)** candidate per cell, since `candidates_sorted` is already sorted by `via_cost`
- Applies the `max_candidates` cap after clustering

**Why it's safe:** Two via-nodes in the same ~200 m cell produce paths that are geographically almost identical. Running the T-test and sharing filter on both would yield the same result — keeping only the cheaper one is a correct optimisation.

**Parameters:**

| Parameter | Type | Description |
|---|---|---|
| `graph` | MultiDiGraph | The road graph (nodes carry `y`=lat, `x`=lon attributes) |
| `candidates_sorted` | list of `(node, float)` | Candidates sorted ascending by `via_cost` |
| `max_candidates` | int | Hard cap applied after clustering |

**Returns:** Filtered list of `(via_node, via_cost)` tuples, capped at `max_candidates`.

**Logs:** `DEBUG` level — number of candidates before/after and percentage reduction.

---

### `_alternative_paths(lat1, lon1, lat2, lon2, weight, via_path_cutoff, epsilon, gamma, alpha, fast_mode=False)`

The main algorithm. See [Section 7](#7-algorithm-deep-dive-_alternative_paths) for a complete step-by-step walkthrough.

**Parameters:**

| Parameter | Type | Description |
|---|---|---|
| `lat1, lon1, lat2, lon2` | float | Source and target coordinates |
| `weight` | str | `"length"` for walking, `"weight"` for wheelchair |
| `via_path_cutoff` | float | Dijkstra search radius = `via_path_cutoff × l_opt`. Set to `1 + epsilon` by callers |
| `epsilon` | float | UBS stretch bound |
| `gamma` | float | Sharing bound |
| `alpha` | float | Local-optimality radius |
| `fast_mode` | bool | Enables clustering + T-test bypass when `True` |

**Returns:** `[[opt_coords], [alt1_coords]?, [alt2_coords]?]`

**Returns `[]` on any unhandled exception** — logs full traceback at `ERROR` level.

---

## 6. Helper Functions

All helpers are private (prefixed `_`) and used internally by the core functions.

### Validation helpers

| Function | Returns | Description |
|---|---|---|
| `_is_hashable(obj)` | `bool` | Returns `True` if `obj` can be used as a dict key / set member |
| `_validate_path(path)` | `bool` | Checks `path` is a non-empty list of hashable, non-container nodes |
| `_validate_path_edges(graph, path)` | `bool` | Checks every consecutive pair in `path` has a real edge in the graph; calls `_validate_path` first |

### Conversion helpers

| Function | Returns | Description |
|---|---|---|
| `_path_edges_to_set(path)` | `set` or `None` | Converts node list to a `set` of `(u, v)` edge tuples for O(1) intersection; returns `None` if path is invalid |
| `_nodes_to_set(path)` | `set` or `None` | Converts node list to a `set` of node IDs; returns `None` if any node is unhashable |

### Cost helpers

| Function | Returns | Description |
|---|---|---|
| `_path_cost(graph, path, weight)` | `float` | Sums edge weights along a node-list path using `_get_edge_weight` |
| `_get_edge_weight(graph, u, v, weight)` | `float` | Safely retrieves one edge's weight. For `MultiDiGraph` returns the **minimum** across all parallel edges from `u` to `v`. Returns `0` if edge is missing |
| `_sum_edge_weights(graph, edges, weight)` | `float` | Sums `_get_edge_weight` over a set of `(u, v)` edge tuples |

### Algorithm helpers

#### `_t_test(graph, path_sv, path_vt, v, T, weight) → bool`

Implements the **T-local-optimality test** from Abraham et al. Lemma 4.1.

**Steps:**
1. Walk **backward** along `path_sv` (the S→V sub-path) from `v`, accumulating edge cost, until `≥ T` is reached → anchor node `x`. If the whole path is shorter than `T`, `x = source`.
2. Walk **forward** along `path_vt` (the V→T sub-path) from `v`, similarly → anchor node `y`. If shorter than `T`, `y = target`.
3. Extract the sub-path `x → y` from the combined via-path.
4. Compare its cost against `nx.dijkstra_path_length(graph, x, y, weight)`.
5. **Pass** if `subpath_cost ≤ dist(x, y) × 1.001` (float tolerance).

**Guarantees (from paper):**
- Lemma 4.1: if the test **passes** → the via-path is T-locally-optimal.
- Lemma 4.2: if it **fails** → the via-path is not even 2T-locally-optimal (correct rejection).

**Cost:** One `nx.dijkstra_path_length()` call per candidate — the dominant runtime cost of the inner loop. This is what `fast_mode` bypasses.

---

#### `_longest_plateau(graph, via_path, dist_s, dist_t, weight) → float`

Computes the **plateau length** from Abraham et al. Lemma 4.4.

A *plateau* is the longest contiguous sub-path of `via_path` where every node appears in **both** the forward shortest-path tree (reachable from source) and the backward shortest-path tree (reachable from target) simultaneously. By Lemma 4.4, a plateau of length L guarantees L-local-optimality.

The plateau length is used as a **bonus term** in the ranking objective:

```
f(v) = 2 × via_cost + shared_cost − plateau_len
```

Lower score = preferred. Subtracting `plateau_len` means paths with longer plateaus (stronger local-optimality) rank higher among admissible candidates.

---

## 7. Algorithm Deep-Dive: `_alternative_paths()`

### Overview

The function implements the **BDV algorithm** from Abraham et al. [2013] Section 4.1, simplified by replacing the true bidirectional Dijkstra (BD) with two independent single-source Dijkstra runs. This trades a small amount of theoretical optimality for a much simpler implementation.

### Step-by-step walkthrough

```
Step 1 ── Snap coordinates to graph nodes
Step 2 ── Compute optimal path (Opt) and its cost l_opt
Step 3 ── Forward Dijkstra:  dist_s, paths_s  from source
Step 4 ── Backward Dijkstra: dist_t, paths_t  from target on reversed graph
Step 5 ── Collect via-node candidates + UBS filter (constraint c)
Step 6 ── Sort, optionally cluster (fast_mode), iterate:
        ├─ Reconstruct path_sv and path_vt
        ├─ Validate edges exist in forward graph  [BUG FIX]
        ├─ Constraint (a): sharing check
        └─ Constraint (b): T-test  [skipped in fast_mode]
Step 7 ── Score, sort admissible paths, de-duplicate, pick top 2
Step 8 ── Return [opt_path] + selected as coordinates
```

---

### Step 1: Node snapping and validation

```python
source = findClosestNode(lat1, lon1, graph)
target = findClosestNode(lat2, lon2, graph)
```

Guards: same node, node missing from graph, no path between them (`nx.has_path`).

---

### Step 2: Compute Opt

```python
opt_path = nx.dijkstra_path(graph, source, target, weight=weight)
l_opt = _path_cost(graph, opt_path, weight)
```

`l_opt` is the cost of the globally optimal path. All three constraints are expressed as multiples of `l_opt`, so it must be computed first. If `l_opt == 0` (adjacent nodes with zero-weight edge), only the optimal path is returned.

---

### Step 3: Forward Dijkstra

```python
cutoff = via_path_cutoff * l_opt   # = (1 + epsilon) * l_opt
dist_s, paths_s = nx.single_source_dijkstra(graph, source, cutoff=cutoff, weight=weight)
```

The `cutoff` terminates the search early — only nodes within `(1+ε) × l_opt` of the source are explored. This is the key performance optimisation: without it, the entire graph would be searched for every query. `paths_s[v]` gives the list of nodes on the shortest path from `source` to `v`.

---

### Step 4: Backward Dijkstra

```python
graph_rev = graph.reverse(copy=True)
dist_t, paths_t = nx.single_source_dijkstra(graph_rev, target, cutoff=cutoff, weight=weight)
```

Reversing the graph converts "distance from target" into a standard single-source query. `dist_t[v]` gives the distance from `v` to `target` in the original graph. `paths_t[v]` is the path `[target, ..., v]` — note the **reverse order**, which is corrected in Step 6.

> **Note:** `graph.reverse(copy=True)` creates a new graph object on every call. For high-throughput deployments, caching the reversed graph in `GraphHolder` would eliminate this allocation.

---

### Step 5: Candidate collection + UBS filter (constraint c)

```python
for v in dist_s:
    if v not in dist_t or v == source or v == target:
        continue
    via_cost = dist_s[v] + dist_t[v]
    if via_cost > (1 + epsilon) * l_opt:   # constraint (c)
        continue
    candidates.append((v, via_cost))
```

`dist_s[v] + dist_t[v]` gives the cost of the via-path through `v` (source→v→target). Applying the UBS constraint here — before reconstructing any full path — eliminates the majority of candidates cheaply. Only nodes reachable from both sides within the stretch budget survive.

---

### Step 6: Inner constraint loop

Candidates are sorted by `via_cost` (cheapest first). In `fast_mode`, `_cluster_candidates_by_grid()` is called before the loop. The top 500 candidates are processed (`MAX_CANDIDATES = 500`).

For each `(via_node, via_cost)`:

**Path reconstruction:**
```python
path_sv = paths_s.get(via_node)            # [source, ..., via_node]
path_vt = list(reversed(paths_t[via_node]))  # [via_node, ..., target]
via_path = path_sv + path_vt[1:]           # via_node appears once
```

**Edge validation (bug fix):**
```python
sv_ok = all(graph.has_edge(pa, pb) for pa, pb in zip(path_sv[:-1], path_sv[1:]))
vt_ok = all(graph.has_edge(pa, pb) for pa, pb in zip(path_vt[:-1], path_vt[1:]))
if not sv_ok or not vt_ok:
    continue    # skip — previously this only logged without skipping
```

**Constraint (a) — Limited Sharing:**
```python
shared_edges = opt_edges & via_edges
shared_cost = _sum_edge_weights(graph, shared_edges, weight)
if shared_cost > gamma * l_opt:
    continue
```

**Constraint (b) — T-local optimality:**
```python
if not fast_mode:
    if not _t_test(graph, path_sv, path_vt, via_node, T, weight):
        continue
```

**Scoring:**
```python
plateau_len = _longest_plateau(graph, via_path, dist_s, dist_t, weight)
score = 2 * via_cost + shared_cost - plateau_len
admissible.append((score, via_path))
```

---

### Step 7: De-duplication and selection

```python
admissible.sort(key=lambda x: x[0])
selected = []
for _, via_path in admissible:
    if len(selected) >= 2: break
    # Check overlap with each already-selected path
    overlap_cost / via_cost > gamma  →  skip (too similar)
    selected.append(via_path)
```

The gamma threshold is applied again, this time comparing each candidate against the already-selected alternatives (not just against Opt). This prevents two very similar alternatives from both being returned.

---

### Step 8: Return

```python
paths = [opt_path] + selected
return pathsToCoordinates(graph, paths, weight)
```

`paths[0]` is always the optimal path. `paths[1]` and `paths[2]` are the alternatives if found.

---

## 8. Parameters: α, γ, ε

These three parameters control the quality-diversity trade-off and are passed from `main.py` via the `request` object. They are **no longer hardcoded** inside `_alternative_paths()`.

---

### ε — epsilon (stretch / UBS bound)

**Formula:** `l(P) ≤ (1 + ε) · l(Opt)`

Controls the maximum permitted length of any alternative path relative to the optimal.

| Value | Effect |
|---|---|
| `0.10–0.15` | Only very short detours allowed; often no alternatives found in dense cities |
| **`0.25` (walking default)** | Up to 25% longer — practical and consistent with paper experiments |
| **`0.35` (wheelchair default)** | Wider budget needed because accessible routes are fewer and inherently longer |
| `0.40+` | Many candidates admitted, but alternatives may feel impractically long |

`via_path_cutoff` is set to `1 + epsilon` by both caller functions, ensuring the Dijkstra search radius is exactly consistent with the UBS constraint — no node is explored that could never pass the UBS filter.

---

### γ — gamma (sharing bound)

**Formula:** `l(Opt ∩ P) ≤ γ · l(Opt)`

Controls how different an alternative must be from Opt (and from other selected alternatives).

| Value | Effect |
|---|---|
| `0.50` | Very different — alternatives must use almost entirely separate roads |
| **`0.65` (walking default)** | Good diversity — alternatives take genuinely distinct corridors |
| **`0.70` (wheelchair default)** | Slightly more permissive — accessible corridors are sparse |
| `0.80+` | Alternatives may look nearly identical to Opt |

Also used in Step 7 de-duplication: two alternatives that overlap by more than `gamma` of the shorter one's length are treated as duplicates.

---

### α — alpha (local-optimality radius)

**Formula:** `T = α · l(Opt)` — the length of sub-segments that must be locally optimal.

| Value | Effect |
|---|---|
| `0.10–0.15` | Only short sub-segments checked; allows alternatives with longer detour stretches |
| **`0.25` (walking default)** | Consistent with paper — any quarter-of-route sub-path must be optimal |
| **`0.20` (wheelchair default)** | Slightly looser — accessible paths may be locally sub-optimal around obstacles |
| `0.35+` | Very strict; many paths rejected; best for long-distance queries |

**Relationship with ε:** a larger `ε` (more stretch) should generally be paired with a smaller `α` (looser local-optimality) to keep the admissible candidate count reasonable.

---

### Defaults (set in `main.py`)

```python
# Walking
request.epsilon = 0.25
request.gamma   = 0.65
request.alpha   = 0.25

# Wheelchair
request.epsilon = 0.35
request.gamma   = 0.70
request.alpha   = 0.20
```

---

## 9. fast_mode: Speedup Heuristic

`fast_mode=False` by default — the full Abraham et al. algorithm runs unchanged. Setting `fast_mode=True` enables two complementary speedups implemented as an `if fast_mode:` branch in `_alternative_paths()`.

### Speedup 1: Spatial grid clustering

Controlled by `_GRID_STEP = 0.002` at module level.

```python
if fast_mode:
    candidates = _cluster_candidates_by_grid(graph, candidates_sorted, MAX_CANDIDATES)
else:
    candidates = candidates_sorted[:MAX_CANDIDATES]
```

Groups via-node candidates into ~200 m grid cells (using OSM `x`/`y` node attributes). Only the cheapest candidate per cell enters the inner loop. In a dense urban graph, this reduces 500 candidates to approximately 100 — an ~80% reduction in inner-loop iterations, and therefore an ~80% reduction in T-test calls.

### Speedup 2: T-test bypass

```python
if not fast_mode:
    if not _t_test(graph, path_sv, path_vt, via_node, T, weight):
        continue
```

`_t_test()` calls `nx.dijkstra_path_length()` once per candidate — the most expensive single operation in the inner loop. Skipping it entirely gives a further ~3–5× speedup.

**Quality trade-off:** UBS and sharing constraints still run in `fast_mode`. Some admitted paths may not be strictly T-locally-optimal, but Lemma 4.2 guarantees they are at most 2T-suboptimal. In practice, the quality difference is small.

### Empirical results (synthetic 12×12 grid graph, 30 queries)

| Mode | Avg runtime | Success rate |
|---|---|---|
| Normal (`fast_mode=False`) | 3.4 ms | ~70% |
| `fast_mode=True` | 0.9 ms | ~80% |
| **Speedup** | **~3.9×** | — |

> Results from `navigation_analysis.py`. `fast_mode` often achieves a *higher* success rate because it passes more candidates through to the final scoring step (the T-test rejection was eliminating some valid alternatives).

### How to expose in `main.py`

```python
def get_path(request, mode="walking", fast_mode=False):
    if mode == "wheelchair":
        return alternative_paths_by_wheelchair(request, fast_mode=fast_mode)
    return alternative_paths_by_walking(request, fast_mode=fast_mode)
```

---

## 10. Walking vs. Wheelchair Mode

The same `_alternative_paths()` function handles both modes. The only differences are in the callers:

| | Walking | Wheelchair |
|---|---|---|
| Caller | `alternative_paths_by_walking()` | `alternative_paths_by_wheelchair()` |
| Pre-processing | None | `updateGraphWeights(graph, request.distance_weight)` |
| Weight attribute | `"length"` | `"weight"` |
| Recommended ε | `0.25` | `0.35` |
| Recommended γ | `0.65` | `0.70` |
| Recommended α | `0.25` | `0.20` |

`updateGraphWeights()` mutates the graph's edge `"weight"` attributes before the algorithm runs. It penalises steep inclines, poor surface types, missing kerb cuts, and other barriers to wheelchair access. Because all cost comparisons (`l_opt`, `via_cost`, `shared_cost`) use the same `weight` attribute throughout, changing the attribute string is sufficient to switch modes — no algorithmic changes required.

---

## 11. Error Handling

### `constraint_fails` diagnostic counter

Inside `_alternative_paths()`, a dict tracks how candidates are eliminated:

```python
constraint_fails = {
    'path_reconstruction': 0,  # paths_s or paths_t returned None
    'path_validation':     0,  # _validate_path() or edge check failed
    'sharing':             0,  # constraint (a) rejected
    't_test':              0,  # constraint (b) rejected (normal mode only)
    'other':               0,
    'success':             0,  # passed all checks
}
```

---

---

## 12. Quick-Reference Table

| Symbol | Name | Role | Walking | Wheelchair |
|---|---|---|---|---|
| `ε` | epsilon | Max stretch: `l(P) ≤ (1+ε)·l(Opt)` | `0.25` | `0.35` |
| `γ` | gamma | Max sharing: `l(Opt∩P) ≤ γ·l(Opt)` | `0.65` | `0.70` |
| `α` | alpha | T-test radius: `T = α·l(Opt)` | `0.25` | `0.20` |
| `l_opt` | — | Cost of the globally optimal path | computed | computed |
| `via_cost` | — | `d_s(v) + d_t(v)` — cost of path through via-node `v` | computed | computed |
| `cutoff` | — | Dijkstra search radius = `(1+ε)·l_opt` | computed | computed |
| `T` | — | Local-optimality window = `α·l_opt` | computed | computed |
| `_GRID_STEP` | — | Grid cell size for clustering ≈ 200 m | `0.002°` | `0.002°` |
| `MAX_CANDIDATES` | — | Max candidates processed in inner loop | `500` | `500` |

---

*Reference: Abraham et al. (2013). Alternative routes in road networks. ACM Journal of Experimental Algorithmics, 18(1), Article 1.3. https://doi.org/10.1145/2444016.2444019*
