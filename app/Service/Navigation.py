import networkx as nx

from ..Holder.GraphHolder import GraphHolder
from .Util import *
import os

# ── Spatial-grid cell size for fast_mode candidate clustering ─────────────────
# ~200 m in latitude/longitude degrees at mid-European latitudes.
_GRID_STEP = 0.002

def _cluster_candidates_by_grid(graph, candidates_sorted, max_candidates):
    """
    Speedup heuristic: spatial grid pre-clustering.

    Groups via-node candidates into ~200 m grid cells (using OSM node lat/lon
    attributes 'y' and 'x').  Only the cheapest candidate per cell is kept.

    Two via-nodes in the same cell produce paths that diverge by at most the
    cell diameter before re-merging, making them effectively duplicates.
    Discarding the more expensive one avoids redundant _t_test() calls with
    negligible impact on path diversity.

    :param graph:             the road graph (nodes carry 'y'=lat, 'x'=lon)
    :param candidates_sorted: list of (via_node, via_cost) sorted by via_cost
    :param max_candidates:    hard cap applied after clustering
    :return:                  filtered and capped list of (via_node, via_cost)
    """
    seen_cells: dict = {}
    filtered = []

    for v, via_cost in candidates_sorted:
        if not _is_hashable(v):
            continue
        try:
            lat = graph.nodes[v].get("y", 0)
            lon = graph.nodes[v].get("x", 0)
        except (KeyError, TypeError):
            # Node has no coordinates — keep it without clustering
            filtered.append((v, via_cost))
            continue

        cell = (round(lat / _GRID_STEP), round(lon / _GRID_STEP))
        if cell not in seen_cells:
            seen_cells[cell] = True
            filtered.append((v, via_cost))

    print(
        f"DEBUG: Grid clustering: {len(candidates_sorted)} candidates → {len(filtered)} after clustering ({(1 - len(filtered) / max(len(candidates_sorted), 1)) * 100:.0f}%% reduction)"
    )
    return filtered[:max_candidates]


def _shortest_path(lat1, lon1, lat2, lon2, weight="length" ):
    # get Graph
    graph = GraphHolder.get_graph()

    try:
        # find closest nodes
        source = findClosestNode(lat1, lon1, graph)
        target = findClosestNode(lat2, lon2, graph)
        if source == target:
            raise Exception("Source and target are the same")
        
        # Check if both nodes exist in the graph
        if source not in graph or target not in graph:
            raise Exception(f"One or both nodes not found in graph: source={source}, target={target}")
        
        # Check if nodes are in the same connected component (for undirected view)
        if not nx.has_path(graph, source, target):
            raise Exception(f"No path exists between source and target. They may be in disconnected areas of the graph.")
        
        # find the shortest path
        path = nx.dijkstra_path(graph, source, target, weight=weight)

        return pathsToCoordinates(graph, [path],weight)
    except Exception as e:
        raise e


def _alternative_paths(lat1, lon1, lat2, lon2, weight, via_path_cutoff, epsilon, gamma, alpha, fast_mode=False):
    """
    Calculates the shortest path and choose at most 2 alternative paths to be displayed in the web-application.

    Implements the simplified Abraham et al. [2013] algorithm: two forward
    Dijkstra runs (one from source, one from target on the reversed graph)
    instead of a true bidirectional search.

    :param lat1, lon1, lat2, lon2: source and target coordinates
    :param weight: 'length' for walking, 'weight' for wheelchair
    :param via_path_cutoff: Dijkstra cutoff multiplier on l_opt.
        Set by callers to (1 + epsilon) so the search radius is consistent
        with the UBS constraint.
    :param epsilon: UBS stretch bound — alternative may cost at most
        (1 + epsilon) * l_opt.
    :param gamma: sharing bound — shared edge weight must be <= gamma * l_opt.
    :param alpha: local-optimality radius — T = alpha * l_opt.
    :param fast_mode: if True, enables two complementary speedups:
        (1) spatial grid clustering reduces candidates by ~80 % before the
            inner loop runs, and
        (2) the T-local-optimality test (_t_test) is skipped entirely.
        fast_mode=False (default) runs the full algorithm as per the paper.
    :return: list of coordinate lists [shortest_path, alt1?, alt2?]
    """

    graph = GraphHolder.get_graph()
    
    print(f"DEBUG: _alternative_paths called with lat1={lat1}, lon1={lon1}, lat2={lat2}, lon2={lon2}, weight={weight}, via_path_cutoff={via_path_cutoff}, fast_mode={fast_mode}, epsilon={epsilon}, gamma={gamma}, alpha={alpha}")

    try:

        # ── Step 1: find source and target nodes ──────────────────────────
        source = findClosestNode(lat1, lon1, graph)
        target = findClosestNode(lat2, lon2, graph)

        if source == target:
            raise Exception("Source and target are the same node.")
        if source not in graph or target not in graph:
            raise Exception(f"Node not found: source={source}, target={target}")
        if not nx.has_path(graph, source, target):
            raise Exception("No path exists between source and target.")

        # ── Step 2: compute optimal shortest path Opt ─────────────────────
        opt_path = nx.dijkstra_path(graph, source, target, weight=weight)
        
        # Validate the optimal path (including edge connectivity)
        if not _validate_path_edges(graph, opt_path):
            raise Exception(f"Invalid path returned from Dijkstra: path contains invalid edges")
        
        l_opt    = _path_cost(graph, opt_path, weight)
        
        print(f"DEBUG: Optimal path cost l_opt = {l_opt}, path length = {len(opt_path)} nodes")

        if l_opt == 0:
            return pathsToCoordinates(graph, [opt_path], weight)

        # ── Step 3: Dijkstra from source (forward) ────────────────────────
        # cutoff = via_path_cutoff * l_opt limits how far we search.
        # This prevents examining via vertices so far away that no
        # admissible alternative can result from them.
        cutoff = via_path_cutoff * l_opt
        
        print(f"DEBUG: cutoff = {via_path_cutoff} * {l_opt} = {cutoff}")

        dist_s, paths_s = nx.single_source_dijkstra(
            graph, source, cutoff=cutoff, weight=weight
        )

        # ── Step 4: Dijkstra from target on reversed graph (backward) ─────
        # Reversing the graph turns "distance from target" into a standard
        # single-source query, equivalent to the backward search in BD.
        graph_rev      = graph.reverse(copy=True)  # Use copy=True for safer operations
        dist_t, paths_t = nx.single_source_dijkstra(
            graph_rev, target, cutoff=cutoff, weight=weight
        )
        
        print(f"DEBUG: Forward search: {len(dist_s)} nodes reachable from source")
        print(f"DEBUG: Backward search: {len(dist_t)} nodes reachable from target")
        print(f"DEBUG: target in paths_t: {target in paths_t}, paths_t.get(target)={paths_t.get(target, 'NOT FOUND')}")

        # ── Step 5: collect via-vertex candidates ─────────────────────────
        # A valid via vertex must be reachable from both s and t within the
        # cutoff, and the combined via-path must satisfy the UBS constraint.
        opt_nodes = _nodes_to_set(opt_path)  # fast membership test
        if opt_nodes is None:
            raise Exception("Could not create node set from optimal path: path contains unhashable nodes")

        # Pre-compute the set of (u,v) edges on Opt for sharing calculation
        opt_edges = _path_edges_to_set(opt_path)
        if opt_edges is None:
            raise Exception("Could not create edge set from optimal path")


        candidates = []
        sample_v_for_debug = None
        for v in dist_s:
            # Validate that v is hashable
            if not _is_hashable(v):
                continue
            if v not in dist_t:
                continue            # not reachable from target side
            if v == source or v == target:
                continue            # trivial paths not useful

            via_cost = dist_s[v] + dist_t[v]

            # ── Constraint (c): UBS / stretch ─────────────────────────────
            # l(Pv) ≤ (1 + ε) · l(Opt)
            if via_cost > (1 + epsilon) * l_opt:
                continue

            candidates.append((v, via_cost))
        
        print(f"DEBUG: Found {len(candidates)} candidate via-nodes")

        if not candidates:
            # No candidates at all — return only the shortest path
            print(f"DEBUG: No candidates found, returning only shortest path")
            return pathsToCoordinates(graph, [opt_path], weight)

        # ── Step 6: sort candidates by cost ──────────────────────────────
        # Sort candidates by via_cost. We process ALL candidates to maximize
        # chances of finding valid alternative paths. The constraint checks
        # will filter out invalid paths efficiently.
        MAX_CANDIDATES = 500

        candidates_sorted = sorted(
            candidates,
            key=lambda x: x[1]
        )

        if fast_mode:
            # ── Speedup (1): spatial grid clustering ──────────────────────
            # Keep only the cheapest candidate per ~200 m grid cell.
            # Nodes in the same cell yield near-identical via-paths; the
            # more expensive ones are safely discarded.
            candidates = _cluster_candidates_by_grid(
                graph, candidates_sorted, MAX_CANDIDATES
            )
        else:
            candidates = candidates_sorted[:MAX_CANDIDATES]
        
        print(f"DEBUG: Processing {len(candidates)} candidate with fast_mode={fast_mode} via-nodes (sorted by via_cost)")
        T = alpha * l_opt   # local-optimality threshold

        admissible = []
        constraint_fails = {'path_reconstruction': 0, 'path_validation': 0, 'sharing': 0, 't_test': 0, 'other': 0, 'success': 0}

        for via_node, via_cost in candidates:
            # NOTE: outer loop uses 'via_node' — never 'u' or 'v' — to
            # prevent any inner variable from shadowing the via-vertex.

            # ── Reconstruct s → via_node ──────────────────────────────────
            path_sv = paths_s.get(via_node)

            if path_sv is None:
                constraint_fails['path_reconstruction'] += 1
                continue

            # ── Reconstruct via_node → t ──────────────────────────────────
            # paths_t contains:
            # [target, ..., via_node]
            # because it was computed on graph_rev.
            path_vt_temp = paths_t.get(via_node)

            if path_vt_temp is None:
                constraint_fails['path_reconstruction'] += 1
                continue

            # Convert to:
            # [via_node, ..., target]
            path_vt = list(reversed(path_vt_temp))

            # ── Basic structure check ─────────────────────────────────────
            if not _validate_path(path_sv) or not _validate_path(path_vt):
                constraint_fails['path_validation'] += 1
                continue

            # ── Edge existence check (forward graph) ──────────────────────
            # Use loop variables 'pa', 'pb' — never 'u','v' — to avoid
            # any risk of shadowing via_node or the outer candidates loop.
            sv_ok = all(graph.has_edge(pa, pb)
                        for pa, pb in zip(path_sv[:-1], path_sv[1:]))
            vt_ok = all(graph.has_edge(pa, pb)
                        for pa, pb in zip(path_vt[:-1], path_vt[1:]))
            if not sv_ok:
                print(f"BAD SV PATH via={via_node}")
                print(path_sv[:10])

            if not vt_ok:
                print(f"BAD VT PATH via={via_node}")
                print(path_vt[:10])

            # ── Combine into full via-path ────────────────────────────────
            via_path = path_sv + path_vt[1:]  # via_node appears once

            # ── Constraint (a): sharing ────────────────────────────────────
            # Compute l(Opt ∩ Pv): sum of weights of edges shared with Opt
            via_edges = _path_edges_to_set(via_path)
            if via_edges is None:
                constraint_fails['path_validation'] += 1
                continue
            
            shared_edges = opt_edges & via_edges
            shared_cost  = _sum_edge_weights(graph, shared_edges, weight)
            if shared_cost > gamma * l_opt:
                constraint_fails['sharing'] += 1
                continue    # too much overlap with Opt

            # ── Constraint (b): T-test — ALWAYS run (paper Lemma 4.1) ────
            # In fast_mode this test is skipped to reduce runtime.
            # The UBS and sharing constraints still hold; some admitted paths
            # may not be strictly T-locally-optimal (Lemma 4.2 tolerance).
            if not fast_mode:
                if not _t_test(graph, path_sv, path_vt, via_node, T, weight):
                    constraint_fails['t_test'] += 1
                    continue

            # ── Plateau length + objective f(v) (paper Section 6) ────────
            plateau_len = _longest_plateau(
                graph, via_path, dist_s, dist_t, weight
            )
            score = 2 * via_cost + shared_cost - plateau_len
            admissible.append((score, via_path))
            constraint_fails['success'] += 1
        
        print(f"DEBUG: Constraint failures: {constraint_fails}")
        print(f"DEBUG: Admissible paths after constraints: {len(admissible)}")

        # ── Step 7: rank and pick top 2 ───────────────────────────────────
        admissible.sort(key=lambda x: x[0])

        # De-duplicate: drop any alternative that shares > gamma with a
        # previously selected alternative (not just with Opt)
        selected = []
        for _, via_path in admissible:
            if len(selected) >= 2:
                break
            
            # Validate the path is a proper list (basic check)
            if not _validate_path(via_path):
                continue
            
            via_edges_set = _path_edges_to_set(via_path)
            if via_edges_set is None:
                continue
            
            # Check overlap with every already-selected path
            too_similar = False
            for prev_path in selected:
                if not _validate_path(prev_path):
                    too_similar = True
                    break
                
                prev_edges = _path_edges_to_set(prev_path)
                if prev_edges is None:
                    too_similar = True
                    break
                
                overlap_cost = _sum_edge_weights(graph, via_edges_set & prev_edges, weight)
                via_cost_check = _path_cost(graph, via_path, weight)
                if via_cost_check > 0 and overlap_cost / via_cost_check > gamma:
                    too_similar = True
                    break
            if not too_similar:
                selected.append(via_path)
        
        print(f"DEBUG: Selected {len(selected)} alternative paths")

        # ── Step 8: return Opt + up to 2 alternatives ─────────────────────
        paths = [opt_path] + selected
        print(f"DEBUG: Returning {len(paths)} total paths (1 optimal + {len(selected)} alternatives)")
        return pathsToCoordinates(graph, paths, weight)

    except Exception as e:
        print(f"ERROR: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        return []


# ─────────────────────────────────────────────────────────────────────────────
# Helper functions
# ─────────────────────────────────────────────────────────────────────────────

def _is_hashable(obj):
    """Check if an object is hashable (can be used in sets/as dict keys)."""
    try:
        hash(obj)
        return True
    except TypeError:
        return False


def _validate_path(path):
    """
    Validate that all nodes in the path are hashable (can be used in sets/dicts).
    Returns True if valid, False otherwise.
    """
    if not path or not isinstance(path, list):
        return False
    
    for node in path:
        # Check that node is not a list or other unhashable type
        if isinstance(node, (list, dict, set)):
            return False
        if not _is_hashable(node):
            return False
    
    return True


def _validate_path_edges(graph, path):
    """
    Validate that all consecutive nodes in the path are connected by edges in the graph.
    Returns True if all edges exist, False otherwise.
    Also returns False if the path is invalid.
    """
    if not _validate_path(path):
        return False
    
    # Check that all consecutive pairs of nodes have edges between them
    for u, v in zip(path[:-1], path[1:]):
        if not graph.has_edge(u, v):
            return False
    
    return True


def _path_edges_to_set(path):
    """
    Safely convert a path (list of nodes) to a set of edges (tuples).
    Returns None if the path is invalid.
    """
    if not _validate_path(path):
        return None
    
    try:
        edges = set(zip(path[:-1], path[1:]))
        return edges
    except TypeError:
        # Fallback: this shouldn't happen if _validate_path passed, but just in case
        return None


def _nodes_to_set(path):
    """
    Safely convert a path (list of nodes) to a set of nodes.
    Returns None if any nodes are unhashable.
    """
    if not _validate_path(path):
        return None
    
    try:
        return set(path)
    except TypeError:
        return None

def _path_cost(graph, path, weight):
    """Sum of edge weights along a node-list path."""
    return sum(
        _get_edge_weight(graph, u, v, weight)
        for u, v in zip(path[:-1], path[1:])
    )


def _get_edge_weight(graph, u, v, weight):
    """
    Safely get edge weight from a graph (handles both DiGraph and MultiDiGraph).
    
    For MultiDiGraph: returns the minimum weight among all edges from u to v.
    For DiGraph: returns the single edge weight.
    
    Returns 0 if edge doesn't exist.
    """
    try:
        if not graph.has_edge(u, v):
            return 0
        
        # Check if it's a MultiDiGraph
        if isinstance(graph, nx.MultiDiGraph):
            # MultiDiGraph: graph[u][v] is {key: {attr dict}, ...}
            edge_dict = graph[u][v]
            if not edge_dict:
                return 0
            # Get minimum weight from all edges
            weights = []
            for key, attrs in edge_dict.items():
                w = attrs.get(weight, 0)
                if isinstance(w, (int, float)):
                    weights.append(w)
            return min(weights) if weights else 0
        else:
            # Regular DiGraph: graph[u][v] is {attr dict}
            return graph[u][v].get(weight, 0)
    except (KeyError, TypeError, AttributeError):
        return 0


def _sum_edge_weights(graph, edges, weight):
    """
    Sum weights of all edges in a set of (u, v) tuples.
    Safely handles both DiGraph and MultiDiGraph.
    """
    total = 0
    for u, v in edges:
        total += _get_edge_weight(graph, u, v, weight)
    return total


def _t_test(graph, path_sv, path_vt, v, T, weight):
    """
    Approximate T-test from Section 4 of Abraham et al. [2013].

    Find:
      x — the node on path_sv that is closest to v but at least T away
          (walking backwards from v toward s). If the entire s→v path
          is shorter than T, x = s.
      y — the symmetric node on path_vt (walking forward from v toward t).
          If the entire v→t path is shorter than T, y = t.

    The test passes if the cost of Pv between x and y equals dist(x, y),
    i.e. that sub-path is already a shortest path.

    Lemma 4.1: if the test passes → Pv is T-locally-optimal.
    Lemma 4.2: if it fails → Pv is not 2T-LO (we correctly reject it).
    """
    # Walk path_sv backwards from v to find x
    cost = 0.0
    x = path_sv[0]  # default: s
    for i in range(len(path_sv) - 1, 0, -1):
        u_node = path_sv[i - 1]
        v_node = path_sv[i]
        edge_w = _get_edge_weight(graph, u_node, v_node, weight)
        cost  += edge_w
        if cost >= T:
            x = u_node
            break

    # Walk path_vt forward from v to find y
    cost = 0.0
    y = path_vt[-1]  # default: t
    for i in range(len(path_vt) - 1):
        u_node = path_vt[i]
        v_node = path_vt[i + 1]
        edge_w = _get_edge_weight(graph, u_node, v_node, weight)
        cost  += edge_w
        if cost >= T:
            y = v_node
            break

    if x == y:
        return True  # degenerate: trivially passes

    # Extract the x→y sub-path from the combined via-path
    combined = path_sv + path_vt[1:]
    try:
        xi = combined.index(x)
        yi = combined.index(y)
    except ValueError:
        return False

    subpath      = combined[xi: yi + 1]
    subpath_cost = _path_cost(graph, subpath, weight)

    # Query true shortest-path distance x→y
    try:
        dist_xy = nx.dijkstra_path_length(graph, x, y, weight=weight)
    except nx.NetworkXNoPath:
        return False

    # Pass if the via sub-path is already optimal (within float tolerance)
    return subpath_cost <= dist_xy * 1.001


def _longest_plateau(graph, via_path, dist_s, dist_t, weight):
    """
    Compute the length of the longest plateau in via_path.

    A plateau (Lemma 4.4) is a maximal contiguous sub-path of Pv where
    every node v satisfies:
        dist_s[v] + dist_t[v]  ≈  dist_s[endpoint] + dist_t[endpoint]
    i.e. the node lies on BOTH the shortest-path tree from s and the
    reverse shortest-path tree from t simultaneously.

    By Lemma 4.4, a plateau of length L guarantees L-local-optimality.
    We use the plateau length in the ranking objective f(v).
    """
    if len(via_path) < 2:
        return 0.0

    best   = 0.0
    curr   = 0.0
    in_plt = False

    for i in range(len(via_path) - 1):
        u = via_path[i]
        v = via_path[i + 1]

        # Validate nodes are hashable before checking membership
        if not _is_hashable(u) or not _is_hashable(v):
            # If nodes are unhashable, can't check membership in dist dicts
            on_plt = False
        else:
            # A node is "on plateau" if it appears in both search trees
            on_plt = (u in dist_s) and (u in dist_t) and \
                     (v in dist_s) and (v in dist_t)

        edge_w = _get_edge_weight(graph, u, v, weight)

        if on_plt:
            curr  += edge_w
            in_plt = True
        else:
            if in_plt:
                best = max(best, curr)
            curr   = 0.0
            in_plt = False

    if in_plt:
        best = max(best, curr)

    return best


def shortest_path_by_walking(request):
    """Calculate the shortest walking path and return it."""
    try:
        return _shortest_path(request.lat1, request.lon1, request.lat2, request.lon2)
    except nx.NetworkXNoPath as e:
        raise Exception(f"No walking path found between the two locations.")
    except Exception as e:
        raise e


def shortest_path_by_wheelchair(request):
    """Calculate the shortest path for wheelchair movement and return it."""
    try:
        graph = GraphHolder.get_graph()
        updateGraphWeights(graph, request.distance_weight)
        return _shortest_path(request.lat1, request.lon1, request.lat2, request.lon2, weight="weight")
    except Exception as e:
        raise e


def alternative_paths_by_walking(request, via_path_cutoff = 1.4, fast_mode=False):
    """Calculate alternative walking paths and return them."""
    try:
        via_path_cutoff = 1 + request.epsilon # (1 + ϵ)-UBS (uniformly bounded stretch)
        return _alternative_paths(request.lat1, request.lon1, request.lat2, request.lon2, "length", via_path_cutoff, epsilon=request.epsilon, gamma=request.gamma, alpha=request.alpha, fast_mode=fast_mode)
    except Exception as e:
        raise e


def alternative_paths_by_wheelchair(request, via_path_cutoff = 1.4, fast_mode=False):
    """Calculate alternative paths for wheelchair movement and return them."""
    try:
        via_path_cutoff = 1 + request.epsilon # (1 + ϵ)-UBS (uniformly bounded stretch)
        graph = GraphHolder.get_graph()
        updateGraphWeights(graph, request.distance_weight)
        return _alternative_paths(request.lat1, request.lon1, request.lat2, request.lon2, "weight", via_path_cutoff, epsilon=request.epsilon, gamma=request.gamma, alpha=request.alpha, fast_mode=fast_mode)
    except Exception as e:
        raise e


if __name__ == "__main__":
    os.chdir("../")