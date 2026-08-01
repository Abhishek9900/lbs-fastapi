import osmnx as ox
from pyproj import Transformer


latlontoxy = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)


def findClosestNode(y, x, G  , transformer = None):
    """
    Find closest node on graph given the map coordinates.

    TODO: If you implemented your own graph (not from OSMNX), add the CRS information to the graph.

    :param y: latitude
    :param x: longitude
    :param G: Graph : networkx graph
    :param transformer: Transformer : pyproj.Transformer
    :return: Int : node id
    """
    if transformer:
        x, y = transformer.transform(x, y)

    return ox.distance.nearest_nodes(G, x, y)


def updateGraphWeights(G, distance_weight: float = 0.5):
    """
    Update the graph weights for each edge to enable wheelchair navigation.
 
    :param G:               networkx graph
    :param distance_weight: float in [0.0, 1.0] from the frontend slider.
                            Controls the blend between two objectives:
 
                              0.0 → weight = slope_time only
                                    (minimise effort / avoid hills entirely)
                              1.0 → weight = length only
                                    (minimise distance, ignore slope)
                              0.5 → equal blend of both (recommended default)
 
    What this function does
    -----------------------
    1. Removes any remaining 'steps' edges — steps must never appear in a
       wheelchair route regardless of the distance_weight value.
    2. For every directed edge (u → v) computes two component scores:
         • slope_time  — travel-time in seconds penalised by uphill slope
         • length      — raw edge length in metres
    3. Blends them using distance_weight:
         weight = distance_weight       * length_normalised
                + (1 - distance_weight) * slope_time_normalised
       Both components are normalised by typical city-scale values so the
       slider has a balanced, predictable effect across the whole network.
    4. All weights are strictly positive (negative weights break Dijkstra).
    5. Both directed edge directions are covered automatically because osmnx
       stores (u→v) and (v→u) as separate entries with opposite slope signs.
 
    Normalisation constants
    -----------------------
    NORM_LENGTH    = 100  m   (typical short urban edge length)
    NORM_TIME      = 100  s   (100 m at 1 m/s flat speed = 100 s)
    These make both components roughly order-1 for typical Bonn streets so
    the blend slider has equal influence regardless of which side it is on.
 
    Slope formula
    -------------
    base_time      = length / FLAT_SPEED_MS
    factor_uphill  = 1 + UPHILL_PENALTY * slope          if slope >= 0
    factor_downhill= max(DOWNHILL_MIN, 1 + slope)        if slope <  0
    slope_time     = base_time * factor
    """
 
    # ── Tuning constants ──────────────────────────────────────────────────
    FLAT_SPEED_MS    = 1.0    # m/s  (~3.6 km/h, typical manual wheelchair)
    UPHILL_PENALTY   = 2.0    # extra time per unit of uphill slope
    DOWNHILL_MIN     = 0.5    # minimum downhill factor (keeps weight > 0)
    NORM_LENGTH      = 100.0  # metres  — normalisation divisor for length
    NORM_TIME        = 100.0  # seconds — normalisation divisor for slope_time
 
    # Clamp distance_weight to [0, 1] defensively
    dw = max(0.0, min(1.0, float(distance_weight)))
    slope_weight = 1.0 - dw
 
    # ── Step 1 — Remove any surviving steps edges ─────────────────────────
    steps_to_remove = [
        (u, v, key)
        for u, v, key, data in G.edges(keys=True, data=True)
        if _is_steps(data.get("highway", ""))
    ]
    G.remove_edges_from(steps_to_remove)
    if steps_to_remove:
        print(f"[updateGraphWeights] Removed {len(steps_to_remove)} steps edges.")
 
    # ── Step 2 — Compute blended weight for every directed edge ───────────
    for u, v, key, data in G.edges(keys=True, data=True):
 
        length = float(data.get("length", 1.0))   # metres
        slope  = float(data.get("slope",  0.0))   # fraction: +0.08 = 8% uphill
 
        # ── Slope-time component ──────────────────────────────────────────
        base_time = length / FLAT_SPEED_MS         # seconds on flat ground
 
        if slope >= 0:
            # Uphill or flat — linearly more costly
            # e.g. slope=0.08 → factor=1.16  (16% slower)
            factor = 1.0 + UPHILL_PENALTY * slope
        else:
            # Downhill — slight saving, clamped to stay positive
            # e.g. slope=-0.05 → factor=max(0.5, 0.95)=0.95
            factor = max(DOWNHILL_MIN, 1.0 + slope)
 
        slope_time = base_time * factor            # always > 0
 
        # ── Blend the two components ──────────────────────────────────────
        #
        #   weight = dw          * (length     / NORM_LENGTH)
        #          + slope_weight * (slope_time / NORM_TIME)
        #
        # Dividing by normalisation constants makes both terms dimensionless
        # and roughly order-1 for typical urban edges, so the slider has a
        # balanced, intuitive effect:
        #   dw=1.0  → Dijkstra finds the shortest distance path
        #   dw=0.0  → Dijkstra finds the least-effort (flattest) path
        #   dw=0.5  → balanced trade-off
        #
        weight = (dw * (length / NORM_LENGTH)
                  + slope_weight * (slope_time / NORM_TIME))
 
        # Guard: weight must be strictly positive for Dijkstra
        G[u][v][key]["weight"] = max(weight, 1e-6)
 
    return G
 
 
# ── Internal helper ────────────────────────────────────────────────────────
 
def _is_steps(highway_value) -> bool:
    """Return True if the highway tag indicates steps."""
    if isinstance(highway_value, list):
        return any(str(t).lower().strip() == "steps" for t in highway_value)
    return str(highway_value).lower().strip() == "steps"


def pathsToCoordinates(G, paths, weight):
    """
    :param G: networkx graph
    :param paths: List : List of paths
    :param weight: weight key to use (e.g., 'weight', 'length')
    :return: List : List of dictionaries [{"path": List of tuples, "length": str}]

    Note: That function is used to convert the paths to coordinates to be used in the frontend.
    Depending on your 'paths' list, you can modify this function to fit your needs.
    But the return format must be same because of frontend.
    """
    returnList = []
    for path in paths:
        # Validate that all edges in the path exist in the graph
        valid_path = True
        for u, v in zip(path[:-1], path[1:]):
            if not G.has_edge(u, v):
                print(f"WARNING: Edge ({u}, {v}) does not exist in graph, skipping path")
                valid_path = False
                break
        
        if not valid_path:
            continue
        
        try:
            totalWeight = sum(G[u][v][0][weight] for u, v in zip(path[:-1], path[1:]))
            totalLength = sum(G[u][v][0]['length'] for u, v in zip(path[:-1], path[1:]))

            pathCoordinates = []
            for node in path:
                x, y = latlontoxy.transform(G.nodes[node]['x'], G.nodes[node]['y'])
                pathCoordinates.append((x, y))

            returnList.append({"path": pathCoordinates, "length": f'length: {round(totalLength)} m, weight: {round(totalWeight)}'})
        except (KeyError, StopIteration) as e:
            print(f"ERROR in pathsToCoordinates: {e}, skipping path")
            continue
    
    return returnList
