import os
import numpy as np
import osmnx as ox
import networkx as nx
import rasterio
from pyproj import Transformer
from pathlib import Path


# ── wheelchair-compatible OSM highway types ───────────────────────────────
WHEELCHAIR_OK = {
    "footway", "path", "pedestrian", "living_street",
    "residential", "service", "tertiary", "tertiary_link",
    "secondary", "secondary_link", "primary", "primary_link",
    "unclassified", "track", "cycleway", "corridor",
}

# ── highway types that must always be excluded ────────────────────────────
WHEELCHAIR_BLOCKED = {
    "motorway", "motorway_link", "trunk", "trunk_link",
    "steps",   # ← most critical: physically impassable for wheelchairs
    "raceway", "proposed", "construction",
}

# Maximum wheelchair slope (8 % = DIN 18040-3 practical field standard)
MAX_SLOPE = 0.08


class GraphHolder:
    _graph : nx.DiGraph = None
    _data_dir = Path(__file__).parent.parent / "Data"
    
    def __init__(self):
        pass
    
    @classmethod
    def get_graph(cls):
        error = FileNotFoundError("Graph is None. Check if the method '__prepareGraph' is implemented.")

        if GraphHolder._graph is not None:
            return GraphHolder._graph

        graphml_path = cls._data_dir / "Bonn_wheelchair.graphml"
        dem_path     = cls._data_dir / "dgm1_05314000_Bonn.tif"

        if graphml_path.exists():
            G = nx.read_graphml(str(graphml_path))

            # Ensure MultiDiGraph so G[u][v][key] access works everywhere.
            if not isinstance(G, nx.MultiDiGraph):
                G = nx.MultiDiGraph(G)

            G = nx.relabel_nodes(G, {n: int(n) for n in G.nodes()})

            # Cast numeric attributes back to float and restore None sentinel.
            # GraphML round-trips all values as strings; -9999.0 was written
            # in place of Python None (GraphML has no null type).
            SENTINEL = -9999.0

            for _, data in G.nodes(data=True):
                for attr in ("x", "y", "elevation"):
                    if attr in data:
                        try:
                            v = float(data[attr])
                            data[attr] = None if v == SENTINEL else v
                        except (ValueError, TypeError):
                            pass

            for _, _, data in G.edges(data=True):
                for attr in ("length", "slope", "weight"):
                    if attr in data:
                        try:
                            v = float(data[attr])
                            data[attr] = None if v == SENTINEL else v
                        except (ValueError, TypeError):
                            pass

            GraphHolder._graph = G
            return GraphHolder._graph

        else:
            GraphHolder.__checkDEMPath(str(dem_path))
            GraphHolder._graph = GraphHolder.__prepareGraph(
                cityName="Bonn,Germany", DEMPath=str(dem_path)
            )
            if GraphHolder._graph is not None:
                return GraphHolder._graph
            else:
                raise error

    @classmethod
    def __checkDEMPath(cls,path):
        error = FileNotFoundError("Digital Elevation Model (DEM) does not exists \n"
                                        "Is the DEM in the 'Data' folder? If not,"
                                        "\nyou can download the DEM data with link below."
                                        "https://databox.bonn.de/public/download-shares/nzdFB6N6O3wwB67sGXrBESdVlwF596Ia ")
        if os.path.exists(path):
            return path
        else:
            raise error

    @classmethod
    def __prepareGraph(cls,cityName,DEMPath):
        """
        Download the OSM graph for *cityName*, enrich every node with
        elevation from the DEM raster, compute slopes for every directed
        edge, filter impassable edges, and save the result as GraphML.

        Steps
        -----
        1. Download OSM graph with osmnx (network_type='walk', simplify=False)
        2. Filter out non-wheelchair highway types (steps, motorways …)
        3. Open the DEM .tif and sample elevation at every node coordinate
        4. Compute slope = Δelevation / length for every directed edge
        5. Remove edges steeper than MAX_SLOPE (8 %)
        6. Save the enriched graph as  <cityName>_wheelchair.graphml

        Parameters
        ----------
        cityName : str
            Place name understood by OSMnx, e.g. "Bonn, Germany".
        DEMPath  : str
            File-system path to the DEM GeoTIFF
            (e.g. the Bonn DGM from opendata.bonn.de).
        """

        output_path = str(cls._data_dir / f"{cityName.split(',')[0].strip()}_wheelchair.graphml")

        # ── STEP 1 — Download OSM graph ───────────────────────────────────
        print(f"[1/5] Downloading OSM walk graph for '{cityName}' …")

        # network_type='walk'  → osmnx pre-filters to pedestrian-accessible
        #                        edges, which already excludes most motorways.
        # simplify=False       → keeps every intermediate node so slope
        #                        calculations are accurate (no long shortcuts
        #                        that would average out steep sections).
        G = ox.graph_from_place(
            cityName,
            network_type="walk",
            simplify=False,
        )

        print(f"    Downloaded: {G.number_of_nodes():,} nodes, "
              f"{G.number_of_edges():,} edges")

        # ── STEP 2 — Remove non-wheelchair edges ─────────────────────────
        print("[2/5] Filtering non-wheelchair highway types …")

        edges_to_remove = []
        for u, v, key, data in G.edges(keys=True, data=True):
            hw = data.get("highway", "")

            # osmnx sometimes stores highway as a list when a way has
            # multiple tags (e.g. ["footway", "steps"]) — handle both cases
            if isinstance(hw, list):
                tags = [str(t).lower().strip() for t in hw]
            else:
                tags = [str(hw).lower().strip()]

            # Block if ANY tag is in the blocked set (steps take priority)
            if any(t in WHEELCHAIR_BLOCKED for t in tags):
                edges_to_remove.append((u, v, key))
                continue

            # Block if NONE of the tags is in the allowed set
            if not any(t in WHEELCHAIR_OK for t in tags):
                edges_to_remove.append((u, v, key))

        G.remove_edges_from(edges_to_remove)
        G.remove_nodes_from(list(nx.isolates(G)))

        print(f"    Removed {len(edges_to_remove):,} edges  |  "
              f"remaining: {G.number_of_nodes():,} nodes, "
              f"{G.number_of_edges():,} edges")

        # ── STEP 3 — Sample elevation from DEM ───────────────────────────
        print(f"[3/5] Sampling elevation from DEM: {DEMPath} …")

        with rasterio.open(DEMPath) as dem:
            dem_epsg = dem.crs.to_epsg() if dem.crs else None
            print(f"    DEM CRS: EPSG:{dem_epsg}")

            # osmnx node coordinates are always in EPSG:4326 (WGS84).
            # The Bonn DEM is typically in EPSG:25832 (UTM Zone 32N).
            # Build a transformer if the CRSs differ.
            if dem_epsg and dem_epsg != 4326:
                transformer = Transformer.from_crs(
                    "EPSG:4326",
                    f"EPSG:{dem_epsg}",
                    always_xy=True,   # always (lon, lat) input order
                )
            else:
                transformer = None

            # Collect (x_dem, y_dem) for every node in one list so we can
            # call dem.sample() once — much faster than one call per node.
            node_ids = list(G.nodes())
            coords_dem = []

            for nid in node_ids:
                node_data = G.nodes[nid]
                lon = float(node_data["x"])   # osmnx stores lon as 'x'
                lat = float(node_data["y"])   # osmnx stores lat as 'y'

                if transformer:
                    x_dem, y_dem = transformer.transform(lon, lat)
                else:
                    x_dem, y_dem = lon, lat

                coords_dem.append((x_dem, y_dem))

            # Batch sample — returns one array per pixel band per coordinate
            sampled = list(dem.sample(coords_dem))
            nodata  = dem.nodata
            valid   = 0

            for nid, pixel in zip(node_ids, sampled):
                val = float(pixel[0])   # band 1

                if nodata is not None and val == nodata:
                    G.nodes[nid]["elevation"] = None
                elif np.isnan(val) or val < -999:
                    G.nodes[nid]["elevation"] = None
                else:
                    G.nodes[nid]["elevation"] = round(val, 3)
                    valid += 1

        print(f"    Valid elevations: {valid:,} / {len(node_ids):,} nodes "
              f"({100 * valid / max(len(node_ids), 1):.1f} %)")

        # ── STEP 4 — Compute slope for every directed edge ────────────────
        print("[4/5] Computing slopes and removing steep edges …")

        edges_to_remove = []
        missing = 0

        for u, v, key, data in G.edges(keys=True, data=True):
            elev_u = G.nodes[u].get("elevation")
            elev_v = G.nodes[v].get("elevation")
            length = float(data.get("length", 0.0))

            # If elevation is unknown or edge has no length, default to flat
            if elev_u is None or elev_v is None or length <= 0:
                G[u][v][key]["slope"] = 0.0
                missing += 1
                continue

            # slope > 0 → uphill   (v is higher than u)
            # slope < 0 → downhill (v is lower  than u)
            slope = (elev_v - elev_u) / length
            G[u][v][key]["slope"] = round(slope, 6)

            # Remove edges that exceed the maximum wheelchair grade
            if abs(slope) > MAX_SLOPE:
                edges_to_remove.append((u, v, key))

        G.remove_edges_from(edges_to_remove)
        G.remove_nodes_from(list(nx.isolates(G)))

        print(f"    Removed {len(edges_to_remove):,} edges (slope > "
              f"{MAX_SLOPE * 100:.0f} %)  |  "
              f"{missing:,} edges had missing elevation (kept, slope=0)")
        print(f"    Final graph: {G.number_of_nodes():,} nodes, "
              f"{G.number_of_edges():,} edges")

        # ── STEP 5 — Save graph ───────────────────────────────────────────
        print(f"[5/5] Saving graph to '{output_path}' …")

        # GraphML cannot serialise Python lists or None.
        # Lists → string.  None → -9999.0 sentinel float so the attribute
        # stays numeric and is correctly cast back to None on reload.
        for _, data in G.nodes(data=True):
            for k, v in list(data.items()):
                if isinstance(v, list):
                    data[k] = str(v)
                elif v is None:
                    data[k] = -9999.0

        for _, _, data in G.edges(data=True):
            for k, v in list(data.items()):
                if isinstance(v, list):
                    data[k] = str(v)
                elif v is None:
                    data[k] = -9999.0

        nx.write_graphml(G, output_path)
        size_mb = os.path.getsize(output_path) / 1_048_576
        print(f"    Saved ({size_mb:.1f} MB) — ready for Task c / d.")

        # Store on the class so other methods can access it without reloading
        cls._graph = G
        return G


if __name__ == '__main__':
    GraphHolder.get_graph()