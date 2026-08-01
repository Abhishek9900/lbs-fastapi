"""
navigation_analysis.py
======================
Standalone analysis and benchmarking tool for the alternative-path algorithm.

This script does NOT require the full application stack (no GraphHolder, no OSM
data). It builds small synthetic road-like graphs and runs the core algorithm
logic to produce:

  1. Parameter sweep  — how epsilon/gamma/alpha affect success rate and path
                        quality on a 50-node grid graph.
  2. Speedup analysis — wall-clock comparison of fast_mode vs. normal mode.
  3. Candidate funnel — shows how many candidates survive each constraint at
                        each filter stage (UBS → sharing → T-test → final).
  4. Scaling chart    — how runtime grows with graph size (N nodes).

Output: 4 PNG charts saved to the current directory.

Usage:
    python navigation_analysis.py

Requirements:
    pip install networkx matplotlib numpy
"""

import time
import random
import math
import itertools
import collections

import networkx as nx
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── colour palette consistent with the presentation ──────────────────────────
C_NAVY   = "#0D1B3E"
C_TEAL   = "#028090"
C_MINT   = "#02C39A"
C_GOLD   = "#F4A261"
C_RED    = "#E63946"
C_LGREY  = "#B0C4CE"
C_WHITE  = "#FFFFFF"
C_OFFWH  = "#E8F4F8"

plt.rcParams.update({
    "figure.facecolor": C_WHITE,
    "axes.facecolor":   C_OFFWH,
    "axes.edgecolor":   C_LGREY,
    "axes.labelcolor":  C_NAVY,
    "xtick.color":      C_NAVY,
    "ytick.color":      C_NAVY,
    "text.color":       C_NAVY,
    "grid.color":       C_WHITE,
    "grid.linewidth":   0.8,
    "font.family":      "DejaVu Sans",
    "font.size":        10,
})


# ── synthetic graph builders ──────────────────────────────────────────────────

def build_grid_graph(rows: int, cols: int, noise: float = 0.3) -> nx.DiGraph:
    """
    Build a directed grid graph with noisy edge weights that simulate a road
    network (slightly irregular distances, occasional shortcuts).
    """
    G = nx.DiGraph()
    for r in range(rows):
        for c in range(cols):
            node = r * cols + c
            lat = r * 0.01
            lon = c * 0.01
            G.add_node(node, y=lat, x=lon)

    rng = random.Random(42)
    for r in range(rows):
        for c in range(cols):
            node = r * cols + c
            for dr, dc in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    nbr = nr * cols + nc
                    base = 100.0  # metres per grid step
                    w = base * (1.0 + rng.uniform(-noise, noise))
                    G.add_edge(node, nbr, length=round(w, 1), weight=round(w * 1.2, 1))
            # add occasional diagonal shortcut
            if rng.random() < 0.08:
                for dr, dc in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        nbr = nr * cols + nc
                        w = 100.0 * math.sqrt(2) * (1.0 + rng.uniform(-0.1, 0.1))
                        G.add_edge(node, nbr, length=round(w, 1), weight=round(w * 1.1, 1))
    return G


# ── core algorithm (self-contained, no GraphHolder) ───────────────────────────

def path_cost(G, path, weight="length"):
    return sum(
        min(G[u][v].get(weight, 0) for v in G[u] if v == nxt)
        for u, nxt in zip(path[:-1], path[1:])
        if G.has_edge(u, nxt)
    )

def get_edge_w(G, u, v, weight):
    if not G.has_edge(u, v):
        return 0
    if isinstance(G, nx.MultiDiGraph):
        return min(d.get(weight, 0) for d in G[u][v].values())
    return G[u][v].get(weight, 0)

def sum_edge_weights(G, edges, weight):
    return sum(get_edge_w(G, u, v, weight) for u, v in edges)

def path_edges(path):
    return set(zip(path[:-1], path[1:]))

def t_test(G, path_sv, path_vt, T, weight):
    """Simplified T-test: check that the x→y sub-path cost == dist(x,y)."""
    if not path_sv or not path_vt:
        return False
    cost = 0.0
    x = path_sv[0]
    for i in range(len(path_sv) - 1, 0, -1):
        cost += get_edge_w(G, path_sv[i - 1], path_sv[i], weight)
        if cost >= T:
            x = path_sv[i - 1]
            break
    cost = 0.0
    y = path_vt[-1]
    for i in range(len(path_vt) - 1):
        cost += get_edge_w(G, path_vt[i], path_vt[i + 1], weight)
        if cost >= T:
            y = path_vt[i + 1]
            break
    if x == y:
        return True
    combined = path_sv + path_vt[1:]
    try:
        xi, yi = combined.index(x), combined.index(y)
    except ValueError:
        return False
    sub_cost = path_cost(G, combined[xi:yi + 1], weight)
    try:
        d_xy = nx.dijkstra_path_length(G, x, y, weight=weight)
    except nx.NetworkXNoPath:
        return False
    return sub_cost <= d_xy * 1.001


def run_alternative_paths(G, source, target, weight="length",
                           epsilon=0.25, gamma=0.65, alpha=0.25,
                           fast_mode=False):
    """
    Run the alternative-path algorithm on graph G.
    Returns (n_alternatives, funnel_counts, elapsed_ms).
    funnel_counts = dict with keys: candidates, ubs_pass, sharing_pass, ttest_pass
    """
    t0 = time.perf_counter()

    funnel = dict(candidates=0, ubs_pass=0, sharing_pass=0, ttest_pass=0)

    try:
        opt_path = nx.dijkstra_path(G, source, target, weight=weight)
    except nx.NetworkXNoPath:
        return 0, funnel, 0.0

    l_opt = path_cost(G, opt_path, weight)
    if l_opt == 0:
        return 0, funnel, 0.0

    cutoff = (1 + epsilon) * l_opt
    opt_edges = path_edges(opt_path)

    dist_s, paths_s = nx.single_source_dijkstra(G, source, cutoff=cutoff, weight=weight)
    G_rev = G.reverse(copy=False)
    dist_t, paths_t = nx.single_source_dijkstra(G_rev, target, cutoff=cutoff, weight=weight)

    candidates = []
    for v in dist_s:
        if v not in dist_t or v == source or v == target:
            continue
        via_cost = dist_s[v] + dist_t[v]
        funnel["candidates"] += 1
        if via_cost <= (1 + epsilon) * l_opt:
            funnel["ubs_pass"] += 1
            candidates.append((v, via_cost))

    if fast_mode:
        # spatial grid clustering
        seen_cells: dict = {}
        filtered = []
        grid_step = 0.002
        for v, vc in sorted(candidates, key=lambda x: x[1]):
            lat = G.nodes[v].get("y", 0)
            lon = G.nodes[v].get("x", 0)
            cell = (round(lat / grid_step), round(lon / grid_step))
            if cell not in seen_cells:
                seen_cells[cell] = True
                filtered.append((v, vc))
        candidates = filtered[:500]
    else:
        candidates = sorted(candidates, key=lambda x: x[1])[:500]

    T = alpha * l_opt
    admissible = []

    for via_node, via_cost in candidates:
        path_sv = paths_s.get(via_node)
        path_vt_tmp = paths_t.get(via_node)
        if path_sv is None or path_vt_tmp is None:
            continue
        path_vt = list(reversed(path_vt_tmp))

        via_path = path_sv + path_vt[1:]
        via_e = path_edges(via_path)
        shared_cost = sum_edge_weights(G, opt_edges & via_e, weight)
        if shared_cost > gamma * l_opt:
            continue
        funnel["sharing_pass"] += 1

        if not fast_mode:
            if not t_test(G, path_sv, path_vt, T, weight):
                continue
        funnel["ttest_pass"] += 1

        score = 2 * via_cost + shared_cost
        admissible.append((score, via_path))

    admissible.sort(key=lambda x: x[0])
    selected = []
    for _, vp in admissible:
        if len(selected) >= 2:
            break
        ve = path_edges(vp)
        ok = True
        for prev in selected:
            ov = sum_edge_weights(G, ve & path_edges(prev), weight)
            vc = path_cost(G, vp, weight)
            if vc > 0 and ov / vc > gamma:
                ok = False
                break
        if ok:
            selected.append(vp)

    elapsed = (time.perf_counter() - t0) * 1000
    return len(selected), funnel, elapsed


# ── Chart 1: Parameter sweep ──────────────────────────────────────────────────

def chart_parameter_sweep():
    print("Chart 1: Parameter sweep …")
    G = build_grid_graph(10, 10)
    nodes = list(G.nodes())
    rng = random.Random(0)

    pairs = []
    while len(pairs) < 40:
        s, t = rng.sample(nodes, 2)
        if nx.has_path(G, s, t):
            pairs.append((s, t))

    eps_vals = [0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40]
    gamma_vals = [0.50, 0.60, 0.65, 0.70, 0.80]

    # success rate vs epsilon
    sr_eps = []
    for eps in eps_vals:
        found = sum(1 for s, t in pairs if run_alternative_paths(G, s, t, epsilon=eps)[0] >= 1)
        sr_eps.append(found / len(pairs) * 100)

    # success rate vs gamma
    sr_gamma = []
    for gam in gamma_vals:
        found = sum(1 for s, t in pairs if run_alternative_paths(G, s, t, gamma=gam)[0] >= 1)
        sr_gamma.append(found / len(pairs) * 100)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    fig.suptitle("Parameter sweep — success rate on 10×10 grid graph (40 queries)", fontsize=12, color=C_NAVY, fontweight="bold")

    ax1.plot([e * 100 for e in eps_vals], sr_eps, color=C_TEAL, marker="o", linewidth=2, markersize=6)
    ax1.set_xlabel("ε (epsilon) — stretch tolerance [%]")
    ax1.set_ylabel("Success rate [%] (≥1 alternative found)")
    ax1.set_title("Effect of ε", fontsize=10)
    ax1.axvline(25, color=C_GOLD, linestyle="--", linewidth=1, label="default ε=0.25")
    ax1.legend(fontsize=9)
    ax1.set_ylim(0, 105)
    ax1.grid(True)

    ax2.plot([g * 100 for g in gamma_vals], sr_gamma, color=C_MINT, marker="s", linewidth=2, markersize=6)
    ax2.set_xlabel("γ (gamma) — max sharing [%]")
    ax2.set_ylabel("Success rate [%]")
    ax2.set_title("Effect of γ", fontsize=10)
    ax2.axvline(65, color=C_GOLD, linestyle="--", linewidth=1, label="default γ=0.65")
    ax2.legend(fontsize=9)
    ax2.set_ylim(0, 105)
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig("chart1_parameter_sweep.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  → chart1_parameter_sweep.png")


# ── Chart 2: fast_mode speedup ────────────────────────────────────────────────

def chart_speedup():
    print("Chart 2: fast_mode speedup …")
    G = build_grid_graph(12, 12)
    nodes = list(G.nodes())
    rng = random.Random(1)

    pairs = []
    while len(pairs) < 30:
        s, t = rng.sample(nodes, 2)
        if nx.has_path(G, s, t):
            pairs.append((s, t))

    times_normal, times_fast = [], []
    for s, t in pairs:
        _, _, ms = run_alternative_paths(G, s, t, fast_mode=False)
        times_normal.append(ms)
        _, _, ms = run_alternative_paths(G, s, t, fast_mode=True)
        times_fast.append(ms)

    avg_n = np.mean(times_normal)
    avg_f = np.mean(times_fast)
    speedup = avg_n / avg_f if avg_f > 0 else 1

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    fig.suptitle(f"fast_mode speedup — 12×12 grid (30 queries) | avg speedup: {speedup:.1f}×", fontsize=12, color=C_NAVY, fontweight="bold")

    x = np.arange(len(pairs))
    ax1.bar(x, times_normal, color=C_TEAL, alpha=0.7, label="Normal mode")
    ax1.bar(x, times_fast, color=C_MINT, alpha=0.9, label="fast_mode")
    ax1.set_xlabel("Query index")
    ax1.set_ylabel("Runtime [ms]")
    ax1.set_title("Per-query runtime comparison")
    ax1.legend(fontsize=9)
    ax1.grid(True, axis="y")

    cats = ["Normal mode\n(full T-test)", "fast_mode\n(clustered + skip T-test)"]
    avgs = [avg_n, avg_f]
    colors = [C_TEAL, C_MINT]
    bars = ax2.bar(cats, avgs, color=colors, width=0.5)
    for bar, val in zip(bars, avgs):
        ax2.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.1,
                 f"{val:.1f} ms", ha="center", va="bottom", fontsize=10, color=C_NAVY)
    ax2.set_ylabel("Average runtime [ms]")
    ax2.set_title(f"Average — {speedup:.1f}× speedup with fast_mode")
    ax2.grid(True, axis="y")

    plt.tight_layout()
    plt.savefig("chart2_speedup.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  → chart2_speedup.png")


# ── Chart 3: Candidate funnel ─────────────────────────────────────────────────

def chart_funnel():
    print("Chart 3: Candidate funnel …")
    G = build_grid_graph(10, 10)
    nodes = list(G.nodes())
    rng = random.Random(2)

    pairs = []
    while len(pairs) < 25:
        s, t = rng.sample(nodes, 2)
        if nx.has_path(G, s, t):
            pairs.append((s, t))

    totals_normal = collections.defaultdict(int)
    totals_fast   = collections.defaultdict(int)

    for s, t in pairs:
        _, fn, _ = run_alternative_paths(G, s, t, fast_mode=False)
        _, ff, _ = run_alternative_paths(G, s, t, fast_mode=True)
        for k in fn:
            totals_normal[k] += fn[k]
        for k in ff:
            totals_fast[k] += ff[k]

    stages = ["candidates", "ubs_pass", "sharing_pass", "ttest_pass"]
    labels = ["Raw\ncandidates", "After UBS\nfilter (c)", "After sharing\nfilter (a)", "After T-test\nfilter (b)"]

    fig, ax = plt.subplots(figsize=(9, 5))
    fig.suptitle("Candidate funnel — how constraints filter via-node candidates", fontsize=12, color=C_NAVY, fontweight="bold")

    x = np.arange(len(stages))
    w = 0.35
    vals_n = [totals_normal[s] for s in stages]
    vals_f = [totals_fast[s] for s in stages]

    b1 = ax.bar(x - w / 2, vals_n, w, label="Normal mode", color=C_TEAL, alpha=0.85)
    b2 = ax.bar(x + w / 2, vals_f, w, label="fast_mode", color=C_MINT, alpha=0.85)

    for b, v in list(zip(b1, vals_n)) + list(zip(b2, vals_f)):
        ax.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.5,
                str(v), ha="center", va="bottom", fontsize=9, color=C_NAVY)

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel("Total candidates (across 25 queries)")
    ax.legend(fontsize=9)
    ax.grid(True, axis="y")

    # annotation: reduction
    n0 = totals_normal["candidates"]
    f0 = totals_fast["candidates"]
    if n0 > 0:
        pct = 100 * (1 - f0 / n0)
        ax.annotate(f"fast_mode enters\nfunnel with {pct:.0f}% fewer\ncandidates (clustering)",
                    xy=(0 + w / 2, f0), xytext=(1.2, f0 * 0.7),
                    arrowprops=dict(arrowstyle="->", color=C_RED, lw=1.2),
                    fontsize=8, color=C_RED)

    plt.tight_layout()
    plt.savefig("chart3_funnel.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  → chart3_funnel.png")


# ── Chart 4: Scaling with graph size ─────────────────────────────────────────

def chart_scaling():
    print("Chart 4: Scaling with graph size …")
    configs = [(4, 4), (6, 6), (8, 8), (10, 10), (12, 12), (14, 14)]
    sizes, times_n, times_f, success_n, success_f = [], [], [], [], []

    for rows, cols in configs:
        G = build_grid_graph(rows, cols)
        n = G.number_of_nodes()
        nodes = list(G.nodes())
        rng = random.Random(3)

        pairs = []
        while len(pairs) < 20:
            s, t = rng.sample(nodes, 2)
            if nx.has_path(G, s, t):
                pairs.append((s, t))

        ms_n, ms_f, ok_n, ok_f = [], [], 0, 0
        for s, t in pairs:
            na, _, ms = run_alternative_paths(G, s, t, fast_mode=False)
            ms_n.append(ms)
            ok_n += (na >= 1)
            na, _, ms = run_alternative_paths(G, s, t, fast_mode=True)
            ms_f.append(ms)
            ok_f += (na >= 1)

        sizes.append(n)
        times_n.append(np.mean(ms_n))
        times_f.append(np.mean(ms_f))
        success_n.append(ok_n / len(pairs) * 100)
        success_f.append(ok_f / len(pairs) * 100)
        print(f"  {rows}×{cols} = {n} nodes | normal {np.mean(ms_n):.1f}ms | fast {np.mean(ms_f):.1f}ms")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    fig.suptitle("Scaling with graph size", fontsize=12, color=C_NAVY, fontweight="bold")

    ax1.plot(sizes, times_n, color=C_TEAL, marker="o", linewidth=2, markersize=6, label="Normal mode")
    ax1.plot(sizes, times_f, color=C_MINT, marker="s", linewidth=2, markersize=6, label="fast_mode")
    ax1.set_xlabel("Graph size (nodes)")
    ax1.set_ylabel("Average runtime [ms]")
    ax1.set_title("Runtime vs. graph size")
    ax1.legend(fontsize=9)
    ax1.grid(True)

    ax2.plot(sizes, success_n, color=C_TEAL, marker="o", linewidth=2, markersize=6, label="Normal mode")
    ax2.plot(sizes, success_f, color=C_MINT, marker="s", linewidth=2, markersize=6, label="fast_mode")
    ax2.set_xlabel("Graph size (nodes)")
    ax2.set_ylabel("Success rate [%]")
    ax2.set_title("Success rate vs. graph size")
    ax2.set_ylim(0, 105)
    ax2.legend(fontsize=9)
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig("chart4_scaling.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  → chart4_scaling.png")


# ── main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("Navigation.py — algorithm analysis")
    print("=" * 60)

    chart_parameter_sweep()
    chart_speedup()
    chart_funnel()
    chart_scaling()

    print("\nAll charts saved. Add them to your presentation on:")
    print("  chart1 → Parameter choices slide (why ε=0.25, γ=0.65)")
    print("  chart2 → fast_mode speedup slide")
    print("  chart3 → Candidate funnel / constraint effectiveness")
    print("  chart4 → Scalability discussion slide")
