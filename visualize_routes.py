from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_data(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def build_node_map(nodes: list[dict]) -> dict[int, dict]:
    return {int(node["id"]): node for node in nodes}


def plot_graph(data: dict, output_path: Path) -> None:
    nodes = data.get("nodes", [])
    edges = data.get("edges", [])
    routes = data.get("routes", [])
    node_map = build_node_map(nodes)

    fig, ax = plt.subplots(figsize=(11, 8), dpi=150)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_facecolor("#fbfbfb")

    for edge in edges:
        u = node_map[int(edge["u"])]
        v = node_map[int(edge["v"])]
        ax.plot([u["x"], v["x"]], [u["y"], v["y"]], color="#cfd6dd", linewidth=1.2, zorder=1)

    for route in routes:
        route_nodes = route.get("nodes", [])
        if len(route_nodes) < 2:
            continue

        xs = [node_map[int(node_id)]["x"] for node_id in route_nodes]
        ys = [node_map[int(node_id)]["y"] for node_id in route_nodes]
        color = route.get("color") or "#1f77b4"
        label = route.get("label") or "route"
        ax.plot(xs, ys, color=color, linewidth=2.4, marker="o", markersize=3.8, label=label, zorder=3)

    for node in nodes:
        is_station = bool(node.get("is_station"))
        color = "#d62728" if is_station else "#111111"
        size = 72 if is_station else 34
        ax.scatter(node["x"], node["y"], s=size, c=color, edgecolors="white", linewidths=0.8, zorder=4)
        ax.text(
            node["x"] + 6,
            node["y"] + 6,
            str(node["id"]),
            fontsize=9,
            color="#222222",
            zorder=5,
        )

    title = data.get("meta", {}).get("title") or "route visualization"
    ax.set_title(title, fontsize=14, pad=12)
    ax.grid(True, color="#e9edf2", linewidth=0.6, linestyle="--", zorder=0)
    ax.tick_params(labelsize=8)

    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(
            handles,
            labels,
            loc="upper left",
            bbox_to_anchor=(1.01, 1.0),
            frameon=True,
            framealpha=0.92,
            fontsize=8,
        )

    fig.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot pickup-point routes from result_viz.json")
    parser.add_argument("input", nargs="?", default="测试数据/示例/result_viz.json", help="Path to result_viz.json")
    parser.add_argument("-o", "--output", default="result_viz.png", help="Output image path")
    parser.add_argument("--show", action="store_true", help="Open the plot window after saving")
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)
    data = load_data(input_path)
    plot_graph(data, output_path)

    if args.show:
        img = plt.imread(output_path)
        plt.figure(figsize=(11, 8), dpi=150)
        plt.imshow(img)
        plt.axis("off")
        plt.show()

    print(f"saved {output_path}")


if __name__ == "__main__":
    main()