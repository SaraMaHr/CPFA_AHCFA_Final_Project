#!/usr/bin/env python3
import argparse
import os

import matplotlib.colors as colors
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.patches import Rectangle


RESULT_DIR = "results/final_project"
INPUT_CSV = os.path.join(RESULT_DIR, "adaptive_quadtree_regions.csv")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot the actual AHCFA adaptive QuadTree leaf regions."
    )
    parser.add_argument("--distribution", choices=["Random", "Powerlaw", "Clustered"], default="Clustered")
    parser.add_argument("--algorithm", choices=["CPFA", "AHCFA"], default="AHCFA")
    parser.add_argument("--seed", type=int, default=None, help="Plot one seed. Default: latest matching seed.")
    parser.add_argument(
        "--metric",
        choices=["visits", "resource_hits", "resource_weight", "score"],
        default="visits",
    )
    parser.add_argument(
        "--arena-size",
        type=float,
        default=10.0,
        help="Full square arena width in meters. Default: 10.0.",
    )
    parser.add_argument(
        "--zoom-to-regions",
        action="store_true",
        help="Zoom to the exported QuadTree region bounds instead of the full arena.",
    )
    parser.add_argument("--output", default=None)
    return parser.parse_args()


def load_regions(args):
    if not os.path.exists(INPUT_CSV):
        raise SystemExit(
            f"{INPUT_CSV} does not exist yet. Rebuild and run at least one AHCFA experiment first."
        )

    data = pd.read_csv(INPUT_CSV)
    subset = data[
        (data["distribution"] == args.distribution)
        & (data["algorithm"] == args.algorithm)
        & (data["is_leaf"] == 1)
    ].copy()

    if args.seed is None:
        if subset.empty:
            raise SystemExit("No matching QuadTree region rows found.")
        args.seed = int(subset["seed"].iloc[-1])

    subset = subset[subset["seed"] == args.seed]
    if subset.empty:
        raise SystemExit("No matching QuadTree region rows found for that seed.")
    return subset


def draw_regions(regions, args):
    fig, ax = plt.subplots(figsize=(6.0, 5.8))
    metric_values = regions[args.metric].astype(float)
    max_value = metric_values.max()
    norm = colors.Normalize(vmin=0, vmax=max_value if max_value > 0 else 1)
    cmap = plt.cm.Reds

    for _, row in regions.iterrows():
        min_x = row["min_x"]
        max_x = row["max_x"]
        min_y = row["min_y"]
        max_y = row["max_y"]
        value = float(row[args.metric])
        width = max_x - min_x
        height = max_y - min_y

        patch = Rectangle(
            (min_x, min_y),
            width,
            height,
            facecolor=cmap(norm(value)),
            edgecolor="white",
            linewidth=1.1,
        )
        ax.add_patch(patch)

        if width >= 0.55 and height >= 0.55:
            label = f"{value:.0f}" if args.metric != "score" else f"{value:.1f}"
            text_color = "white" if norm(value) > 0.58 else "black"
            ax.text(
                min_x + width / 2,
                min_y + height / 2,
                label,
                ha="center",
                va="center",
                fontsize=6,
                color=text_color,
            )

    region_x_min = regions["min_x"].min()
    region_x_max = regions["max_x"].max()
    region_y_min = regions["min_y"].min()
    region_y_max = regions["max_y"].max()

    if args.zoom_to_regions:
        x_min = region_x_min
        x_max = region_x_max
        y_min = region_y_min
        y_max = region_y_max
    else:
        half_arena = args.arena_size / 2.0
        x_min = -half_arena
        x_max = half_arena
        y_min = -half_arena
        y_max = half_arena

        arena = Rectangle(
            (x_min, y_min),
            args.arena_size,
            args.arena_size,
            fill=False,
            edgecolor="black",
            linewidth=1.8,
        )
        safe_range = Rectangle(
            (region_x_min, region_y_min),
            region_x_max - region_x_min,
            region_y_max - region_y_min,
            fill=False,
            edgecolor="black",
            linewidth=1.0,
            linestyle="--",
            alpha=0.7,
        )
        ax.add_patch(arena)
        ax.add_patch(safe_range)

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x position (m)")
    ax.set_ylabel("y position (m)")
    ax.set_title(
        f"{args.distribution} {args.algorithm} adaptive QuadTree\n"
        f"seed {args.seed}, colored by {args.metric.replace('_', ' ')}",
        fontsize=11,
    )

    sm = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    fig.colorbar(sm, ax=ax, fraction=0.046, pad=0.04, label=args.metric.replace("_", " "))
    fig.tight_layout()

    output = args.output
    if output is None:
        output = os.path.join(
            RESULT_DIR,
            f"adaptive_quadtree_{args.distribution}_{args.algorithm}_seed{args.seed}_{args.metric}.png",
        )

    fig.savefig(output, dpi=300)
    print(output)


def main():
    args = parse_args()
    regions = load_regions(args)
    draw_regions(regions, args)


if __name__ == "__main__":
    main()
