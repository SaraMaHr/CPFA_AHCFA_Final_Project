#!/usr/bin/env python3
import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


RESULT_DIR = "results/final_project"
INPUT_CSV = os.path.join(RESULT_DIR, "visit_heatmap_cells.csv")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create Arturo-style 12x12 visit heatmaps from ARGoS visit logs."
    )
    parser.add_argument("--distribution", choices=["Random", "Powerlaw", "Clustered"], default="Random")
    parser.add_argument("--algorithm", choices=["CPFA", "AHCFA"], default="AHCFA")
    parser.add_argument("--seed", type=int, default=None, help="Plot one seed. Default: aggregate all seeds.")
    parser.add_argument(
        "--aggregate",
        choices=["mean", "sum"],
        default="mean",
        help="How to combine multiple seeds when --seed is not given.",
    )
    parser.add_argument("--output", default=None)
    return parser.parse_args()


def load_heatmap(args):
    if not os.path.exists(INPUT_CSV):
        raise SystemExit(
            f"{INPUT_CSV} does not exist yet. Rebuild and run at least one experiment first."
        )

    data = pd.read_csv(INPUT_CSV)
    subset = data[
        (data["distribution"] == args.distribution)
        & (data["algorithm"] == args.algorithm)
    ].copy()

    if args.seed is not None:
        subset = subset[subset["seed"] == args.seed]

    if subset.empty:
        raise SystemExit("No matching heatmap rows found for that algorithm/distribution/seed.")

    grid_size = int(subset["grid_size"].iloc[0])
    group = subset.groupby(["row", "col"])["visits"]
    values = group.mean() if args.aggregate == "mean" and args.seed is None else group.sum()

    matrix = np.zeros((grid_size, grid_size), dtype=float)
    for (row, col), visits in values.items():
        matrix[int(row), int(col)] = visits
    return matrix


def draw_heatmap(matrix, args):
    fig, ax = plt.subplots(figsize=(5.2, 5.2))
    image = ax.imshow(matrix, cmap="Reds", interpolation="nearest")

    ax.set_xticks(np.arange(matrix.shape[1]))
    ax.set_yticks(np.arange(matrix.shape[0]))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.tick_params(length=0)

    ax.set_xticks(np.arange(-0.5, matrix.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, matrix.shape[0], 1), minor=True)
    ax.grid(which="minor", color="white", linewidth=1.2)

    max_value = matrix.max()
    threshold = max_value * 0.55 if max_value > 0 else 0
    for row in range(matrix.shape[0]):
        for col in range(matrix.shape[1]):
            value = matrix[row, col]
            label = f"{value:.0f}" if args.seed is not None or args.aggregate == "sum" else f"{value:.1f}"
            color = "white" if value > threshold else "black"
            ax.text(col, row, label, ha="center", va="center", fontsize=6, color=color)

    seed_label = f"seed {args.seed}" if args.seed is not None else f"{args.aggregate} over seeds"
    ax.set_title(f"{args.distribution} {args.algorithm} visit counts\n{seed_label}", fontsize=11)
    fig.colorbar(image, ax=ax, fraction=0.046, pad=0.04, label="visits")
    fig.tight_layout()

    output = args.output
    if output is None:
        seed_part = f"seed{args.seed}" if args.seed is not None else args.aggregate
        output = os.path.join(
            RESULT_DIR,
            f"visit_heatmap_{args.distribution}_{args.algorithm}_{seed_part}.png",
        )

    fig.savefig(output, dpi=300)
    print(output)


def main():
    args = parse_args()
    matrix = load_heatmap(args)
    draw_heatmap(matrix, args)


if __name__ == "__main__":
    main()
