import os
import re

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats


RESULT_DIR = "results/final_project"
LEGACY_RESULT_DIR = "results"
COMBINED = os.path.join(RESULT_DIR, "final_project_cpfa_ahcfa_50runs.csv")
DISTRIBUTIONS = ["Random", "Powerlaw", "Clustered"]
ALGORITHMS = ["CPFA", "AHCFA"]


def load_results():
    if os.path.exists(COMBINED):
        return pd.read_csv(COMBINED)

    rows = []
    for distribution in ["Random", "Powerlaw", "Clustered"]:
        for algorithm in ["CPFA", "AHCFA"]:
            path = os.path.join(RESULT_DIR, f"{distribution}_{algorithm}_50runs.csv")
            if not os.path.exists(path):
                continue
            df = pd.read_csv(path)
            df["distribution"] = distribution
            df["algorithm"] = algorithm
            rows.append(df)

    if not rows:
        raise SystemExit("No result CSV files found. Run scripts/run_final_project_50_trials.sh first.")
    return pd.concat(rows, ignore_index=True)


def write_markdown_table(df, path):
    rounded = df.round(2)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("| " + " | ".join(rounded.columns) + " |\n")
        handle.write("| " + " | ".join(["---"] * len(rounded.columns)) + " |\n")
        for _, row in rounded.iterrows():
            handle.write("| " + " | ".join(str(row[col]) for col in rounded.columns) + " |\n")


def plot_metric(data, metric, ylabel, filename):
    labels = []
    values = []
    for distribution in ["Random", "Powerlaw", "Clustered"]:
        for algorithm in ["CPFA", "AHCFA"]:
            subset = data[(data["distribution"] == distribution) & (data["algorithm"] == algorithm)]
            if subset.empty:
                continue
            labels.append(f"{distribution}\n{algorithm}")
            values.append(subset[metric].to_numpy())

    plt.figure(figsize=(10, 5))
    plt.boxplot(values, labels=labels, showmeans=True)
    plt.ylabel(ylabel)
    plt.title(ylabel + " comparison")
    plt.tight_layout()
    plt.savefig(os.path.join(RESULT_DIR, filename), dpi=200)
    plt.close()


def plot_annotated_score_boxplot(data):
    labels = []
    values = []
    positions = []
    pos = 1

    for distribution in DISTRIBUTIONS:
        for algorithm in ALGORITHMS:
            subset = data[(data["distribution"] == distribution) & (data["algorithm"] == algorithm)]
            labels.append(f"{distribution}\n{algorithm}")
            values.append(subset["score"].to_numpy())
            positions.append(pos)
            pos += 1
        pos += 0.8

    plt.figure(figsize=(12, 6))
    plt.boxplot(values, positions=positions, widths=0.55, showmeans=True)
    plt.xticks(positions, labels)
    plt.ylabel("Collected resources in 12 minutes")
    plt.title("CPFA vs AHCFA collected resources")

    y_max = max(max(v) for v in values)
    for i, vals in enumerate(values):
        mean = np.mean(vals)
        median = np.median(vals)
        plt.text(positions[i], y_max + 4, f"mean={mean:.1f}\nmed={median:.1f}",
                 ha="center", va="bottom", fontsize=8)

    plt.ylim(top=y_max + 18)
    plt.tight_layout()
    plt.savefig(os.path.join(RESULT_DIR, "cpfa_vs_ahcfa_collected_resources_boxplot_annotated.png"), dpi=200)
    plt.close()


def plot_mean_score_bars(data, improvement_df):
    x = np.arange(len(DISTRIBUTIONS))
    width = 0.36

    means = {
        algorithm: [
            data[(data["distribution"] == distribution) & (data["algorithm"] == algorithm)]["score"].mean()
            for distribution in DISTRIBUTIONS
        ]
        for algorithm in ALGORITHMS
    }
    stds = {
        algorithm: [
            data[(data["distribution"] == distribution) & (data["algorithm"] == algorithm)]["score"].std()
            for distribution in DISTRIBUTIONS
        ]
        for algorithm in ALGORITHMS
    }

    plt.figure(figsize=(9, 5))
    plt.bar(x - width / 2, means["CPFA"], width, yerr=stds["CPFA"], capsize=4, label="CPFA")
    plt.bar(x + width / 2, means["AHCFA"], width, yerr=stds["AHCFA"], capsize=4, label="AHCFA")
    plt.xticks(x, DISTRIBUTIONS)
    plt.ylabel("Mean collected resources")
    plt.title("Mean collected resources with standard deviation")
    plt.legend()

    for i, distribution in enumerate(DISTRIBUTIONS):
        row = improvement_df[improvement_df["distribution"] == distribution].iloc[0]
        y = max(means["CPFA"][i] + stds["CPFA"][i], means["AHCFA"][i] + stds["AHCFA"][i]) + 5
        plt.text(x[i], y, f"{row['score_improvement_percent']:+.2f}%", ha="center", fontsize=10)

    plt.tight_layout()
    plt.savefig(os.path.join(RESULT_DIR, "cpfa_vs_ahcfa_mean_score_bar.png"), dpi=200)
    plt.close()


def plot_improvement_bars(improvement_df):
    plt.figure(figsize=(8, 4.5))
    colors = ["#2ca02c" if value >= 0 else "#d62728"
              for value in improvement_df["score_improvement_percent"]]
    bars = plt.bar(improvement_df["distribution"], improvement_df["score_improvement_percent"], color=colors)
    plt.axhline(0, color="black", linewidth=1)
    plt.ylabel("AHCFA improvement over CPFA (%)")
    plt.title("Collected-resource percentage improvement")
    for bar in bars:
        height = bar.get_height()
        va = "bottom" if height >= 0 else "top"
        plt.text(bar.get_x() + bar.get_width() / 2, height, f"{height:+.2f}%",
                 ha="center", va=va, fontsize=10)
    plt.tight_layout()
    plt.savefig(os.path.join(RESULT_DIR, "ahcfa_percent_improvement_bar.png"), dpi=200)
    plt.close()


def load_forage_data(distribution, algorithm):
    legacy_name = "cluster" if distribution == "Clustered" else distribution.lower()
    path = os.path.join(
        LEGACY_RESULT_DIR,
        f"{legacy_name}_{algorithm}_r24_tag256_10by10_quard_arena_0_ForageData.txt",
    )
    if not os.path.exists(path):
        return pd.DataFrame()

    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for seed, line in enumerate(handle, start=1):
            values = [int(v) for v in re.findall(r"-?\d+", line)]
            if not values:
                continue
            cumulative = np.cumsum(values)
            for minute, collected in enumerate(cumulative, start=1):
                rows.append(
                    {
                        "distribution": distribution,
                        "algorithm": algorithm,
                        "seed": seed,
                        "minute": minute,
                        "cumulative_score": collected,
                        "minute_score": values[minute - 1],
                    }
                )
    return pd.DataFrame(rows)


def load_all_forage_data():
    rows = []
    for distribution in DISTRIBUTIONS:
        for algorithm in ALGORITHMS:
            df = load_forage_data(distribution, algorithm)
            if not df.empty:
                rows.append(df)
    if not rows:
        return pd.DataFrame()
    return pd.concat(rows, ignore_index=True)


def plot_collection_curves(forage_data):
    if forage_data.empty:
        return

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), sharey=True)
    for distribution in DISTRIBUTIONS:
        ax = axes[DISTRIBUTIONS.index(distribution)]
        for algorithm in ALGORITHMS:
            subset = forage_data[
                (forage_data["distribution"] == distribution) &
                (forage_data["algorithm"] == algorithm)
            ]
            if subset.empty:
                continue
            grouped = subset.groupby("minute")["cumulative_score"].agg(["mean", "std"]).reset_index()
            ax.plot(grouped["minute"], grouped["mean"], marker="o", label=algorithm)
            ax.fill_between(
                grouped["minute"],
                grouped["mean"] - grouped["std"],
                grouped["mean"] + grouped["std"],
                alpha=0.15,
            )

        ax.set_xlabel("Time (minutes)")
        ax.set_ylabel("Mean cumulative collected resources")
        ax.set_title(distribution)
        ax.set_xticks(range(1, 13))
        ax.set_ylim(0, 215)
        ax.legend()

        plt.figure(figsize=(7.5, 4.8))
        for algorithm in ALGORITHMS:
            subset = forage_data[
                (forage_data["distribution"] == distribution) &
                (forage_data["algorithm"] == algorithm)
            ]
            if subset.empty:
                continue
            grouped = subset.groupby("minute")["cumulative_score"].agg(["mean", "std"]).reset_index()
            plt.plot(grouped["minute"], grouped["mean"], marker="o", label=algorithm)
            plt.fill_between(
                grouped["minute"],
                grouped["mean"] - grouped["std"],
                grouped["mean"] + grouped["std"],
                alpha=0.15,
            )
            final_mean = grouped[grouped["minute"] == 12]["mean"].iloc[0]
            plt.text(12.15, final_mean, f"{final_mean:.1f}", va="center", fontsize=9)

        plt.xlabel("Time (minutes)")
        plt.ylabel("Mean cumulative collected resources")
        plt.title(f"Collected resources over time: {distribution}")
        plt.xticks(range(1, 13))
        plt.ylim(0, 215)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(RESULT_DIR, f"{distribution.lower()}_collection_curve.png"), dpi=200)
        plt.close()

    fig.suptitle("Collected resources over time", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(RESULT_DIR, "cpfa_vs_ahcfa_collection_over_time.png"), dpi=200)
    plt.close(fig)


def make_minute_by_minute_table(forage_data):
    if forage_data.empty:
        return pd.DataFrame()

    rows = []
    for distribution in DISTRIBUTIONS:
        for minute in range(1, 13):
            row = {"distribution": distribution, "minute": minute}
            for algorithm in ALGORITHMS:
                subset = forage_data[
                    (forage_data["distribution"] == distribution) &
                    (forage_data["algorithm"] == algorithm) &
                    (forage_data["minute"] == minute)
                ]
                row[f"{algorithm.lower()}_mean_collected"] = subset["cumulative_score"].mean()
                row[f"{algorithm.lower()}_std_collected"] = subset["cumulative_score"].std()
            cpfa = row["cpfa_mean_collected"]
            ahcfa = row["ahcfa_mean_collected"]
            row["absolute_gain"] = ahcfa - cpfa
            row["improvement_percent"] = 100.0 * (ahcfa - cpfa) / cpfa if cpfa else np.nan
            rows.append(row)

    table = pd.DataFrame(rows)
    table.to_csv(os.path.join(RESULT_DIR, "minute_by_minute_collection_table.csv"), index=False)
    write_markdown_table(table, os.path.join(RESULT_DIR, "minute_by_minute_collection_table.md"))
    return table


def make_collection_phase_table(forage_data):
    if forage_data.empty:
        return pd.DataFrame()

    checkpoints = [4, 8, 12]
    rows = []
    for distribution in DISTRIBUTIONS:
        row = {"distribution": distribution}
        for algorithm in ALGORITHMS:
            subset = forage_data[
                (forage_data["distribution"] == distribution) &
                (forage_data["algorithm"] == algorithm)
            ]
            for minute in checkpoints:
                value = subset[subset["minute"] == minute]["cumulative_score"].mean()
                row[f"{algorithm.lower()}_{minute}min_mean"] = value
        for minute in checkpoints:
            cpfa = row.get(f"cpfa_{minute}min_mean", np.nan)
            ahcfa = row.get(f"ahcfa_{minute}min_mean", np.nan)
            row[f"{minute}min_improvement_percent"] = 100.0 * (ahcfa - cpfa) / cpfa if cpfa else np.nan
        rows.append(row)

    phase_df = pd.DataFrame(rows)
    phase_df.to_csv(os.path.join(RESULT_DIR, "collection_phase_summary.csv"), index=False)
    write_markdown_table(phase_df, os.path.join(RESULT_DIR, "collection_phase_summary.md"))
    return phase_df


def summarize(data):
    summary = data.groupby(["distribution", "algorithm"]).agg(
        runs=("score", "count"),
        mean_score=("score", "mean"),
        median_score=("score", "median"),
        std_score=("score", "std"),
        min_score=("score", "min"),
        max_score=("score", "max"),
        mean_time=("time_seconds", "mean"),
        median_time=("time_seconds", "median"),
    ).reset_index()
    print(summary.round(2))
    summary.to_csv(os.path.join(RESULT_DIR, "score_summary_table.csv"), index=False)
    write_markdown_table(summary, os.path.join(RESULT_DIR, "score_summary_table.md"))

    rows = []
    for distribution in DISTRIBUTIONS:
        cpfa = data[(data["distribution"] == distribution) & (data["algorithm"] == "CPFA")]
        ahcfa = data[(data["distribution"] == distribution) & (data["algorithm"] == "AHCFA")]
        if cpfa.empty or ahcfa.empty:
            continue

        merged = cpfa[["seed", "score", "time_seconds"]].merge(
            ahcfa[["seed", "score", "time_seconds"]],
            on="seed",
            suffixes=("_cpfa", "_ahcfa"),
        )
        mean_cpfa = cpfa["score"].mean()
        mean_ahcfa = ahcfa["score"].mean()
        median_cpfa = cpfa["score"].median()
        median_ahcfa = ahcfa["score"].median()
        improvement = 100.0 * (mean_ahcfa - mean_cpfa) / mean_cpfa if mean_cpfa else 0.0
        paired_wins = (merged["score_ahcfa"] > merged["score_cpfa"]).sum()
        paired_ties = (merged["score_ahcfa"] == merged["score_cpfa"]).sum()
        paired_losses = (merged["score_ahcfa"] < merged["score_cpfa"]).sum()
        paired_mean_delta = (merged["score_ahcfa"] - merged["score_cpfa"]).mean()
        paired_median_delta = (merged["score_ahcfa"] - merged["score_cpfa"]).median()
        wilcoxon_p = stats.wilcoxon(
            merged["score_ahcfa"],
            merged["score_cpfa"],
            zero_method="wilcox",
            alternative="two-sided",
        ).pvalue

        rows.append(
            {
                "distribution": distribution,
                "cpfa_mean_score": mean_cpfa,
                "ahcfa_mean_score": mean_ahcfa,
                "absolute_mean_gain": mean_ahcfa - mean_cpfa,
                "score_improvement_percent": improvement,
                "cpfa_median_score": median_cpfa,
                "ahcfa_median_score": median_ahcfa,
                "absolute_median_gain": median_ahcfa - median_cpfa,
                "paired_mean_delta": paired_mean_delta,
                "paired_median_delta": paired_median_delta,
                "paired_wins": paired_wins,
                "paired_ties": paired_ties,
                "paired_losses": paired_losses,
                "paired_runs": len(merged),
                "wilcoxon_p_value": wilcoxon_p,
            }
        )

    improvement_df = pd.DataFrame(rows)
    print("\nCollected-resource improvement:")
    print(improvement_df.round(2))
    improvement_df.to_csv(os.path.join(RESULT_DIR, "ahcfa_vs_cpfa_improvement.csv"), index=False)
    write_markdown_table(improvement_df, os.path.join(RESULT_DIR, "ahcfa_vs_cpfa_improvement.md"))
    return summary, improvement_df


def main():
    data = load_results()
    summary, improvement_df = summarize(data)
    plot_metric(data, "score", "Collected resources in 12 minutes", "cpfa_vs_ahcfa_collected_resources_boxplot.png")
    plot_metric(data, "time_seconds", "Foraging time in seconds", "cpfa_vs_ahcfa_time_boxplot.png")
    plot_annotated_score_boxplot(data)
    plot_mean_score_bars(data, improvement_df)
    plot_improvement_bars(improvement_df)

    forage_data = load_all_forage_data()
    if not forage_data.empty:
        forage_data.to_csv(os.path.join(RESULT_DIR, "per_minute_collection_data.csv"), index=False)
        minute_table = make_minute_by_minute_table(forage_data)
        phase_df = make_collection_phase_table(forage_data)
        print("\nMinute-by-minute collection table:")
        print(minute_table.round(2))
        print("\nCollection phase summary:")
        print(phase_df.round(2))
        plot_collection_curves(forage_data)


if __name__ == "__main__":
    main()
