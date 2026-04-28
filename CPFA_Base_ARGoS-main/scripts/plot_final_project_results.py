import os

import matplotlib.pyplot as plt
import pandas as pd


RESULT_DIR = "results/final_project"
COMBINED = os.path.join(RESULT_DIR, "final_project_cpfa_ahcfa_50runs.csv")


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


def summarize(data):
    summary = data.groupby(["distribution", "algorithm"]).agg(
        runs=("score", "count"),
        mean_score=("score", "mean"),
        median_score=("score", "median"),
        std_score=("score", "std"),
        mean_time=("time_seconds", "mean"),
        median_time=("time_seconds", "median"),
    )
    print(summary.round(2))

    rows = []
    for distribution in ["Random", "Powerlaw", "Clustered"]:
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
        improvement = 100.0 * (mean_ahcfa - mean_cpfa) / mean_cpfa if mean_cpfa else 0.0
        paired_wins = (merged["score_ahcfa"] > merged["score_cpfa"]).sum()
        paired_ties = (merged["score_ahcfa"] == merged["score_cpfa"]).sum()

        rows.append(
            {
                "distribution": distribution,
                "cpfa_mean_score": mean_cpfa,
                "ahcfa_mean_score": mean_ahcfa,
                "score_improvement_percent": improvement,
                "paired_wins": paired_wins,
                "paired_ties": paired_ties,
                "paired_runs": len(merged),
            }
        )

    improvement_df = pd.DataFrame(rows)
    print("\nCollected-resource improvement:")
    print(improvement_df.round(2))
    improvement_df.to_csv(os.path.join(RESULT_DIR, "ahcfa_vs_cpfa_improvement.csv"), index=False)


def main():
    data = load_results()
    summarize(data)
    plot_metric(data, "score", "Collected resources in 12 minutes", "cpfa_vs_ahcfa_collected_resources_boxplot.png")
    plot_metric(data, "time_seconds", "Foraging time in seconds", "cpfa_vs_ahcfa_time_boxplot.png")


if __name__ == "__main__":
    main()
