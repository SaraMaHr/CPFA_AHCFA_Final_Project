import os

import numpy as np
import pandas as pd


RESULT_DIR = "results/final_project"
INPUT = os.path.join(RESULT_DIR, "collection_milestones.csv")
DISTRIBUTIONS = ["Random", "Powerlaw", "Clustered"]
ALGORITHMS = ["CPFA", "AHCFA"]


def write_markdown_table(df, path):
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("| " + " | ".join(df.columns) + " |\n")
        handle.write("| " + " | ".join(["---"] * len(df.columns)) + " |\n")
        for _, row in df.iterrows():
            handle.write("| " + " | ".join(str(row[col]) for col in df.columns) + " |\n")


def format_seconds(value):
    if pd.isna(value):
        return "not reached"
    return f"{value:.1f}"


def main():
    if not os.path.exists(INPUT):
        raise SystemExit("Missing results/final_project/collection_milestones.csv. Run scripts/run_endgame_phase_trials.sh first.")

    data = pd.read_csv(INPUT)
    for col in ["time_80", "time_90", "time_100", "phase_80_90", "phase_90_100"]:
        data.loc[data[col] < 0, col] = np.nan

    raw_summary = []
    display_rows = []
    for distribution in DISTRIBUTIONS:
        row = {"Config.": distribution}
        raw_row = {"distribution": distribution}
        for algorithm in ALGORITHMS:
            subset = data[(data["distribution"] == distribution) & (data["algorithm"] == algorithm)]
            complete = subset.dropna(subset=["phase_80_90", "phase_90_100"])

            phase_80_90 = complete["phase_80_90"].mean()
            phase_90_100 = complete["phase_90_100"].mean()
            reached_runs = len(complete)

            row[f"{algorithm} 80% -> 90%"] = format_seconds(phase_80_90)
            row[f"{algorithm} 90% -> 100%"] = format_seconds(phase_90_100)
            raw_row[f"{algorithm.lower()}_phase_80_90_mean"] = phase_80_90
            raw_row[f"{algorithm.lower()}_phase_90_100_mean"] = phase_90_100
            raw_row[f"{algorithm.lower()}_complete_runs"] = reached_runs

        cpfa_end = raw_row["cpfa_phase_90_100_mean"]
        ahcfa_end = raw_row["ahcfa_phase_90_100_mean"]
        if pd.isna(cpfa_end) or pd.isna(ahcfa_end) or cpfa_end == 0:
            improvement = np.nan
            row["Endgame Improvement"] = "not available"
        else:
            improvement = 100.0 * (cpfa_end - ahcfa_end) / cpfa_end
            row["Endgame Improvement"] = f"{improvement:+.2f}%"

        raw_row["endgame_improvement_percent"] = improvement
        display_rows.append(row)
        raw_summary.append(raw_row)

    display = pd.DataFrame(display_rows)
    raw = pd.DataFrame(raw_summary)

    os.makedirs(RESULT_DIR, exist_ok=True)
    display.to_csv(os.path.join(RESULT_DIR, "endgame_phase_table.csv"), index=False)
    raw.to_csv(os.path.join(RESULT_DIR, "endgame_phase_summary_raw.csv"), index=False)
    write_markdown_table(display, os.path.join(RESULT_DIR, "endgame_phase_table.md"))

    print(display.to_string(index=False))


if __name__ == "__main__":
    main()
