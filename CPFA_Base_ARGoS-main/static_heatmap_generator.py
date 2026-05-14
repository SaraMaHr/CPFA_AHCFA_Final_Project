
"""
Cluster-Based Heatmap Generator for AHCFA
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
import os
import sys


#Gets the latest csv file stored in the folder
def find_latest_csv(pattern="dotplot_data/visited_positions_*.csv"):
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)

#Loads the csv file and extracts the visited positions, metadata, and adaptive region information
def load_visited_positions(filename):
    try:
        df = pd.read_csv(filename, comment='#')

        metadata = {}
        adaptive_regions = []

        with open(filename, 'r') as f:
            for line in f:
                if not line.startswith('#'):
                    break

                if 'Simulation Time:' in line:
                    metadata['sim_time'] = line.split(':')[1].strip().split()[0]

                elif 'Leaf Clusters' in line:
                    metadata['num_clusters'] = line.split(':')[1].strip()

                elif line.startswith('# Region '):
                    try:
                        line_content = line.split(':', 1)[1].strip()

                        parts = line_content.split('x')

                        x_part = parts[0].strip()
                        remaining = parts[1].strip()

                        x_vals = x_part.strip('[]').split(',')
                        min_x = float(x_vals[0])
                        max_x = float(x_vals[1])

                        bracket_index = remaining.index(']')
                        y_part = remaining[:bracket_index + 1]
                        stats_part = remaining[bracket_index + 1:].strip()

                        y_vals = y_part.strip('[]').split(',')
                        min_y = float(y_vals[0])
                        max_y = float(y_vals[1])

                        visits = 0
                        resource_hits = 0

                        for stat in stats_part.split():
                            if stat.startswith('Visits='):
                                visits = int(stat.split('=')[1])
                            elif stat.startswith('ResourceHits='):
                                resource_hits = int(stat.split('=')[1])

                        adaptive_regions.append({
                            'MinX': min_x,
                            'MaxX': max_x,
                            'MinY': min_y,
                            'MaxY': max_y,
                            'Visits': visits,
                            'ResourceHits': resource_hits
                        })

                    except Exception as e:
                        print("Could not parse region:", e)

        if 'X' in df.columns and 'Y' in df.columns:
            positions = df[['X', 'Y']].values
        else:
            positions = None

        return positions, metadata, adaptive_regions

    except Exception as e:
        print("Error loading file:", e)
        return None, {}, []

#Creates a heatmap based on the adaptive regions and their visit counts, with the nest at the center and the arena boundaries defined
def create_cluster_heatmap(positions, metadata, adaptive_regions, output_file=None):

    fig, ax = plt.subplots(figsize=(12, 10))
    valid_regions = [r for r in adaptive_regions ]

    if valid_regions:
        max_visits = max(r['Visits'] for r in valid_regions)
        cmap = plt.cm.YlOrRd

        for idx, region in enumerate(valid_regions):

            width = region['MaxX'] - region['MinX']
            height = region['MaxY'] - region['MinY']

            intensity = region['Visits'] / max_visits
            color = cmap(intensity)

            rect = plt.Rectangle(
                (region['MinX'], region['MinY']),
                width,
                height,
                facecolor=color,
                edgecolor='black',
                linewidth=2,
                alpha=0.75
            )

            ax.add_patch(rect)

            center_x = (region['MinX'] + region['MaxX']) / 2
            center_y = (region['MinY'] + region['MaxY']) / 2

            ax.text(
                center_x,
                center_y,
                f'{region["Visits"]}',
                ha='center',
                va='center',
                fontsize=10,
                fontweight='bold'
            )

        sm = plt.cm.ScalarMappable(
            cmap=cmap,
            norm=plt.Normalize(vmin=0, vmax=max_visits)
        )

        cbar = plt.colorbar(sm, ax=ax)
        cbar.set_label("Cluster Visit Count", rotation=270, labelpad=20)

    # Draw nest
    nest = plt.Circle((0, 0), 0.15, color='green', zorder=10)
    ax.add_patch(nest)

    # Arena settings
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_aspect('equal')

    ax.grid(True, alpha=0.3)
    ax.axhline(0, color='black', linewidth=0.5)
    ax.axvline(0, color='black', linewidth=0.5)

    sim_time = metadata.get('sim_time', 'Unknown')
    num_clusters = len(valid_regions)

    ax.set_title(
        f'Adaptive Cluster Heatmap(Clustered Food)\n'
        f'Time: {sim_time}s | Active Clusters: {num_clusters}',
        fontsize=14,
        fontweight='bold'
    )
    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print("Saved:", output_file)
    else:
        plt.show()


def main():

    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = find_latest_csv()

    if csv_file is None:
        print("No CSV files found.")
        return

    print("Loading:", csv_file)

    positions, metadata, adaptive_regions = load_visited_positions(csv_file)

    if not adaptive_regions:
        print("No adaptive regions found.")
        return

    base_name = os.path.splitext(csv_file)[0]
    output_file = f"{base_name}_cluster_heatmap(Random Food).png"

    create_cluster_heatmap(
        positions,
        metadata,
        adaptive_regions,
        output_file
    )


if __name__ == "__main__":
    main()