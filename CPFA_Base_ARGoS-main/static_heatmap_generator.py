"""
Clustering-based Heatmap Generator for CPFA
Reads visited positions and creates a density heatmap based on robot visit clusters
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde
import glob
import os
import sys

def find_latest_csv(pattern="dotplot_data/visited_positions_*.csv"):
    """Find the most recent CSV file matching the pattern"""
    files = glob.glob(pattern)
    if not files:
        return None
    
    # Sort by modification time, get the latest
    latest_file = max(files, key=os.path.getmtime)
    return latest_file

def load_visited_positions(filename):
    """Load visited positions data from CSV file"""
    try:
        # Read the CSV file, skipping header comments
        df = pd.read_csv(filename, comment='#')
        
        # Extract metadata and adaptive regions from comments
        metadata = {}
        adaptive_regions = []
        with open(filename, 'r') as f:
            for line in f:
                if line.startswith('#'):
                    if 'Simulation Time:' in line:
                        metadata['sim_time'] = line.split(':')[1].strip().split()[0]
                    elif 'Total Positions:' in line:
                        metadata['total_positions'] = line.split(':')[1].strip()
                    elif 'Leaf Clusters' in line:
                        metadata['num_clusters'] = line.split(':')[1].strip()
                    elif 'Total Adaptive Regions:' in line:
                        metadata['total_regions'] = line.split(':')[1].strip()
                    elif line.startswith('# Region '):
                        # Parse region info: # Region i: [MinX,MaxX] x [MinY,MaxY] Visits=X ResourceHits=Y
                        try:
                            # Remove the "# Region i: " part
                            line_content = line.split(':', 1)[1].strip()  # "[MinX,MaxX] x [MinY,MaxY] Visits=X ResourceHits=Y"
                            
                            # Split by 'x' to separate X and Y ranges
                            parts = line_content.split('x')
                            x_part = parts[0].strip()  # "[MinX,MaxX]"
                            remaining = parts[1].strip()  # "[MinY,MaxY] Visits=X ResourceHits=Y"
                            
                            # Extract X coordinates
                            x_coords = x_part.strip('[]').split(',')
                            min_x = float(x_coords[0].strip())
                            max_x = float(x_coords[1].strip())
                            
                            # Extract Y coordinates and stats
                            # Find where the stats start (after "]")
                            bracket_index = remaining.index(']')
                            y_part = remaining[:bracket_index+1]  # "[MinY,MaxY]"
                            stats_part = remaining[bracket_index+1:].strip()  # "Visits=X ResourceHits=Y"
                            
                            y_coords = y_part.strip('[]').split(',')
                            min_y = float(y_coords[0].strip())
                            max_y = float(y_coords[1].strip())
                            
                            # Extract Visits and ResourceHits
                            visits = 0
                            resource_hits = 0
                            for stat in stats_part.split():
                                if stat.startswith('Visits='):
                                    visits = int(stat.split('=')[1])
                                elif stat.startswith('ResourceHits='):
                                    resource_hits = int(stat.split('=')[1])
                            
                            region = {
                                'MinX': min_x,
                                'MaxX': max_x,
                                'MinY': min_y,
                                'MaxY': max_y,
                                'Visits': visits,
                                'ResourceHits': resource_hits
                            }
                            adaptive_regions.append(region)
                        except Exception as e:
                            print(f"Warning: Could not parse region line: {line} ({e})")
                else:
                    break
        
        # Get only the X,Y data
        if 'X' in df.columns and 'Y' in df.columns:
            positions = df[['X', 'Y']].values
        else:
            positions = None
        
        return positions, metadata, adaptive_regions
        
    except Exception as e:
        print(f"Error loading {filename}: {e}")
        return None, {}, []

def create_cluster_heatmap(positions, metadata, adaptive_regions, output_file=None):
    """Create a density heatmap from visited positions with algorithm-discovered clusters"""
    
    if positions is None or len(positions) == 0:
        print("No visited positions to plot")
        return
    
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Extract X and Y coordinates
    x_data = positions[:, 0]
    y_data = positions[:, 1]
    
    # Create 2D density heatmap using KDE (kernel density estimation)
    # This shows concentration of visits around discovered clusters
    try:
        xy = np.vstack([x_data, y_data])
        z = gaussian_kde(xy)(xy)
        
        # Create scatter plot with density coloring
        scatter = ax.scatter(x_data, y_data, c=z, s=30, cmap='YlOrRd', alpha=0.6, edgecolors='darkred', linewidth=0.5)
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Visit Density', rotation=270, labelpad=20)
    except Exception as e:
        print(f"Could not compute KDE: {e}, using simple scatter plot")
        scatter = ax.scatter(x_data, y_data, c='red', s=20, alpha=0.6)
    
    # Draw algorithm-discovered adaptive regions (clusters)
    if adaptive_regions and len(adaptive_regions) > 0:
        colors = plt.cm.tab10(np.linspace(0, 1, len(adaptive_regions)))
        
        for idx, region in enumerate(adaptive_regions):
            # Draw rectangle for each adaptive region
            width = region['MaxX'] - region['MinX']
            height = region['MaxY'] - region['MinY']
            rect = plt.Rectangle((region['MinX'], region['MinY']), width, height, 
                               fill=False, edgecolor=colors[idx], linewidth=2, 
                               linestyle='--', alpha=0.8, 
                               label=f'Cluster {idx} (Visits: {region["Visits"]})')
            ax.add_patch(rect)
            
            # Add cluster center label
            center_x = (region['MinX'] + region['MaxX']) / 2
            center_y = (region['MinY'] + region['MaxY']) / 2
            ax.text(center_x, center_y, f'C{idx}', fontsize=10, fontweight='bold',
                   ha='center', va='center', bbox=dict(boxstyle='circle', 
                   facecolor=colors[idx], alpha=0.7))
    
    # Draw nest (at origin)
    nest_radius = 0.15
    nest_circle = plt.Circle((0, 0), nest_radius, color='green', alpha=0.7, label='Nest', zorder=10)
    ax.add_patch(nest_circle)
    
    # Set axis limits (arena is 8x8 centered at origin)
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    ax.set_aspect('equal')
    
    # Add grid
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.axvline(x=0, color='k', linewidth=0.5)
    
    # Titles and labels
    sim_time = metadata.get('sim_time', 'Unknown')
    num_clusters = metadata.get('num_clusters', 'Unknown')
    num_positions = len(positions)
    
    ax.set_title(f'Robot Visit Density with Adaptive Clusters\n'
                f'Time: {sim_time}s | Discovered Clusters: {num_clusters} | Total Visits: {num_positions}', 
                fontsize=14, fontweight='bold')
    ax.set_xlabel('World X Coordinate (meters)', fontsize=12)
    ax.set_ylabel('World Y Coordinate (meters)', fontsize=12)
    
    # Add statistics box
    stats_text = f'Total Visits: {num_positions}\n'
    stats_text += f'Adaptive Clusters: {num_clusters}\n'
    if adaptive_regions:
        total_visits = sum([r['Visits'] for r in adaptive_regions])
        total_hits = sum([r['ResourceHits'] for r in adaptive_regions])
        stats_text += f'Total Region Visits: {total_visits}\n'
        stats_text += f'Total Resource Hits: {total_hits}'
    
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
           verticalalignment='top', bbox=dict(boxstyle='round', 
           facecolor='white', alpha=0.9), fontsize=10)
    
    ax.legend(loc='upper right', fontsize=9, ncol=2)
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Adaptive clustering heatmap saved to: {output_file}")
    else:
        plt.show()

def main():
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = find_latest_csv()
        
    if csv_file is None:
        print("No CSV files found matching pattern 'dotplot_data/visited_positions_*.csv'")
        print("Make sure the simulation is running and generating visited positions CSV files.")
        print("\nTo enable CSV export in your C++ code:")
        print("1. Add exportVisitedPositionsToCSV() call in CPFA_loop_functions.cpp PreStep()")
        print("2. Call it every 10 seconds to export robot visited positions")
        return
    
    print(f"Loading data from: {csv_file}")
    
    positions, metadata, adaptive_regions = load_visited_positions(csv_file)
    
    if positions is None or len(positions) == 0:
        print("No visited positions found in CSV file")
        return
    
    print(f"Loaded {len(positions)} visited positions")
    print(f"Simulation time: {metadata.get('sim_time', 'Unknown')} seconds")
    print(f"Number of adaptive clusters discovered: {metadata.get('num_clusters', 'Unknown')}")
    
    # Generate output filename based on input
    base_name = os.path.splitext(csv_file)[0]
    output_file = f"{base_name}_adaptive_clustering_heatmap.png"
    
    create_cluster_heatmap(positions, metadata, adaptive_regions, output_file)

if __name__ == "__main__":
    main()
