# Adaptive Clustering-Based Heatmap Visualization for CPFA/AHCFA

This directory contains tools for visualizing the **adaptive clusters discovered by your AHCFA algorithm** 

## Setup

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```
## Usage
Generate a single heatmap from the latest CSV file:

```bash
python3 static_heatmap_generator.py
```

Or specify a specific file:
```bash
python3 static_heatmap_generator.py dotplot_data/visited_positions_120.csv
```

### Algorithm-Discovered Clusters

This visualization shows:

1. **Visit Density Heatmap**: Robot positions are plotted with color intensity showing density of visits
2. **Adaptive Regions**: Colored rectangles show the boundaries of clusters
3. **Nest Location**: Green circle at the origin shows the robot nest


## File Organization

CSV files are stored in the `dotplot_data/` directory and named: `visited_positions_[simulation_time].csv`

For example: `dotplot_data/visited_positions_120.csv` contains:
- Visited position data at 120 seconds of simulation time
- All adaptive regions discovered by that point
- Visit and resource hit counts for each region

## Directory Structure
```
CPFA_Base_ARGoS-main/
├── dotplot_data/                    # Generated CSV files (created by simulation)
│   ├── visited_positions_10.csv
│   ├── visited_positions_20.csv
│   ├── visited_positions_10_adaptive_clustering_heatmap.png
│   └── ...
├── static_heatmap_generator.py      # Static heatmap generation
├── HEATMAP_README.md                # This file
└── requirements.txt                 # Python dependencies
```