# Adaptive Clustering-Based Heatmap Visualization for CPFA/AHCFA

This directory contains tools for visualizing the **adaptive clusters discovered by your AHCFA algorithm** and the robot exploration patterns around those discovered clusters.

## Setup

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```
## Usage

### Option 1: Live Adaptive Clustering Heatmap (Recommended)
Run the live viewer that automatically updates as new CSV files are generated:

```bash
python3 live_heatmap_viewer.py
```

Optional parameters:
- `--pattern "dotplot_data/visited_positions_*.csv"` - Change the file pattern to monitor
- `--interval 2000` - Update interval in milliseconds

### Option 2: Static Adaptive Clustering Heatmap
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
3. **Cluster Labels**: Each cluster shows:
   - A label (C0, C1, C2, etc.)
   - Number of visits recorded in that region
   - The adaptive region boundaries
4. **Nest Location**: Green circle at the origin shows the robot nest

### Reading the Visualization

- **Large clusters with high visit counts** = Regions your algorithm identified as important
- **Overlapping colors** = Adaptive regions that are neighbors in the spatial hierarchy
- **Sparse visit density in a cluster** = The algorithm discovered the region but robots haven't thoroughly explored it yet

### Already Implemented in CPFA_loop_functions.cpp:

1. **Headers added** (in .h file):
   - `#include <fstream>` - File I/O
   - `#include <sys/stat.h>` - Directory operations
   - `#include <cerrno>` - Error handling

2. **Member variables** (in .h file):
   - `std::vector<argos::CVector2> VisitedPositions;` - Stores visited positions
   - `std::vector<argos::CVector2> ClusterCenters;` - (optional) Stores food cluster centers

3. **Methods** (in .cpp file):
   - `exportVisitedPositionsToCSV(filename)` - Exports positions + adaptive regions
   - `createDirectoryIfNotExists(dirPath)` - Creates output directories

4. **Export call in PreStep()** - Every 10 seconds:
   ```cpp
   if(currentTime - lastExportTime >= 10.0) {
       createDirectoryIfNotExists("dotplot_data");
       std::string filename = "dotplot_data/visited_positions_" + std::to_string((int)currentTime) + ".csv";
       exportVisitedPositionsToCSV(filename);
       lastExportTime = currentTime;
   }
   ```

5. **Data sources**:
   - `RecordVisitedLocations()` - Now also populates VisitedPositions vector
   - `AdaptiveRegions` - Automatically exported with boundaries and visit counts

### CSV Export Format

The exported CSV includes:
```
# Visited Positions Export - Simulation Time: 120 seconds
# Total Positions: 5432
# Number of Adaptive Clusters: 8
# Format: X,Y (coordinates in meters)
# Adaptive Regions (Discovered Clusters):
# Region 0: [-4,0] x [-4,0] Visits=652 ResourceHits=12
# Region 1: [0,4] x [-4,0] Visits=543 ResourceHits=8
# Region 2: [0,4] x [0,4] Visits=823 ResourceHits=15
...
# === VISITED POSITIONS DATA ===
X,Y
-2.5,3.1
-2.3,3.2
0.8,-1.5
...
```

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
├── live_heatmap_viewer.py           # Live visualization
├── static_heatmap_generator.py      # Static heatmap generation
├── HEATMAP_README.md                # This file
└── requirements.txt                 # Python dependencies
```