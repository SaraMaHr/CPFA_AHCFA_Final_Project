# Adaptive Hierarchical Clustered Foraging Algorithm Documentation

This document summarizes what we changed for Final Project 3 and how to run,
visualize, and evaluate the implementation.

## 1. Project Requirement Summary

The goal is to improve CPFA for robot swarm foraging.
The key requirements are:

1. Implement an efficient way to organize visited locations or regions.
2. Improve the search strategy so robots can find resources in less-visited
   areas instead of repeatedly visiting only pheromone-rich locations.
3. Evaluate performance in three resource distributions:
   - Random
   - Powerlaw
   - Clustered
4. Use the required experiment settings:
   - Arena size: 10 m x 10 m
   - Collection zone radius: 0.25 m in the assignment table
   - Number of resources: 256
   - Number of robots: 24
   - Foraging time: 12 minutes, or 720 seconds
   - Runs per distribution: 50
5. Compare the new algorithm with the original CPFA.
6. Report percentage improvement in collected resources.
7. Plot results using box plots.

## 2. Motivation From Arturo's GCFA Paper

Arturo's GCFA paper proposes a grid-based complete foraging algorithm. GCFA
stores visited regions in a fixed grid and sends robots toward least-visited
regions.

The paper's result section shows:

1. GCFA improves Random and Powerlaw distributions because it reduces repeated
   searching in already-visited regions.
2. GCFA does not improve Clustered distribution. In clustered environments,
   CPFA can be better because pheromone trails and site fidelity keep robots
   near productive clusters.
3. GCFA's weakness in clusters happens because least-visited-region selection
   may send robots away from known resource clusters and into empty areas.

AHCFA was designed to address that weakness by combining:

1. CPFA exploitation:
   - pheromone trails
   - site fidelity
   - correlated random walk
2. GCFA-style memory:
   - visited-region tracking
   - low-visit region selection
3. New improvements:
   - adaptive QuadTree regions instead of a fixed grid
   - hybrid scoring instead of least-visited-only scoring
   - adaptive mode switching between clustered and sparse behavior
   - target balancing so too many robots do not choose the same region

## 3. Algorithm Implemented

The implemented algorithm is:

Adaptive Hierarchical Clustered Foraging Algorithm, or AHCFA.

AHCFA uses a central shared memory in the loop functions. Robots still behave
mostly like CPFA robots, but when they return to the nest they can upload
visited-location samples and receive a better next target.

## 4. Main Code Changes

### 4.1 QuadTree-Based Visited Region Memory

File:

```text
source/CPFA/CPFA_loop_functions.h
source/CPFA/CPFA_loop_functions.cpp
```

Added an `AdaptiveRegion` structure. Each region stores:

```text
MinX, MaxX, MinY, MaxY
Visits
ResourceHits
ResourceWeight
Children[4]
```

The root region covers the full forage arena. When a region receives enough
visit observations, it splits into four child regions.

This gives adaptive resolution:

1. Frequently visited or important regions become smaller and more precise.
2. Sparse regions remain large and cheap to store.
3. The server does not need to store all raw visited positions.

Important functions:

```text
ResetAdaptiveRegions()
RecordVisitedLocations()
InsertAdaptiveObservation()
SplitAdaptiveRegion()
CollectAdaptiveLeaves()
```

### 4.2 Limited Robot Visit Memory

File:

```text
source/CPFA/CPFA_controller.cpp
source/CPFA/CPFA_controller.h
```

Each robot stores a small FIFO queue of visited locations while searching.
The implementation samples a robot's position every 25 seconds and keeps at
most 20 samples.

Important functions:

```text
SampleVisitedLocation()
UploadVisitedLocations()
```

When the robot returns to the nest, it uploads these samples to the loop
function. The server updates the QuadTree and the robot clears its local
queue.

This follows Arturo's GCFA idea of limited onboard memory.

### 4.3 Hybrid Region Scoring

File:

```text
source/CPFA/CPFA_loop_functions.cpp
```

AHCFA does not select regions only by least-visited count. It uses a hybrid
score:

```text
score =
  exploration term
  + pheromone term
  + resource probability term
  + site-fidelity/cluster term
  + precision bonus
```

Implemented in:

```text
ScoreAdaptiveRegion()
SelectAdaptiveSearchTarget()
```

The idea is:

1. Less-visited regions are attractive for exploration.
2. Regions near pheromones remain attractive for exploitation.
3. Regions with previous resource discoveries receive higher probability.
4. Regions near site-fidelity points remain attractive.
5. Smaller QuadTree regions get a small precision bonus.

This helps avoid GCFA's clustered-distribution problem.

### 4.4 Resource Discovery Memory

File:

```text
source/CPFA/CPFA_controller.cpp
source/CPFA/CPFA_loop_functions.cpp
```

When a robot finds food, the code estimates local resource density and records
the discovery in the adaptive memory.

Important functions:

```text
SetLocalResourceDensity()
RecordResourceDiscovery()
```

This information is used as the resource-probability term in the region score.

### 4.5 Adaptive Search Mode Switching

File:

```text
source/CPFA/CPFA_controller.cpp
source/CPFA/CPFA_loop_functions.cpp
```

AHCFA checks recent discovery density to decide whether the environment is
behaving like a clustered environment.

Important function:

```text
IsClusteredResourceMode()
```

Behavior:

1. If recent discoveries suggest clustering, robots preserve CPFA behavior more
   strongly by using pheromones and site fidelity.
2. If discoveries are sparse, robots use QuadTree-guided exploration more often.
3. Even in clustered mode, some robots still use adaptive exploration so the
   whole swarm does not crowd into one cluster.

### 4.6 Target Balancing

File:

```text
source/CPFA/CPFA_loop_functions.cpp
source/CPFA/CPFA_loop_functions.h
```

We noticed in visualization that many robots were going to the same cluster.
To reduce that crowding, AHCFA now keeps short-lived target claims.

Important function:

```text
CountAdaptiveTargetClaims()
```

When a QuadTree region has already been selected recently by other robots, its
score is temporarily penalized. This encourages other robots to spread to other
regions.

### 4.7 CPFA vs AHCFA Switch

File:

```text
source/CPFA/CPFA_loop_functions.h
source/CPFA/CPFA_loop_functions.cpp
source/CPFA/CPFA_controller.cpp
experiments/final_project/*.xml
```

Added:

```xml
UseAHCFA="1"
```

Meaning:

```text
UseAHCFA="1" -> run AHCFA
UseAHCFA="0" -> run CPFA baseline
```

This is important for fair evaluation because the assignment asks us to compare
against original CPFA.

The run script automatically creates temporary XML files with `UseAHCFA="0"`
or `UseAHCFA="1"` so both algorithms can be tested using the same seeds.

### 4.8 Result File Naming

File:

```text
source/CPFA/CPFA_loop_functions.cpp
```

Result filenames now include the algorithm name:

```text
CPFA
AHCFA
```

This avoids mixing CPFA and AHCFA results.

## 5. Experiment XML Files

The final project XML files are:

```text
experiments/final_project/Random_AHCFA_r24_tag256_10by10.xml
experiments/final_project/Powerlaw_AHCFA_r24_tag256_10by10.xml
experiments/final_project/Clustered_AHCFA_r24_tag256_10by10.xml
```

Each file uses:

```text
24 robots
256 resources
10 m x 10 m arena
720 seconds maximum simulation time
```

For clustered resources:

```text
NumberOfClusters = 4
ClusterWidthX = 8
ClusterWidthY = 8
Total resources = 4 x 8 x 8 = 256
```

Robot IDs in visualization may look like `F35`, but this does not mean robot
35. It means:

```text
F3 group, robot index 5
```

The XML uses four groups of six robots:

```text
F0 group: F00 to F05
F1 group: F10 to F15
F2 group: F20 to F25
F3 group: F30 to F35
Total: 24 robots
```

## 6. Visualization Changes

Visualization was enabled in:

```text
experiments/final_project/Clustered_AHCFA_r24_tag256_10by10.xml
```

Run clustered visualization with:

```bash
cd /home/sara/Downloads/CPFA_AHCFA_Final_Project/CPFA_Base_ARGoS-main
argos3 -c experiments/final_project/Clustered_AHCFA_r24_tag256_10by10.xml
```

Do not use `-z` when visualization is needed. The `-z` flag disables
visualization.

### Visualization Color Meaning

In the visualization:

```text
Black disks: uncollected resources
Orange disks: nearby resources highlighted during local density estimation
Yellow disks: site-fidelity locations
Green/yellow/red markers: pheromone waypoints or trails with different strengths
Green large circle: nest or collection zone
Black disks near/in the nest: collected resources
```

Orange highlighting is temporary. It shows which local food neighborhood was
detected when a robot found a resource.

## 7. How To Build

From the project folder:

```bash
cd /home/sara/Downloads/CPFA_AHCFA_Final_Project/CPFA_Base_ARGoS-main
cmake --build build
```

## 8. How To Run One Simulation Without Visualization

Use `-z` for headless mode.

Random:

```bash
argos3 -z -c experiments/final_project/Random_AHCFA_r24_tag256_10by10.xml
```

Powerlaw:

```bash
argos3 -z -c experiments/final_project/Powerlaw_AHCFA_r24_tag256_10by10.xml
```

Clustered:

```bash
argos3 -z -c experiments/final_project/Clustered_AHCFA_r24_tag256_10by10.xml
```

## 9. How To Run One Clustered Visualization

```bash
cd /home/sara/Downloads/CPFA_AHCFA_Final_Project/CPFA_Base_ARGoS-main
argos3 -c experiments/final_project/Clustered_AHCFA_r24_tag256_10by10.xml
```

## 10. How To Run The Full Evaluation

The full evaluation compares CPFA and AHCFA.

Run:

```bash
cd /home/sara/Downloads/CPFA_AHCFA_Final_Project/CPFA_Base_ARGoS-main
cmake --build build
bash scripts/run_final_project_50_trials.sh
python3 scripts/plot_final_project_results.py
```

This runs:

```text
3 distributions x 2 algorithms x 50 seeds = 300 simulations
```

The script uses the same seeds for CPFA and AHCFA. This gives a fair paired
comparison.

## 11. Evaluation Output Files

Results are stored in:

```text
results/final_project/
```

Important files:

```text
final_project_cpfa_ahcfa_50runs.csv
Random_CPFA_50runs.csv
Random_AHCFA_50runs.csv
Powerlaw_CPFA_50runs.csv
Powerlaw_AHCFA_50runs.csv
Clustered_CPFA_50runs.csv
Clustered_AHCFA_50runs.csv
ahcfa_vs_cpfa_improvement.csv
cpfa_vs_ahcfa_collected_resources_boxplot.png
cpfa_vs_ahcfa_time_boxplot.png
```

The most important file for the report is:

```text
ahcfa_vs_cpfa_improvement.csv
```

It contains the percent improvement in collected resources.

## 12. Improvement Formula

The assignment asks for percentage improvement in collected resources.

Use:

```text
Improvement (%) =
((Mean AHCFA collected resources - Mean CPFA collected resources)
 / Mean CPFA collected resources) x 100
```

The metric should be calculated separately for:

```text
Random
Powerlaw
Clustered
```

## 13. How To Explain AHCFA In The Report

You can describe AHCFA like this:

AHCFA improves CPFA by adding adaptive spatial memory to the central server.
Robots upload a small number of sampled visited locations when they return to
the nest. The server organizes those samples using a QuadTree instead of a
fixed grid. It then scores candidate regions using both exploration and
exploitation information: visit count, pheromone strength, resource discovery
probability, and site-fidelity evidence. The algorithm switches behavior based
on recent discovery density: if resources appear clustered, it preserves CPFA's
pheromone and site-fidelity behavior; if resources appear sparse, it increases
QuadTree-guided exploration.

## 14. How To Explain Improvement Over GCFA In Clustered Resources

Arturo's GCFA paper shows that GCFA can perform worse in clustered
distributions. The reason is that GCFA's least-visited-region strategy may send
robots away from known productive clusters.

AHCFA addresses this by:

1. Keeping pheromone and site fidelity from CPFA.
2. Recording resource probability in QuadTree regions.
3. Using hybrid scoring instead of least-visited-only scoring.
4. Switching toward CPFA exploitation when recent discoveries indicate a
   clustered environment.
5. Keeping some adaptive exploration so robots can still discover other
   clusters.
6. Penalizing regions that already have many recent robot target claims.

This gives a balanced strategy:

```text
CPFA strength: exploit known clusters
GCFA strength: avoid repeated visits to empty areas
AHCFA goal: combine both and adapt between them
```

## 15. Verification Already Performed

The project has been built successfully with:

```bash
cmake --build build
```

Headless smoke tests were run for:

```text
Random
Powerlaw
Clustered
```

The CPFA/AHCFA switch was also sanity checked on one clustered seed:

```text
CPFA seed 1: 76 collected
AHCFA seed 1: 85 collected
```

This one seed is only a smoke test. The final proof should use the full 50-run
evaluation.

## 16. Final Report Checklist

For the final report, include:

1. Problem statement from Project 3.
2. Weakness of CPFA:
   - robots over-follow pheromone/site-fidelity locations
   - some regions remain underexplored
3. Weakness of GCFA in Arturo's clustered results:
   - fixed grid
   - least-visited-only selection
   - poor clustered performance
4. AHCFA method:
   - adaptive QuadTree
   - limited robot visit memory
   - hybrid scoring
   - adaptive switching
   - target balancing
5. Experimental setup:
   - 10 m x 10 m arena
   - 24 robots
   - 256 resources
   - 12 minutes
   - 50 runs per distribution
6. Results:
   - box plots for collected resources
   - percentage improvement table
   - discussion by distribution
7. Conclusion:
   - AHCFA is designed to retain CPFA's cluster exploitation while adding
     efficient memory-guided exploration.
