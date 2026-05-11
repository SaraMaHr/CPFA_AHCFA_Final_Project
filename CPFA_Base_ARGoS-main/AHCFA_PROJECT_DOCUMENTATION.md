# Adaptive Hierarchical Clustered Foraging Algorithm Documentation

This document describes the current 12-minute final-project implementation in
`CPFA_Base_ARGoS-main`.

## 1. Project Setup

The project evaluates an improved CPFA-style foraging controller against the
original CPFA baseline.

Required experiment structure:

- Arena: 10 m x 10 m
- Resources: 256
- Robots: 24
- Runtime: 12 minutes, or 720 seconds
- Runs: 50 per distribution
- Distributions: Random, Powerlaw, Clustered
- Metric: collected resources after 12 minutes

The current XML files use `NestRadius="0.25"`. The runtime code no longer
scales the nest radius, so the XML value is the real simulation value.

## 2. Motivation From Arturo's GCFA

Arturo's GCFA paper improves final-stage collection by recording visited regions
in a grid and sending robots toward less-visited regions. This works well for
Random and Powerlaw endgame collection, but the paper shows a weakness in
Clustered resources: pure least-visited-region selection can pull robots away
from productive pheromone/site-fidelity locations.

AHCFA is designed as a hybrid:

- Keep CPFA exploitation: pheromones, site fidelity, correlated random walk.
- Add GCFA-style spatial memory: limited visited-location reports.
- Use adaptive QuadTree regions instead of a fixed grid.
- Score regions with both exploration and exploitation information.
- Preserve CPFA behavior when exploration hurts, especially clustered resource
  exploitation, while adding stronger Random-specific coverage late in the run.

Important note: Arturo's reported `48%` and `35%` improvements are from
endgame phase time, especially `90% -> 100%`, not from 12-minute collected
resources. The class-project script reports collected resources at 720 seconds.

## 3. Current Algorithm

The implemented algorithm is AHCFA: Adaptive Hierarchical Clustered Foraging
Algorithm.

Robots still use the CPFA finite-state machine:

- `DEPARTING`
- `SEARCHING`
- `SURVEYING`
- `RETURNING`

AHCFA adds shared spatial memory in the loop functions and uses it only when it
is expected to help. The current implementation is distribution-aware: Random
uses stronger adaptive coverage because it has weak pheromone/site-fidelity
gradients, while Powerlaw and Clustered keep more CPFA exploitation.

## 4. Main Implementation Details

### QuadTree Spatial Memory

Files:

```text
source/CPFA/CPFA_loop_functions.h
source/CPFA/CPFA_loop_functions.cpp
```

The loop functions maintain adaptive regions. Each region stores:

```text
MinX, MaxX, MinY, MaxY
Visits
ResourceHits
ResourceWeight
Children[4]
```

Important functions:

```text
ResetAdaptiveRegions()
RecordVisitedLocations()
InsertAdaptiveObservation()
SplitAdaptiveRegion()
CollectAdaptiveLeaves()
```

The server stores compact region counters instead of every raw robot location.

### Limited Robot Visit Memory

Files:

```text
source/CPFA/CPFA_controller.h
source/CPFA/CPFA_controller.cpp
```

Each robot samples its position every 25 seconds while searching and keeps at
most 20 samples. When it returns to the nest, it uploads those samples to the
loop functions.

Important functions:

```text
SampleVisitedLocation()
UploadVisitedLocations()
```

### Hybrid Region Scoring

File:

```text
source/CPFA/CPFA_loop_functions.cpp
```

AHCFA scores candidate regions using:

```text
score =
  exploration term
  + pheromone term
  + resource discovery term
  + site-fidelity term
  + precision bonus
```

Important functions:

```text
ScoreAdaptiveRegion()
SelectAdaptiveSearchTarget()
```

Current tuning:

- Exploration is low early and stronger late.
- Random doubles the exploration phase multiplier in region scoring because
  systematic coverage is the main useful signal for that distribution.
- Pheromone/resource/site-fidelity terms are weighted more strongly than in the
  first AHCFA draft.
- Resource weight is divided by `sqrt(1 + Visits)` instead of `1 + Visits`, so
  productive Powerlaw/Clustered areas are not punished too aggressively.
- In clustered mode, exploration is reduced and pheromone/site-fidelity evidence
  is weighted more strongly.

### Clustered Mode

File:

```text
source/CPFA/CPFA_loop_functions.cpp
```

`IsClusteredResourceMode()` watches recent resource discoveries. When recent
discoveries are dense, AHCFA behaves more like CPFA so robots do not abandon
productive clusters.

### Adaptive Local Sweep

Files:

```text
source/CPFA/CPFA_controller.h
source/CPFA/CPFA_controller.cpp
```

AHCFA includes a GCFA-style local sweep target list:

```text
AddAdaptiveSweepTargets()
ContinueAdaptiveSweep()
```

The sweep creates nine local targets around an adaptive target. In the current
12-minute tuning, the sweep is used conservatively and is disabled in clustered
mode. Random starts the sweep after 300 seconds because coverage is useful
earlier there; Powerlaw and Clustered use the later 480-second start.

### Distribution-Aware Adaptive Targeting

File:

```text
source/CPFA/CPFA_controller.cpp
```

After a robot returns to the nest, AHCFA chooses between CPFA targets and
adaptive QuadTree targets using distribution-specific probabilities.

Current AHCFA probabilities:

```text
Random:
  before 4 minutes: 0.04
  4 to 8 minutes:  0.15
  after 8 minutes: 0.40

Powerlaw/Clustered default:
  before 4 minutes: 0.00
  4 to 8 minutes:  0.05
  after 8 minutes: 0.25
```

If no CPFA pheromone or site-fidelity target is selected, Random can fall back
to adaptive exploration with up to `0.70` probability. Clustered mode caps the
adaptive probability at `0.05` so robots keep exploiting productive clusters.

### Opportunistic Pickup While Returning

File:

```text
source/CPFA/CPFA_controller.cpp
```

AHCFA checks for food while returning to the nest in all three distributions.
If a robot passes over a resource on the way back, it can collect it instead of
ignoring it. This gives AHCFA an extra collection opportunity without changing
the CPFA baseline behavior.

### CPFA/AHCFA Switch

File:

```text
source/CPFA/CPFA_loop_functions.cpp
experiments/final_project/*.xml
scripts/run_final_project_50_trials.sh
```

The XML setting is:

```xml
UseAHCFA="1"
```

Meaning:

```text
UseAHCFA="1" -> AHCFA
UseAHCFA="0" -> CPFA baseline
```

The run script creates temporary XML files so CPFA and AHCFA run with the same
seeds.

## 5. Experiment XML Files

Current final-project XML files:

```text
experiments/final_project/Random_AHCFA_r24_tag256_10by10.xml
experiments/final_project/Powerlaw_AHCFA_r24_tag256_10by10.xml
experiments/final_project/Clustered_AHCFA_r24_tag256_10by10.xml
```

Current key settings:

```text
Arena size: 10 m x 10 m
Robots: 24
Resources: 256
MaxSimTimeInSeconds: 720
NestRadius: 0.25
```

Clustered resources:

```text
NumberOfClusters = 4
ClusterWidthX = 8
ClusterWidthY = 8
Total resources = 4 x 8 x 8 = 256
```

The XML places four groups of six robots:

```text
F0 group: 6 robots
F1 group: 6 robots
F2 group: 6 robots
F3 group: 6 robots
Total: 24 robots
```

## 6. Build And Run

From the project folder:

```bash
cd /home/sara/Documents/CPFA_AHCFA_Final_Project/CPFA_Base_ARGoS-main
export LD_LIBRARY_PATH="$PWD/build/source/Base:$PWD/build/source/CPFA:$LD_LIBRARY_PATH"
cmake --build build
```

Run one headless simulation:

```bash
argos3 -n -z -c experiments/final_project/Random_AHCFA_r24_tag256_10by10.xml
```

Run the full 12-minute evaluation:

```bash
bash scripts/run_final_project_50_trials.sh
python3 scripts/plot_final_project_results.py
cat results/final_project/ahcfa_vs_cpfa_improvement.csv
```

Create an Arturo-style 12 x 12 visit-count heatmap after at least one run:

```bash
python3 scripts/plot_visit_heatmap.py --distribution Random --algorithm AHCFA
```

For one specific seed:

```bash
python3 scripts/plot_visit_heatmap.py --distribution Random --algorithm AHCFA --seed 263532
```

Create the actual AHCFA QuadTree map, with mixed-size adaptive leaf regions:

```bash
python3 scripts/plot_adaptive_quadtree_map.py --distribution Clustered --algorithm AHCFA --metric visits --arena-size 10
python3 scripts/plot_adaptive_quadtree_map.py --distribution Clustered --algorithm AHCFA --metric resource_hits --arena-size 10
```

The fixed-grid heatmap is for comparison with Arturo's GCFA figure. The
QuadTree map is the real AHCFA decision memory, so it shows large cells in
low-information areas and smaller cells where the algorithm has observed more
activity. The plot frame covers the full 10 m x 10 m arena; the dashed inner
border marks the safe foraging range used by the robot controller near walls.

This runs:

```text
3 distributions x 2 algorithms x 50 seeds = 300 simulations
```

## 7. Output Files

Main result folder:

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
score_summary_table.csv
ahcfa_vs_cpfa_improvement.csv
cpfa_vs_ahcfa_collected_resources_boxplot.png
cpfa_vs_ahcfa_mean_score_bar.png
ahcfa_percent_improvement_bar.png
visit_heatmap_cells.csv
visit_heatmap_Random_AHCFA_mean.png
adaptive_quadtree_regions.csv
adaptive_quadtree_Clustered_AHCFA_seed980913_visits.png
adaptive_quadtree_Clustered_AHCFA_seed980913_resource_hits.png
```

The most important report file is:

```text
results/final_project/ahcfa_vs_cpfa_improvement.csv
```

## 8. Improvement Formula

The class project asks for percentage improvement in collected resources:

```text
Improvement (%) =
((Mean AHCFA collected resources - Mean CPFA collected resources)
 / Mean CPFA collected resources) x 100
```

This is calculated separately for Random, Powerlaw, and Clustered.

## 9. Latest Completed 50-Run Result

Latest stored 50-run result in
`results/final_project/ahcfa_vs_cpfa_improvement.csv`:

```text
Random:
  CPFA mean  = 68.26
  AHCFA mean = 70.70
  Improvement = +3.57%
  Wilcoxon p = 0.275

Powerlaw:
  CPFA mean  = 81.26
  AHCFA mean = 92.00
  Improvement = +13.22%
  Wilcoxon p = 0.0028

Clustered:
  CPFA mean  = 70.08
  AHCFA mean = 74.40
  Improvement = +6.16%
  Wilcoxon p = 0.000515
```

Interpretation:

- Random now improves in mean score after adding Random-specific adaptive
  coverage, earlier sweep timing, and returning-trip pickup. Its p-value is
  still above `0.001`, so it should be reported as a positive but not
  Arturo-level statistically significant result.
- Powerlaw improves because productive regions are protected and returning
  robots can opportunistically collect encountered resources.
- Clustered improves because CPFA exploitation is preserved instead of using
  pure least-visited-region behavior like GCFA.

If the nest radius is changed again, rerun the full 50-run script and replace
these numbers with the new CSV values because the baseline changes too.

## 10. Endgame Analysis

Arturo's GCFA paper reports final-stage time improvements. To generate a
similar table, use the endgame script:

```bash
TRIALS=50 MAX_TIME_SECONDS=3600 bash scripts/run_endgame_phase_trials.sh
cat results/final_project/endgame_phase_table.md
```

This is a different metric from the 12-minute collected-resource score.

## 11. Report Explanation

Suggested short report description:

AHCFA extends CPFA with adaptive spatial memory at the central server. Robots
upload a bounded set of sampled visited locations when they return to the nest.
The server stores these observations in adaptive QuadTree regions and scores
candidate regions using both exploration and exploitation terms: visit count,
pheromone strength, resource discovery weight, and site-fidelity evidence. The
algorithm uses stronger directed coverage for Random, but remains conservative
in clustered conditions so it preserves CPFA's ability to exploit productive
resource areas while still adding directed exploration when resources become
harder to find.

## 12. Final Report Checklist

Include:

1. Project requirement summary.
2. CPFA baseline behavior and weakness.
3. GCFA motivation and its clustered-distribution weakness.
4. AHCFA method:
   - limited robot memory
   - QuadTree region memory
   - hybrid scoring
   - clustered-mode protection
   - distribution-aware adaptive target probabilities
   - conservative local sweep
   - opportunistic pickup while returning
5. Experimental setup:
   - 10 m x 10 m arena
   - 24 robots
   - 256 resources
   - 720 seconds
   - 50 runs per distribution
   - current nest radius used in XML
6. CPFA vs AHCFA results.
7. Box plots and percent improvement table.
8. Honest note that Arturo's endgame table uses a different metric from the
   12-minute class-project score.
