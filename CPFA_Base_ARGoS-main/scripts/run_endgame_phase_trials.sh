#!/usr/bin/env bash
set -euo pipefail

TRIALS="${TRIALS:-50}"
MAX_TIME_SECONDS="${MAX_TIME_SECONDS:-3600}"

mkdir -p results/final_project
rm -f results/final_project/collection_milestones.csv

combined="results/final_project/endgame_phase_trials.csv"
echo "algorithm,distribution,score,time_seconds,seed" > "$combined"

for dist in Random Powerlaw Clustered; do
  cfg="experiments/final_project/${dist}_AHCFA_r24_tag256_10by10.xml"

  for algorithm in CPFA AHCFA; do
    for seed in $(seq 1 "$TRIALS"); do
      tmp_cfg="$(mktemp "/tmp/${dist}_${algorithm}_endgame_${seed}.XXXXXX.xml")"
      use_ahcfa=0
      if [ "$algorithm" = "AHCFA" ]; then
        use_ahcfa=1
      fi

      sed \
        -e "s/random_seed=\"[0-9]*\"/random_seed=\"${seed}\"/" \
        -e "s/UseAHCFA=\"1\"/UseAHCFA=\"${use_ahcfa}\"/" \
        -e "s/MaxSimTimeInSeconds=\"[0-9]*\"/MaxSimTimeInSeconds=\"${MAX_TIME_SECONDS}\"/" \
        "$cfg" > "$tmp_cfg"

      echo "Running endgame ${algorithm} ${dist} seed ${seed}"
      result_line="$(argos3 -n -z -c "$tmp_cfg" 2>/dev/null | tail -n 1 | tr -d '\r')"
      rm -f "$tmp_cfg"

      echo "${algorithm},${dist},${result_line}" >> "$combined"
    done
  done
done

python3 scripts/make_endgame_phase_table.py
