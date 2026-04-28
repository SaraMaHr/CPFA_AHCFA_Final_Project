#!/usr/bin/env bash
set -euo pipefail

mkdir -p results/final_project

combined="results/final_project/final_project_cpfa_ahcfa_50runs.csv"
echo "algorithm,distribution,score,time_seconds,seed" > "$combined"

for dist in Random Powerlaw Clustered; do
  cfg="experiments/final_project/${dist}_AHCFA_r24_tag256_10by10.xml"

  for algorithm in CPFA AHCFA; do
    out="results/final_project/${dist}_${algorithm}_50runs.csv"
    echo "score,time_seconds,seed" > "$out"

    for seed in $(seq 1 50); do
      tmp_cfg="$(mktemp "/tmp/${dist}_${algorithm}_${seed}.XXXXXX.xml")"

      sed \
        -e "s/random_seed=\"[0-9]*\"/random_seed=\"${seed}\"/" \
        -e "s/UseAHCFA=\"1\"/UseAHCFA=\"$([ "$algorithm" = "AHCFA" ] && echo 1 || echo 0)\"/" \
        "$cfg" > "$tmp_cfg"

      echo "Running ${algorithm} ${dist} seed ${seed}"
      result_line="$(argos3 -n -z -c "$tmp_cfg" 2>/dev/null | tail -n 1 | tr -d '\r')"
      rm -f "$tmp_cfg"

      echo "$result_line" >> "$out"
      echo "${algorithm},${dist},${result_line}" >> "$combined"
    done
  done
done
