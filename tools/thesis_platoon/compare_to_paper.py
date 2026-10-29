#!/usr/bin/env python3
"""Compare thesis_run's summary.csv files with the thesis' published results.

paper_results.csv holds the numbers from the NAFIPS 2024 paper and slides
(Tables 1-2 and the appendix charts): total gap error, total time lost
(converted from the slides' hours) and minimum TTC for each driving model in
the four scenarios.

    thesis_run --scenario 3 --out results/scenario3     (repeat for 1, 2, 4)
    python3 tools/thesis_platoon/compare_to_paper.py results
"""
import csv
import os
import sys

PAPER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "paper_results.csv")
COLUMNS = ("total_gap_error_s", "total_time_lost_s", "min_ttc_s")


def main(results_dir):
    with open(PAPER, newline="") as f:
        paper = {(r["scenario"], r["model"]): r for r in csv.DictReader(f)}

    matched = total = 0
    print(f"{'scenario':>8} {'model':<7} {'metric':<18} {'paper':>12} {'this run':>12}  verdict")
    for scenario in "1234":
        path = os.path.join(results_dir, f"scenario{scenario}", "summary.csv")
        if not os.path.exists(path):
            continue
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                expected = paper[(scenario, row["model"])]
                for column in COLUMNS:
                    want, got = float(expected[column]), float(row[column])
                    total += 1
                    # the paper rounds to 2 decimals (the slides' hours to about 1 s)
                    ok = abs(want - got) <= (1.0 if column == "total_time_lost_s" else 0.011)
                    matched += ok
                    print(f"{scenario:>8} {row['model']:<7} {column:<18} {want:>12.2f} {got:>12.2f}  {'match' if ok else 'DIFFERS'}")
    print(f"\n{matched} of {total} published numbers reproduced")
    return 0 if total else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "thesis_results"))
