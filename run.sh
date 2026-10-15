#!/bin/bash

echo "--- Compiling ---"
make clean
make || { echo "Compilation failed"; exit 1; }

TEST_CASES=(
    "case/case1.txt"
    "case/case2.txt"
    "case/case3.txt"
    "case/case4.txt"
    "case/case5.txt"
    "case/bonus.txt"
)

for input_file in "${TEST_CASES[@]}"; do
    base_name="${input_file%.txt}"
    output_file="${base_name}_out.txt"
    csv_file="${base_name}_out.csv"

    echo "--- Running $input_file ---"
    
    time ./Lab3 "$input_file" "$output_file"
    
    python3 convert_submission_to_csv.py "$output_file" "$csv_file"
    python3 plotter.py "$output_file"
    
    echo "Finished: $csv_file"
done

echo "--- All tasks completed ---"