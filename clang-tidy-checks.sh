#!/bin/bash
clang_tidy="clang-tidy-14"
files=$(find include example -type f \( -name "go1_read_motors.cpp" -o -name "*.hpp" \))

clang-format --dry-run -Werror $files
$clang_tidy -p=build/ --config-file=clang-tidy-checks.txt $files
