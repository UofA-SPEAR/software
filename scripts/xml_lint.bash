#!/usr/bin/env bash
set -e

file=spear_simulator/launch/diffdrive.launch

for file in pkg/spear_*/launch/*.launch; do
    echo "Linting $file"
    diff <(cat $file) <(tail -n +2 <(xmllint $file --format)) -B
done
