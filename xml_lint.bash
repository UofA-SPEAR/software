#
set -e

file=spear_simulator/launch/diffdrive.launch

for file in spear_*/launch/*.launch; do
    echo "Linting $file"
    diff <(cat $file) <(xmllint $file --format) -B
done
