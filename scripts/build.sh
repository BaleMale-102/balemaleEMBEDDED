#!/bin/bash
# build.sh - 빌드
# Usage: ./build.sh [package_name]

cd ~/balemaleEMBEDDED

if [ -z "$1" ]; then
    echo "Building all packages..."
    colcon build
else
    echo "Building: $@"
    colcon build --packages-select "$@"
fi

echo ""
echo "Done. Run: source install/setup.bash"
