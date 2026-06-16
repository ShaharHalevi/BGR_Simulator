#!/usr/bin/env bash
set -e

rm -rf build/ install/ log/
colcon build
source install/setup.bash
colcon test --packages-select bgr_description --event-handlers console_direct+
colcon test-result --all --verbose
