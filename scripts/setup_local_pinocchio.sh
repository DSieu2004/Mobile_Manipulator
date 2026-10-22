#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
debs="$project_root/third_party/debs"
prefix="$project_root/third_party/ros/opt/ros/humble"
mkdir -p "$debs" "$prefix"

(
    cd "$debs"
    apt-get download ros-humble-eigenpy ros-humble-coal ros-humble-pinocchio
)
for package in "$debs"/*.deb; do
    dpkg-deb -x "$package" "$project_root/third_party/ros"
done

# ROS debs contain a few absolute paths for their Python extension targets.
# Rebase those targets to the local prefix used by the OCS2 build task.
while IFS= read -r -d '' target; do
    sed -i "s|/opt/ros/humble|$prefix|g" "$target"
done < <(find "$prefix/lib/x86_64-linux-gnu/cmake" -name '*Targets-none.cmake' -print0)

touch "$project_root/third_party/COLCON_IGNORE"
echo "Pinocchio, Coal and EigenPy are ready in $prefix"
