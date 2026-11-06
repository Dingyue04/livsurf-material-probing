#!/bin/bash

# Clone Franka dependencies into the workspace
vcs import /ros2_ws/src < /ros2_ws/src/dependency.repos --recursive --skip-existing

# Apply manage_overruns patch to hardware_interface (not yet upstream)
if [ -d /ros2_ws/src/ros2_control/hardware_interface ]; then
  git -C /ros2_ws/src/ros2_control apply /ros2_ws/src/patches/manage_overruns.patch --verbose 2>&1 || \
    echo "Warning: manage_overruns patch may already be applied or failed to apply"
fi

# Add the Pilz LIN planning pipeline used by probing contact stages.
if [ -d /ros2_ws/src/franka_fr3_moveit_config ]; then
  fr3_config=/ros2_ws/src/franka_fr3_moveit_config
  pilz_patch=/ros2_ws/src/patches/fr3_pilz_lin.patch
  if patch --directory="$fr3_config" --strip=1 --forward --batch --dry-run \
      < "$pilz_patch" >/dev/null 2>&1; then
    patch --directory="$fr3_config" --strip=1 --forward --batch \
      < "$pilz_patch"
  elif patch --directory="$fr3_config" --strip=1 --reverse --batch --dry-run \
      < "$pilz_patch" >/dev/null 2>&1; then
    echo "FR3 Pilz LIN patch is already applied"
  else
    echo "Warning: FR3 Pilz LIN patch does not match this source version"
  fi
fi

exec "$@"
