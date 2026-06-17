#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
mkdir -p vendor

sync_repo() {
  local url="$1"
  local dir="$2"
  shift 2

  if [ -d "$dir/.git" ]; then
    echo "Updating $dir"
    git -C "$dir" fetch --depth 1 origin
    git -C "$dir" checkout -q FETCH_HEAD
  else
    echo "Cloning $url -> $dir"
    git clone --depth 1 --filter=blob:none --sparse --no-checkout "$url" "$dir"
  fi

  if [ "$#" -gt 0 ]; then
    git -C "$dir" sparse-checkout set "$@"
  fi
  git -C "$dir" checkout -q
}

UNITREE_SPARSE_PATHS="${UNITREE_SPARSE_PATHS:-robots/g1_description}"
sync_repo "https://github.com/unitreerobotics/unitree_ros.git" "vendor/unitree_ros" $UNITREE_SPARSE_PATHS
sync_repo "https://github.com/ROBOTIS-GIT/ai_worker.git" "vendor/ai_worker" ffw_description

echo
echo "URDF targets:"
ls -lh \
  vendor/unitree_ros/robots/g1_description/g1_29dof.urdf \
  vendor/ai_worker/ffw_description/urdf/ffw_bg2_rev4_follower/ffw_bg2_follower.urdf
