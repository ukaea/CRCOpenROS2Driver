#!/usr/bin/env bash

# resolve repo root (script lives at repo root)
REPO_ROOT="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

# rosdeps
sudo rosdep init
rosdep update
rosdep install --from-paths "$REPO_ROOT" --ignore-src -r -y

# ORL driver (exact filename; change if yours differs)
ORL_DEB="$REPO_ROOT/crcopen_hardware/lib/orl_driver-4.41.5.31647-Linux.deb"
sudo apt-get install -y "$ORL_DEB"