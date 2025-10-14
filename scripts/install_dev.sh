#!/usr/bin/env bash
set -euo pipefail

# Optional: install ORL if present in the repo (relative path)
DEB_GLOB="/ros2_ws/src/comau-crcopen-ros2-driver/crcopen_hardware/lib/orl_driver-*.deb"
if compgen -G "${DEB_GLOB}" > /dev/null; then
  echo "[install_dev] Installing ORL from: $(ls ${DEB_GLOB})"
  apt-get update
  apt-get install -y ${DEB_GLOB}
else
  echo "[install_dev] ORL .deb not found under crcopen_hardware/lib/ (skipping)"
fi

# rosdep for the mounted workspace
rosdep init || true
rosdep update || true
rosdep install --from-paths /ros2_ws/src --ignore-src -r -y
echo "[install_dev] Done."
