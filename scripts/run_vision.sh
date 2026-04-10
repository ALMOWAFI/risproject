#!/bin/bash
# Load params and run the vision node.
# Usage: bash run_vision.sh
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
YAML="$SCRIPT_DIR/../config/game_params.yaml"

echo "[vision] Loading params from $YAML"
rosparam load "$YAML"

echo "[vision] Starting vision_node"
rosrun memory_game vision_node __name:=vision_node
