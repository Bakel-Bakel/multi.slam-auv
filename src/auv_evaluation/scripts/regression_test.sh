#!/usr/bin/env bash
# Deterministic rosbag regression (spec §12): replay a canonical mission, log the
# estimate + ground truth, and assert ATE does not regress beyond a threshold.
#
# Usage: regression_test.sh <bag_path> <ate_threshold_m> [estimate_topic] [estimate_type]
set -euo pipefail

BAG="${1:?usage: regression_test.sh <bag> <ate_threshold> [topic] [type]}"
THRESH="${2:?provide ATE threshold in metres}"
EST_TOPIC="${3:-/slam/pose}"
EST_TYPE="${4:-pose_cov}"
OUT="$(mktemp -d)"

echo "[regression] logging to ${OUT}"
ros2 launch auv_evaluation evaluate.launch.py \
  estimate_topic:="${EST_TOPIC}" estimate_type:="${EST_TYPE}" output_dir:="${OUT}" &
LOG_PID=$!
sleep 2

echo "[regression] replaying ${BAG}"
ros2 bag play "${BAG}" --clock
sleep 2
kill "${LOG_PID}" 2>/dev/null || true

echo "[regression] evaluating"
ros2 run auv_evaluation run_evo "${OUT}/estimate.tum" "${OUT}/ground_truth.tum" \
  --ate-threshold "${THRESH}"
