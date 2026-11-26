#!/usr/bin/env bash
# Start the RKNN inspection vision node through the installed ROS 2 workspace.
# 通过已安装的 ROS 2 工作空间启动 RKNN 巡检视觉节点。
set -euo pipefail
: "${IIR_ROS_SETUP:?}" "${IIR_ROS_WS_SETUP:?}"
set +u
source "$IIR_ROS_SETUP"
source "$IIR_ROS_WS_SETUP"
set -u
exec ros2 launch iir_vision_rknn inspection_vision.launch.py \
    backend:="${IIR_RKNN_BACKEND:-auto}" \
    model_file:="${IIR_RKNN_MODEL:-}" \
    labels_file:="${IIR_RKNN_LABELS:-}" \
    capture_dir:="${IIR_CAPTURE_DIR:-$HOME/.cache/iir/captures}"
