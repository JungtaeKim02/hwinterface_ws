#!/usr/bin/env bash
set -eo pipefail

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
  echo "Usage: $0 [bag-directory]"
  echo "Start the drive test launch with uart:=true before recording. Stop recording with Ctrl+C."
  exit 0
fi
if (( $# > 1 )); then
  echo "Usage: $0 [bag-directory]" >&2
  exit 2
fi

workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"

bag_directory=${1:-"$workspace/bags/drive_test_$(date +%Y%m%d_%H%M%S)"}
if [[ -e "$bag_directory" ]]; then
  echo "Bag output already exists: $bag_directory" >&2
  exit 1
fi

manager=/uart/controller_manager
feedback_topic=/uart/wheel_feedback/joint_states
feedback_info=$(ros2 topic info "$feedback_topic" --no-daemon 2>/dev/null || true)
if ! grep -Eq '^Publisher count: [1-9][0-9]*$' <<< "$feedback_info"; then
  controller_params=$(mktemp -t robocup-wheel-feedback.XXXXXX.yaml)
  trap 'rm -f -- "$controller_params"' EXIT
  cat > "$controller_params" <<'YAML'
/uart/wheel_feedback:
  ros__parameters:
    type: joint_state_broadcaster/JointStateBroadcaster
    use_local_topics: true
    joints: [wheel_0_joint, wheel_1_joint, wheel_2_joint]
    interfaces: [position, velocity]
YAML
  if ! ros2 run controller_manager spawner wheel_feedback \
    -c "$manager" --controller-manager-timeout 15 --param-file "$controller_params" \
    --ros-args -r __ns:=/uart; then
    echo "WARNING: Live wheel feedback is unavailable. Recording available drive topics for diagnostics." >&2
    echo "Check /uart/drive_status and the UART controller manager log before driving." >&2
  fi
  rm -f -- "$controller_params"
  trap - EXIT
fi

mkdir -p -- "$(dirname -- "$bag_directory")"
echo "Recording drive test to $bag_directory (Ctrl+C to finish)"
exec ros2 bag record --storage mcap --output "$bag_directory" --topics \
  /joy \
  /omni_drive/cmd_vel \
  /sim/wheel_commands \
  "$feedback_topic" \
  /uart/drive_status \
  /uart/can_preview \
  /joint_states \
  /omni_drive/odom \
  /clock \
  /tf \
  /tf_static
