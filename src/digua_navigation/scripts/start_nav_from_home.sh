#!/usr/bin/env bash
set -e

POSE_NAME="home"
SESSION="digua_nav"
FORCE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pose)
      POSE_NAME="$2"
      shift 2
      ;;
    --session)
      SESSION="$2"
      shift 2
      ;;
    --force)
      FORCE=1
      shift
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Usage: ros2 run digua_navigation start_nav_from_home.sh [--pose home] [--session digua_nav] [--force]"
      exit 1
      ;;
  esac
done

if [[ "$FORCE" == "1" ]]; then
  tmux kill-session -t "$SESSION" 2>/dev/null || true
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' already exists."
  echo "Attach with: tmux attach -t $SESSION"
  echo "Or restart with: ros2 run digua_navigation start_nav_from_home.sh --force"
  exit 1
fi

LOG_DIR="$HOME/digua_ws/digua_navigation_data/logs/nav_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

COMMON="source /opt/ros/humble/setup.bash; source $HOME/digua_ws/install/setup.bash; export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-23}; export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0};"

echo "Starting navigation tmux session: $SESSION"
echo "Initial pose: $POSE_NAME"
echo "Logs: $LOG_DIR"

tmux new-session -d -s "$SESSION" -n localization \
  "bash -lc '$COMMON ros2 launch digua_navigation localization.launch.py 2>&1 | tee $LOG_DIR/localization.log'"

tmux new-window -t "$SESSION" -n init_pose \
  "bash -lc '$COMMON sleep 5; ros2 run digua_navigation auto_initial_pose.py $POSE_NAME --repeat 20; ros2 run digua_navigation wait_for_tf.py map odom --timeout 60; echo \"Initial pose done. map->odom is ready.\"; exec bash'"

tmux new-window -t "$SESSION" -n navigation \
  "bash -lc '$COMMON ros2 run digua_navigation wait_for_tf.py map odom --timeout 90 && ros2 launch digua_navigation navigation.launch.py 2>&1 | tee $LOG_DIR/navigation.log; exec bash'"

tmux new-window -t "$SESSION" -n command \
  "bash -lc '$COMMON echo \"Commands ready.\"; echo \"Try:\"; echo \"  ros2 run digua_navigation follow_named_route.py home test_front home --timeout-per-goal 180\"; echo \"  ros2 run digua_navigation go_to_named_pose.py test_front --timeout 180\"; exec bash'"

tmux select-window -t "$SESSION:localization"
tmux attach -t "$SESSION"
