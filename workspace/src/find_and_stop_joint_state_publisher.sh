#!/bin/bash
# joint_state_publisherノードを見つけて停止するスクリプト

echo "=== Finding joint_state_publisher process ==="

# プロセスを検索
JOINT_STATE_PID=$(ps aux | grep '[j]oint_state_publisher' | awk '{print $2}')

if [ -z "$JOINT_STATE_PID" ]; then
    echo "joint_state_publisher process not found"
    echo ""
    echo "Checking active nodes:"
    ros2 node list
    echo ""
    echo "Checking /joint_states publishers:"
    ros2 topic info /joint_states
else
    echo "Found joint_state_publisher process: PID=$JOINT_STATE_PID"
    echo "Killing process..."
    kill $JOINT_STATE_PID
    sleep 1
    echo "Done!"
    echo ""
    echo "Verifying:"
    ros2 topic info /joint_states
fi
