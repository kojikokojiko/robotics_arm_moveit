#!/bin/bash
# joint_state_publisherを停止するスクリプト

echo "Stopping joint_state_publisher node..."
ros2 node kill /joint_state_publisher 2>/dev/null || echo "joint_state_publisher node not found or already stopped"
echo "Done. You can now run move_arm_simple.py"
