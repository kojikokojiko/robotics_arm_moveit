#!/bin/bash
# /joint_statesトピックにパブリッシュしているノードを確認するスクリプト

echo "=== Checking /joint_states topic publishers ==="
echo ""
echo "Topic info:"
ros2 topic info /joint_states
echo ""
echo "Active nodes:"
ros2 node list
echo ""
echo "To stop joint_state_publisher:"
echo "  ros2 node kill /joint_state_publisher"
echo ""
echo "To monitor joint states:"
echo "  ros2 topic echo /joint_states"
