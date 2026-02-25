#!/usr/bin/env python3
"""
MoveIt の高度デモ: MoveGroup アクションで関節目標を送り、計画された軌道を取得して
/joint_states で再生する。run_named_poses.py から利用される。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import math


class MoveItAdvancedDemo(Node):
    def __init__(self):
        super().__init__("moveit_advanced_demo")

        # 利用可能な MoveGroup アクションを検出
        action_names = self._discover_move_group_actions()
        self.move_group_client = None
        self._connected_action_name = None

        for action_name in action_names:
            try:
                client = ActionClient(self, MoveGroup, action_name)
                if client.wait_for_server(timeout_sec=5.0):
                    self.move_group_client = client
                    self._connected_action_name = action_name
                    self.get_logger().info(f"Connected to MoveGroup action: {action_name}")
                    break
            except Exception as e:
                self.get_logger().debug(f"Could not connect to {action_name}: {e}")

        if self.move_group_client is None:
            self.get_logger().warn(
                "MoveGroup action server not found. Try: ros2 launch moveit_config demo.launch.py"
            )

        # 現在の関節状態（/joint_states パブリッシュ用）
        self.current_joint1 = 0.0
        self.current_joint2 = 0.8
        self.current_joint3 = 0.2
        self.current_joint4 = 0.0
        self.current_joint5 = 0.0
        self.current_joint6 = 0.0
        self._current_target_joints = None

        # 軌道再生用
        self.trajectory_points = []
        self.trajectory_start_time = None
        self.trajectory_joint_indices = {}
        self.is_playing_trajectory = False
        self.current_trajectory_index = 0
        self.trajectory_time_scale = 2.0

        # パブリッシャ
        self.joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self.target_marker_publisher = self.create_publisher(
            Marker, "/target_pose_marker", 10
        )

        # 200Hz で /joint_states をパブリッシュ
        self.create_timer(0.005, self.publish_joint_states)
        self.publish_joint_states()

    def _discover_move_group_actions(self):
        """ros2 action list から MoveGroup 型のアクション名を取得。優先: /move_group/move_action"""
        actions = []
        try:
            import subprocess
            result = subprocess.run(
                ["ros2", "action", "list", "-t"],
                capture_output=True,
                text=True,
                timeout=5.0,
            )
            if result.returncode != 0:
                return ["/move_group/move_action", "/move_action"]
            for line in result.stdout.splitlines():
                line = line.strip()
                if "MoveGroup" not in line or not line:
                    continue
                parts = line.split()
                if parts:
                    name = parts[0]
                    if name not in actions:
                        actions.append(name)
            # 優先: ノード名付き（move_group がリマップしたもの）
            node_specific = [a for a in actions if "/move_group/" in a]
            if node_specific:
                return node_specific
            if actions:
                return actions
        except Exception:
            pass
        return ["/move_group/move_action", "/move_action"]

    def _get_error_name(self, code):
        """MoveIt エラーコードを文字列に"""
        names = {
            1: "SUCCESS",
            99999: "FAILURE",
            99998: "PLANNING_FAILED",
            99997: "INVALID_MOTION_PLAN",
            99996: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            99995: "CONTROL_FAILED",
            99994: "UNABLE_TO_AQUIRE_SENSOR_DATA",
            99993: "TIMED_OUT",
            99992: "PREEMPTED",
            99991: "START_STATE_IN_COLLISION",
            99990: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            99989: "GOAL_IN_COLLISION",
            99988: "GOAL_VIOLATES_PATH_CONSTRAINTS",
        }
        return names.get(code, f"UNKNOWN_{code}")

    def _joints_to_ee_tip(self, j1, j2, j3, j4, j5, j6):
        """
        簡易 FK: 関節角度から tool_link 先端の base_link 座標 (x,y,z) を計算。
        arm.xacro のリンク構成に合わせている（j4,j5,j6 は手先方向への寄与のみ近似）。
        """
        # arm.xacro: base(0,0,0.1) -> j2(0,0,0.6) -> 腕 L1=0.6 -> 肘 -> L2=0.1+0.5 -> 手首 -> L3=0.05+0.02+0.02
        z_base_j2 = 0.1 + 0.5  # 0.6
        L1 = 0.6   # arm_link (j2->j3)
        L2 = 0.1 + 0.5  # elbow_link + forearm (j3->j5 まで)
        L3 = 0.05 + 0.02 + 0.02  # wrist + hand + tool
        # 肩平面内の水平成分と高さ（j1 で回転するので半径 = 水平距離）
        radius = L1 * math.sin(j2) + L2 * math.sin(j2 + j3) + L3 * math.sin(j2 + j3)
        z_arm = z_base_j2 + L1 * math.cos(j2) + L2 * math.cos(j2 + j3) + L3 * math.cos(j2 + j3)
        x = math.cos(j1) * radius
        y = math.sin(j1) * radius
        z = z_arm
        return (x, y, z)

    def _publish_target_marker(self):
        """目標姿勢のエンドエフェクタ位置にマーカー（黄緑の球）をパブリッシュ"""
        if self._current_target_joints is None:
            return
        j1, j2, j3, j4, j5, j6 = self._current_target_joints
        try:
            x, y, z = self._joints_to_ee_tip(j1, j2, j3, j4, j5, j6)
        except Exception:
            return
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "target"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = Point(x=x, y=y, z=z)
        m.pose.orientation.w = 1.0
        m.scale.x = 0.06
        m.scale.y = 0.06
        m.scale.z = 0.06
        m.color.r = 0.2
        m.color.g = 0.9
        m.color.b = 0.2
        m.color.a = 0.9
        self.target_marker_publisher.publish(m)

    def move_to_joint_positions(
        self,
        joint1,
        joint2,
        joint3,
        joint4,
        joint5,
        joint6,
        wait_for_result=True,
        timeout=30.0,
    ):
        """
        関節目標を MoveGroup に送り、計画された軌道を取得して再生する。
        戻り値: 成功 True / 失敗 False
        """
        if self.move_group_client is None:
            self.get_logger().error("MoveGroup action server not available.")
            return False

        self._current_target_joints = (
            float(joint1),
            float(joint2),
            float(joint3),
            float(joint4),
            float(joint5),
            float(joint6),
        )
        self._publish_target_marker()

        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal_msg.request.group_name = "arm_group"
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 1.0
        goal_msg.request.max_acceleration_scaling_factor = 1.0

        constraints = Constraints()
        tol = 0.002
        for jname, jpos in [
            ("joint1", joint1),
            ("joint2", joint2),
            ("joint3", joint3),
            ("joint4", joint4),
            ("joint5", joint5),
            ("joint6", joint6),
        ]:
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = float(jpos)
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(constraints)

        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.look_around = True
        goal_msg.planning_options.look_around_attempts = 3

        self.get_logger().info(
            f"Moving to: j1={joint1:.3f} j2={joint2:.3f} j3={joint3:.3f} "
            f"j4={joint4:.3f} j5={joint5:.3f} j6={joint6:.3f}"
        )

        future = self.move_group_client.send_goal_async(goal_msg)
        if not wait_for_result:
            return True

        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error("Send goal timed out.")
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            self.get_logger().error("Get result timed out.")
            return False

        result = result_future.result().result
        has_trajectory = (
            hasattr(result, "planned_trajectory")
            and result.planned_trajectory
            and len(result.planned_trajectory.joint_trajectory.points) > 0
        )

        if has_trajectory:
            traj = result.planned_trajectory.joint_trajectory
            self.get_logger().info(
                f"Planning succeeded (trajectory points: {len(traj.points)})"
            )
            self.visualize_trajectory(traj)
            self.wait_for_trajectory_completion()
            return True
        if getattr(result, "error_code", None) and result.error_code.val == 1:
            self.get_logger().info("Motion succeeded (no trajectory).")
            return True

        code = getattr(result, "error_code", None)
        code_val = code.val if code else None
        self.get_logger().error(
            f"Planning failed: {self._get_error_name(code_val) if code_val else 'unknown'}"
        )
        return False

    def publish_joint_states(self):
        """タイマーコールバック: 軌道再生中は補間値、それ以外は現在関節値を /joint_states にパブリッシュ"""
        if self.is_playing_trajectory and self.trajectory_points and self.trajectory_start_time is not None:
            elapsed_ns = self.get_clock().now().nanoseconds - self.trajectory_start_time
            elapsed = elapsed_ns / 1e9
            last_pt = self.trajectory_points[-1]
            last_time = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec / 1e9
            scaled = last_time * self.trajectory_time_scale
            if elapsed <= scaled:
                t_traj = elapsed / self.trajectory_time_scale
                self._interpolate_trajectory(t_traj)
            else:
                # 再生完了
                ji = self.trajectory_joint_indices
                if len(last_pt.positions) > max(ji.get("joint1", 0), ji.get("joint2", 0)):
                    self.current_joint1 = last_pt.positions[ji.get("joint1", 0)]
                    self.current_joint2 = last_pt.positions[ji.get("joint2", 0)]
                if ji.get("joint3") is not None and len(last_pt.positions) > ji["joint3"]:
                    self.current_joint3 = last_pt.positions[ji["joint3"]]
                if ji.get("joint4") is not None and len(last_pt.positions) > ji["joint4"]:
                    self.current_joint4 = last_pt.positions[ji["joint4"]]
                if ji.get("joint5") is not None and len(last_pt.positions) > ji["joint5"]:
                    self.current_joint5 = last_pt.positions[ji["joint5"]]
                if ji.get("joint6") is not None and len(last_pt.positions) > ji["joint6"]:
                    self.current_joint6 = last_pt.positions[ji["joint6"]]
                self.is_playing_trajectory = False
                self.trajectory_points = []
                self.current_trajectory_index = 0
                self.get_logger().info("Trajectory playback completed.")

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        msg.position = [
            float(self.current_joint1),
            float(self.current_joint2),
            float(self.current_joint3),
            float(self.current_joint4),
            float(self.current_joint5),
            float(self.current_joint6),
        ]
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        self.joint_state_publisher.publish(msg)

    def _interpolate_trajectory(self, elapsed_time):
        """軌道の waypoints 間を線形補間して現在の関節位置を更新"""
        if not self.trajectory_points:
            return
        ji = self.trajectory_joint_indices
        if ji.get("joint1") is None or ji.get("joint2") is None:
            return
        start_idx = max(0, self.current_trajectory_index - 1)
        for i in range(start_idx, len(self.trajectory_points)):
            pt = self.trajectory_points[i]
            t = pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9
            if elapsed_time <= t:
                self.current_trajectory_index = i
                if i == 0:
                    if len(pt.positions) > max(ji.values()):
                        self.current_joint1 = pt.positions[ji["joint1"]]
                        self.current_joint2 = pt.positions[ji["joint2"]]
                        if ji.get("joint3") is not None:
                            self.current_joint3 = pt.positions[ji["joint3"]]
                        if ji.get("joint4") is not None:
                            self.current_joint4 = pt.positions[ji["joint4"]]
                        if ji.get("joint5") is not None:
                            self.current_joint5 = pt.positions[ji["joint5"]]
                        if ji.get("joint6") is not None:
                            self.current_joint6 = pt.positions[ji["joint6"]]
                else:
                    prev = self.trajectory_points[i - 1]
                    t_prev = prev.time_from_start.sec + prev.time_from_start.nanosec / 1e9
                    if t > t_prev:
                        alpha = (elapsed_time - t_prev) / (t - t_prev)
                        alpha = max(0.0, min(1.0, alpha))
                        max_idx = max(ji.values())
                        if len(pt.positions) > max_idx and len(prev.positions) > max_idx:
                            for name, idx in ji.items():
                                if idx is not None and idx <= max_idx:
                                    v = prev.positions[idx] + (pt.positions[idx] - prev.positions[idx]) * alpha
                                    setattr(self, f"current_{name}", v)
                return
        # 最後のポイントを超えた
        last = self.trajectory_points[-1]
        if len(last.positions) > max(ji.values()):
            self.current_joint1 = last.positions[ji["joint1"]]
            self.current_joint2 = last.positions[ji["joint2"]]
            if ji.get("joint3") is not None:
                self.current_joint3 = last.positions[ji["joint3"]]
            if ji.get("joint4") is not None:
                self.current_joint4 = last.positions[ji["joint4"]]
            if ji.get("joint5") is not None:
                self.current_joint5 = last.positions[ji["joint5"]]
            if ji.get("joint6") is not None:
                self.current_joint6 = last.positions[ji["joint6"]]

    def visualize_trajectory(self, joint_trajectory):
        """
        計画された JointTrajectory を再生用に準備する。
        時間が 0 の場合はデフォルト時間を付与し、末尾に正確な目標点を追加する。
        """
        if not joint_trajectory or not joint_trajectory.points:
            self.get_logger().warn("Empty trajectory.")
            return

        names = joint_trajectory.joint_names
        joint_indices = {}
        for idx, n in enumerate(names):
            if n in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"):
                joint_indices[n] = idx
        if "joint1" not in joint_indices or "joint2" not in joint_indices:
            self.get_logger().error("Trajectory missing joint1 or joint2.")
            return

        self.trajectory_joint_indices = joint_indices
        points_list = list(joint_trajectory.points)

        # 元の軌道の総時間を計算し、0 ならデフォルト時間を付与（先にやる）
        total_time = 0.0
        if points_list:
            last_pt = points_list[-1]
            total_time = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec / 1e9
        if total_time <= 0.0 and len(points_list) > 1:
            dt = 0.25
            total_time = (len(points_list) - 1) * dt
            for i, pt in enumerate(points_list):
                t = i * dt
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            self.get_logger().warn(f"Trajectory had no time; using default {total_time:.2f}s")
        elif total_time <= 0.0 and len(points_list) == 1:
            total_time = 1.0

        # 正確な目標点を末尾に追加（黄緑マーカーに届くように）
        if self._current_target_joints is not None and points_list:
            target = self._current_target_joints
            last_pt = points_list[-1]
            target_by_name = {
                "joint1": float(target[0]),
                "joint2": float(target[1]),
                "joint3": float(target[2]),
                "joint4": float(target[3]),
                "joint5": float(target[4]),
                "joint6": float(target[5]),
            }
            goal_positions = [
                target_by_name.get(n, last_pt.positions[i] if i < len(last_pt.positions) else 0.0)
                for i, n in enumerate(names)
            ]
            extra_time = total_time + 0.25
            extra = JointTrajectoryPoint()
            extra.positions = goal_positions
            extra.velocities = [0.0] * len(goal_positions) if not last_pt.velocities else list(last_pt.velocities)
            extra.time_from_start = Duration(sec=int(extra_time), nanosec=int((extra_time % 1) * 1e9))
            points_list.append(extra)
            total_time = extra_time
            self.get_logger().info("Appended exact goal point for target marker.")

        self.trajectory_points = points_list
        self.trajectory_start_time = self.get_clock().now().nanoseconds
        self.is_playing_trajectory = True
        self.current_trajectory_index = 0

        # 最初の点で現在関節を初期化
        if points_list and joint_indices:
            fp = points_list[0]
            if len(fp.positions) > max(joint_indices.values()):
                self.current_joint1 = fp.positions[joint_indices["joint1"]]
                self.current_joint2 = fp.positions[joint_indices["joint2"]]
                if "joint3" in joint_indices:
                    self.current_joint3 = fp.positions[joint_indices["joint3"]]
                if "joint4" in joint_indices:
                    self.current_joint4 = fp.positions[joint_indices["joint4"]]
                if "joint5" in joint_indices:
                    self.current_joint5 = fp.positions[joint_indices["joint5"]]
                if "joint6" in joint_indices:
                    self.current_joint6 = fp.positions[joint_indices["joint6"]]

        playback = total_time * self.trajectory_time_scale
        self.get_logger().info(
            f"Visualizing trajectory: {len(points_list)} points, "
            f"duration {total_time:.2f}s -> playback {playback:.2f}s (scale {self.trajectory_time_scale}x)"
        )

    def wait_for_trajectory_completion(self):
        """軌道再生が終わるまで待つ（スケール後の再生時間 + 余裕）"""
        if not self.trajectory_points:
            return
        last = self.trajectory_points[-1]
        total = last.time_from_start.sec + last.time_from_start.nanosec / 1e9
        playback = total * self.trajectory_time_scale
        start = time.time()
        while self.is_playing_trajectory and (time.time() - start) < playback + 0.5:
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)


def main():
    rclpy.init()
    node = MoveItAdvancedDemo()
    if node.move_group_client is None:
        node.get_logger().error("Start MoveIt first: ros2 launch moveit_config demo.launch.py")
        node.destroy_node()
        rclpy.shutdown()
        return
    node.get_logger().info("MoveIt Advanced Demo ready. Use run_named_poses.py for the full demo.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
