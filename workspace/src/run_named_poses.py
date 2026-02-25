#!/usr/bin/env python3
"""
6-DoFアームの名前付きポーズ（group_state）を順にパスプランニングするデモ。
手首・手先の向き（j4,j5,j6）を変えるポーズを含め、6軸の強みを見せる。
デモ開始前に障害物を1つ置き、MoveItが衝突を避けて計画する様子を確認できる。

使い方:
  1. 別ターミナルで: ros2 launch moveit_config demo.launch.py
  2. 本スクリプト: cd /workspace && python3 src/run_named_poses.py

障害物の表示:
  - Foxglove: 3D パネルで「Add panel」→「3D」を追加し、
    「Topics」に /obstacle_markers を追加（MarkerArray で直方体が表示されます）。
  - RViz: 「Add」→「By topic」→「/planning_scene」の PlanningScene を追加。
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
import time
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

# 2dof_arm.srdf の group_state と同一の関節角度（rad）: (j1, j2, j3, j4, j5, j6)
NAMED_POSES = {
    "home": (0.0, 0.8, 0.2, 0.0, 0.0, 0.0),
    "ready": (0.0, 0.7, 0.15, 0.0, 0.0, 0.0),
    "vertical": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "shoulder_30": (0.0, 0.52, 0.2, 0.0, 0.0, 0.0),
    "shoulder_60": (0.0, 1.05, 0.35, 0.0, 0.0, 0.0),
    "elbow_bent": (0.0, 1.0, 0.45, 0.0, 0.0, 0.0),
    "shoulder_90": (0.0, 1.57, 0.5, 0.0, 0.0, 0.0),
    "shoulder_100": (0.0, 1.75, 0.45, 0.0, 0.0, 0.0),
    "arm_down": (0.0, 1.9, 0.5, 0.0, 0.0, 0.0),
    "arm_reach": (0.0, 2.2, 0.3, 0.0, 0.0, 0.0),
    "arm_forward": (0.0, 1.2, 0.25, 0.0, 0.0, 0.0),
    "arm_folded": (0.0, 1.15, 0.6, 0.0, 0.0, 0.0),
    "tilt_down": (0.0, 0.7, 0.2, 0.0, -0.6, 0.0),
    "tilt_up": (0.0, 0.7, 0.2, 0.0, 0.5, 0.0),
    "wrist_roll_90": (0.0, 0.75, 0.2, 0.0, 0.0, 1.57),
    "wrist_yaw_left": (0.0, 0.7, 0.2, 0.8, 0.0, 0.0),
    "wrist_yaw_right": (0.0, 0.7, 0.2, -0.8, 0.0, 0.0),
    "point_right": (0.6, 0.75, 0.15, 0.0, 0.0, 0.0),
    "point_left": (-0.6, 0.75, 0.15, 0.0, 0.0, 0.0),
    "pick_pose": (0.0, 1.0, 0.4, 0.0, -0.7, 0.0),
    "place_pose": (0.3, 0.8, 0.25, 0.5, 0.2, 0.0),
}

# 障害物: base_link 基準の位置 (x, y, z)、サイズ (x, y, z) [m]
# アーム正面の「通り道」に近い位置に置き、はっきり避ける動きが見えるようにする
# （以前わかりやすかった設定に戻している）
OBSTACLE_BOX_POSITION = (0.4, 0.0, 0.7)
OBSTACLE_BOX_SIZE = (0.12, 0.12, 0.22)


def add_obstacle_box(node, name="demo_obstacle", position=None, size=None):
    """
    プランニングシーンに直方体の障害物を1つ追加する。
    MoveIt はこの障害物を避けて軌道を計画する。
    """
    position = position or OBSTACLE_BOX_POSITION
    size = size or OBSTACLE_BOX_SIZE
    for svc_name in ("/apply_planning_scene", "/move_group/apply_planning_scene"):
        client = node.create_client(ApplyPlanningScene, svc_name)
        if client.wait_for_service(timeout_sec=2.0):
            break
    else:
        node.get_logger().warn(
            "ApplyPlanningScene not available; running without obstacle. "
            "Obstacle avoidance will not be visible."
        )
        return False

    co = CollisionObject()
    co.header = Header(frame_id="base_link", stamp=node.get_clock().now().to_msg())
    co.id = name
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [float(size[0]), float(size[1]), float(size[2])]
    co.primitives = [box]
    pose = Pose()
    pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
    pose.orientation.w = 1.0
    co.primitive_poses = [pose]
    co.operation = CollisionObject.ADD

    scene = PlanningScene()
    scene.is_diff = True
    scene.world = PlanningSceneWorld()
    scene.world.collision_objects = [co]
    scene.robot_state.is_diff = True

    req = ApplyPlanningScene.Request()
    req.scene = scene
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        node.get_logger().warn("ApplyPlanningScene call timed out.")
        return False
    result = future.result()
    success = getattr(result, "success", True)
    if success:
        node.get_logger().info(
            f"Added obstacle '{name}' at {position}, size {size}. "
            "Motion plans will avoid it."
        )
        # RViz で障害物を表示するため /planning_scene にもパブリッシュ（複数回で確実に）
        try:
            pub = node.create_publisher(PlanningScene, "/planning_scene", 10)
            for _ in range(5):
                scene.header.stamp = node.get_clock().now().to_msg()
                scene.header.frame_id = "base_link"
                pub.publish(scene)
                rclpy.spin_once(node, timeout_sec=0.05)
                time.sleep(0.05)
        except Exception as e:
            node.get_logger().debug(f"Could not publish to /planning_scene: {e}")
    return success


def _obstacle_marker_message(position, size, stamp):
    """障害物の MarkerArray を1つ作る（Foxglove 用）。stamp は node.get_clock().now().to_msg() を渡す。"""
    m = Marker()
    m.header.frame_id = "base_link"
    m.header.stamp = stamp
    m.ns = "obstacle"
    m.id = 0
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = float(position[0])
    m.pose.position.y = float(position[1])
    m.pose.position.z = float(position[2])
    m.pose.orientation.w = 1.0
    m.scale.x = float(size[0])
    m.scale.y = float(size[1])
    m.scale.z = float(size[2])
    m.color.r = 1.0
    m.color.g = 0.4
    m.color.b = 0.0
    m.color.a = 0.9
    msg = MarkerArray()
    msg.markers.append(m)
    return msg


def publish_obstacle_marker_for_foxglove(node, position=None, size=None):
    """
    Foxglove で障害物を見えるように、同じ位置・サイズの直方体を
    visualization_msgs/MarkerArray で /obstacle_markers にパブリッシュする。
    Foxglove の 3D パネルでこのトピックを追加するとオレンジの箱が表示される。
    デモ中も表示を維持するため、バックグラウンドで約2秒ごとに再パブリッシュする。
    """
    position = position or OBSTACLE_BOX_POSITION
    size = size or OBSTACLE_BOX_SIZE
    pub = node.create_publisher(MarkerArray, "/obstacle_markers", 10)
    import threading
    stop_obstacle_publish = threading.Event()

    def _publish_loop():
        while not stop_obstacle_publish.wait(2.0):
            try:
                stamp = node.get_clock().now().to_msg()
                msg = _obstacle_marker_message(position, size, stamp)
                pub.publish(msg)
            except Exception:
                break

    thread = threading.Thread(target=_publish_loop, daemon=True)
    thread.start()
    # 起動直後も数回パブリッシュ
    for _ in range(5):
        stamp = node.get_clock().now().to_msg()
        msg = _obstacle_marker_message(position, size, stamp)
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.15)
    node.get_logger().info(
        "Obstacle published to /obstacle_markers. In Foxglove: add '3D' panel and add topic /obstacle_markers."
    )


def main():
    rclpy.init()
    from moveit_advanced_demo import MoveItAdvancedDemo

    demo = MoveItAdvancedDemo()
    if demo.move_group_client is None:
        demo.get_logger().error(
            "MoveGroup action server not available. Start MoveIt first: ros2 launch moveit_config demo.launch.py"
        )
        demo.destroy_node()
        rclpy.shutdown()
        return

    # 名前付きポーズ用: 軌道再生をゆっくりに
    demo.trajectory_time_scale = 2.0

    # 障害物を1つ置いて MoveIt の衝突回避を実感できるようにする
    demo.get_logger().info("Adding obstacle to planning scene (MoveIt will plan around it)...")
    add_obstacle_box(demo, name="demo_obstacle", position=OBSTACLE_BOX_POSITION, size=OBSTACLE_BOX_SIZE)
    time.sleep(0.5)
    # Foxglove で障害物を見えるように Marker をパブリッシュ（3D パネルで /obstacle_markers を追加）
    publish_obstacle_marker_for_foxglove(demo, position=OBSTACLE_BOX_POSITION, size=OBSTACLE_BOX_SIZE)

    # 6-DoF デモ用シーケンス（少し整理版）:
    #  - joint2 を3段階で曲げる → 肘をさらに前に伸ばす
    #  - 手先の上下（j5）、ロール（j6）、ヨー（j4）を変える
    #  - ベース回転で左右を向く → ピック/プレース → ホーム
    sequence = [
        ("home", NAMED_POSES["home"]),
        ("vertical", NAMED_POSES["vertical"]),        # j2=0 まっすぐ
        ("shoulder_30", NAMED_POSES["shoulder_30"]),  # j2≈30°
        ("elbow_bent", NAMED_POSES["elbow_bent"]),    # 中間
        ("arm_reach", NAMED_POSES["arm_reach"]),      # 最大に近く前に伸ばす
        ("arm_folded", NAMED_POSES["arm_folded"]),    # たたむ
        ("ready", NAMED_POSES["ready"]),
        ("tilt_down", NAMED_POSES["tilt_down"]),      # 手先を下向き（j5）
        ("tilt_up", NAMED_POSES["tilt_up"]),          # 手先を上向き（j5）
        ("wrist_roll_90", NAMED_POSES["wrist_roll_90"]),  # 手首ロール（j6）
        ("wrist_yaw_left", NAMED_POSES["wrist_yaw_left"]),
        ("wrist_yaw_right", NAMED_POSES["wrist_yaw_right"]),
        ("ready", NAMED_POSES["ready"]),
        ("point_right", NAMED_POSES["point_right"]),  # ベース回転で右へ（j1）
        ("point_left", NAMED_POSES["point_left"]),    # 左へ
        ("pick_pose", NAMED_POSES["pick_pose"]),      # 掴む姿勢（j5 下向き）
        ("place_pose", NAMED_POSES["place_pose"]),    # 置く姿勢（j4 で向き変更）
        ("home", NAMED_POSES["home"]),
    ]

    demo.get_logger().info("=== 6-DoF Named Poses Demo ===")
    demo.get_logger().info("Sequence: home → vertical → shoulder_30/60/90/100 → arm_down/reach → ... → home")
    for i, (name, joints) in enumerate(sequence, 1):
        demo.get_logger().info(f"[{i}/{len(sequence)}] Moving to: {name}")
        ok = demo.move_to_joint_positions(*joints)
        if not ok:
            demo.get_logger().warn(f"  Failed to reach '{name}', continuing...")
        time.sleep(1.5)

    demo.get_logger().info("=== Demo finished ===")
    demo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
