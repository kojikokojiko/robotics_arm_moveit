# MoveIt 2 を触ってみた — わかったこと・仕組みのメモ・このプロジェクトでの使い方

MoveIt について詳しいわけではなく、ROS 2 で 6-DoF アームを動かすために**使ってみた**ときのメモです。**MoveIt って何ができるの？** とか **中身はどうなってる？** とか、**名前付きポーズと障害物回避のデモ**で実際にやったことを、自分用にまとめています。

---

## 1. MoveIt って何？

### 1.1 いまの理解

**MoveIt** は、ロボットアームの**モーションプランニング（動作計画）**用のオープンソースフレームワークで、**MoveIt 2** がその ROS 2 版として提供されています。自分は「アームを動かしたいときの軌道を出してくれるライブラリ」という理解で使っています。

主な役割は、**現在の関節角度**から**目標の関節角度（あるいは手先の姿勢）**まで、障害物やジョイントリミットを考慮した**衝突のない軌道**を計算することです。そのために、プランニング用ノード（`move_group`）、アクション／サービスインターフェース、ロボットモデルや制約の読み込み、OMPL などのプランナーとの統合が用意されています。

つまり、「ここまで動かしたい」という目標を渡すと、安全に到達できる関節の軌道を返してくれる、というイメージです。実際にモーターを駆動するかどうかは、ROS 2 Control やシミュレータなど別のレイヤーが担当する想定です。

### 1.2 なぜ MoveIt を使ったか

このプロジェクトで MoveIt を選んだ理由は、おおよそ次のとおりです。

- **軌道を自前で書かなくてよい**  
  関節空間や作業空間に障害物を置いても、衝突を避けた経路を自動で探索してくれる。
- **ロボットに依存しない設計**  
  URDF／SRDF でロボットを定義すれば、同じ API で別のアームにも流用しやすい。
- **ROS 2 との統合**  
  トピック・サービス・アクションで他ノードと連携でき、TF、JointState、Planning Scene と一貫した世界観で扱える。

---

## 2. 使ううえで押さえた概念

### 2.1 ロボットモデル（URDF / SRDF）

- **URDF (Unified Robot Description Format)**  
  リンク・ジョイント・外観・質量などの**物理的なロボットの形**を定義します。本プロジェクトでは `robot_description` パッケージの xacro から生成しています。

- **SRDF (Semantic Robot Description Format)**  
  MoveIt 用の**意味的な**情報を追加します。
  - **グループ (group)**: どのジョイントをまとめて計画するか（例: `arm_group` = joint1～joint6）。
  - **エンドエフェクタ**: 手先のリンク。
  - **名前付きポーズ (group_state)**: `home`, `ready`, `vertical`, `elbow_bent` などの**事前定義された関節角度**。RViz の「Planning」タブや、本プロジェクトの `run_named_poses.py` で参照します。

SRDF は「このロボットを MoveIt でどう扱うか」を決める設計図です。

### 2.2 プランニングシーン (Planning Scene)

**プランニングシーン**は、MoveIt が「今、世界がどうなっているか」を表す内部状態です。

- **ロボットの現在の関節角度**  
  `/joint_states` などから取得し、ロボットの現在姿勢として使います。
- **障害物**  
  衝突オブジェクト（箱・球・メッシュなど）を追加・削除できます。計画時に「これらに当たらない経路」を求めます。
- **許容される衝突（ACL）**  
  特定のリンク同士の接触を許可する設定も可能です。

クライアントは `GetPlanningScene` サービスでシーンを取得し、`ApplyPlanningScene` で障害物を追加できます。本プロジェクトの `moveit_advanced_demo.py` でも Planning Scene を利用しています。

### 2.3 目標の表し方（制約）

「どこに動かすか」は**制約 (Constraints)** で指定します。

- **関節空間の目標 (JointConstraint)**  
  「joint1 = 0.5 rad, joint2 = 1.0 rad, …」のように、各関節の目標値と許容幅（tolerance_above / tolerance_below）を指定します。本プロジェクトでは名前付きポーズを**関節角度の組**として渡し、6 本すべてに `JointConstraint` を設定して MoveIt に計画させています。
- **デカルト（手先）の目標 (Position/Orientation Constraint)**  
  手先の位置・姿勢を指定する場合は、逆運動学（IK）で関節角度に変換してから計画します。

MoveIt は「これらの制約を満たすような経路」を探索します。

### 2.4 モーションプランニングと軌道

- **モーションプランニング**  
  現在状態から目標状態まで、**衝突回避**と**ジョイントリミット**を満たす経路（ waypoints の列）を計算します。内部では **OMPL (Open Motion Planning Library)** などのプランナーが使われます。
- **軌道 (Trajectory)**  
  計画結果は **関節空間の軌道**（時間刻みの `JointTrajectory`）として返ります。各点に「いつその角度にいるか」（`time_from_start`）が付き、実機やシミュレータはこの軌道に沿って動かします。

本プロジェクトでは **plan_only = True** で「計画だけ」を依頼し、返ってきた軌道を自前で `/joint_states` に流して RViz 上で可視化しています（実機制御は行っていません）。

---

## 3. 中で何が動いてるか（自分用メモ）

### 3.1 move_group ノード

実際に動かすと、中心になってるのは **move_group** というノードでした。起動するとだいたい次のようなことをしているようです。

- ロボットモデル（URDF + SRDF）の読み込み
- プランニングシーンの維持（`/joint_states` の購読、障害物の適用）
- **MoveGroup アクションサーバー**の提供（クライアントが「この目標で計画して」とリクエストする窓口）
- プランニングパイプラインの実行（プランナー・運動学の呼び出し）

本プロジェクトでは `ros2 launch moveit_config demo.launch.py` で `move_group` と RViz を起動し、そのあと `run_named_poses.py` が **MoveGroup アクション**で「関節目標」を送って計画を依頼します。

### 3.2 MoveGroup アクション

クライアント（本プロジェクトでは `moveit_advanced_demo.py`）は、**MoveGroup アクション**で 1 回の「計画リクエスト」を送ります。

- **リクエスト (Goal)** に含める主なもの:
  - `group_name`: どのプランニンググループか（例: `arm_group`）
  - `goal_constraints`: 目標（本プロジェクトでは 6 関節分の `JointConstraint`）
  - `workspace_parameters`, `allowed_planning_time`, `max_velocity_scaling_factor` など
  - `planning_options.plan_only`: True にすると「軌道を返すだけで実行はしない」
- **レスポンス (Result)** に含まれる主なもの:
  - `planned_trajectory`: 計画された `JointTrajectory`（関節名と各時刻の positions / velocities など）
  - `error_code`: 成功(1) や各種失敗コード

つまり「この関節目標まで、衝突回避して軌道を出して」という 1 回のやり取りが、1 本の軌道になります。`run_named_poses.py` は名前付きポーズのリストに対して、順番にこのアクションを呼び出しています。

### 3.3 プランニングパイプラインの流れ（イメージ）

1. クライアントが **MoveGroup.Goal** を送信（例: joint1～6 の目標値）。
2. move_group が現在の **プランニングシーン**（現在の joint_states + 障害物）を取得。
3. **運動学**（KDL など）で必要に応じて FK/IK を計算（関節目標の場合はそのまま利用）。
4. **プランナー（OMPL 等）**が、現在状態から目標状態まで、障害物・ジョイントリミットを避ける経路を探索。
5. 得られた経路を **軌道**（時間パラメータ付き）に変換し、**MoveGroup.Result** の `planned_trajectory` として返す。

設定ファイル（`joint_limits.yaml`, `kinematics.yaml`, `ompl_planning_pipeline.yaml` など）が、このパイプラインのパラメータを決めています。

### 3.4 運動学 (Kinematics)

手先の位置・姿勢と関節角度の対応は **運動学** で計算します。

- **順運動学 (FK)**: 関節角度 → 手先の姿勢
- **逆運動学 (IK)**: 手先の姿勢 → 関節角度（解が複数ある場合がある）

本プロジェクトの `moveit_config` では **KDL (Kinematics and Dynamics Library)** プラグイン（`KDLKinematicsPlugin`）を指定しています。関節目標のみを送る場合は IK は使わず、プランナーが関節空間で直接計画します。

---

## 4. このプロジェクトではどう使ってるか

### 4.1 起動と役割分担

1. **MoveIt の起動**  
   `ros2 launch moveit_config demo.launch.py`  
   - move_group ノードが立ち上がり、MoveGroup アクションサーバー（例: `/move_group/move_action`）が利用可能になる。
   - RViz でロボットモデルと Planning Scene が表示される。

2. **実装確認用スクリプト**  
   `python3 src/run_named_poses.py`  
   - `MoveItAdvancedDemo`（`moveit_advanced_demo.py`）を利用。
   - **デモ開始前に障害物を 1 つ**プランニングシーンに追加し、MoveIt が衝突を避けて計画する様子を確認できる。
   - 名前付きポーズのリスト（整理したシーケンス）に沿って、順に **MoveGroup アクション**で「関節目標」を送り、計画された軌道を受け取る。
   - 受け取った軌道は **plan_only** のため実行はせず、自前で時間スケールをかけて `/joint_states` にパブリッシュし、RViz や Foxglove 上でアームが動いて見えるようにしている。

### 4.2 名前付きポーズと SRDF

名前付きポーズは **SRDF の group_state** で定義されています（`moveit_config/config/2dof_arm.srdf`）。  
`run_named_poses.py` の `NAMED_POSES` は、それと同じ関節角度を Python の辞書で持っています。  
スクリプトは「home → vertical → shoulder_30 → … → home」のように、この辞書とシーケンスに従って目標を送り、MoveIt に軌道を計画させています。

### 4.3 計画リクエストの内容（要点）

`moveit_advanced_demo.py` の `move_to_joint_positions()` では、おおよそ次のようにしています。

- **goal_constraints**: 6 関節すべてに `JointConstraint` を追加（位置と tolerance）。
- **planning_options.plan_only = True**: 軌道のみ取得し、MoveIt 側では実行しない。
- **look_around = True**: 解が見つかりやすくなるよう複数試行。

返ってきた `planned_trajectory` のポイント列に「正確な目標の 1 点」を末尾に追加し、時間が 0 のときはデフォルトの時間配分を付与。そのうえで `trajectory_time_scale` をかけて再生し、RViz 上で滑らかに見えるようにしています。

### 4.4 設定ファイルの役割

| ファイル | 役割 |
|----------|------|
| **URDF / xacro** | ロボットのリンク・ジョイント・見た目・質量。MoveIt はこれを読み込んでロボットモデルを構築。 |
| **SRDF (2dof_arm.srdf)** | プランニンググループ（arm_group）、エンドエフェクタ、名前付きポーズ（group_state）。 |
| **joint_limits.yaml** | 各関節の min/max、速度・加速度リミット。プランナーがこの範囲内で計画。 |
| **kinematics.yaml** | 運動学プラグイン（KDL）とそのパラメータ。 |
| **move_group.launch.py** | move_group ノードの起動、パラメータの読み込み、MoveGroup アクションのリマップ（例: `/move_group/move_action`）。 |

これらが揃うことで、「6-DoF アーム用の MoveIt」として一貫した設定になります。

### 4.5 障害物の追加と衝突回避の可視化

`run_named_poses.py` では、名前付きポーズのシーケンスを回す**前**に、プランニングシーンへ**障害物を 1 つ追加**しています。

- **追加の仕方**  
  `ApplyPlanningScene` サービス（`/apply_planning_scene` または `/move_group/apply_planning_scene`）に、**PlanningScene** メッセージを送ります。その中に **CollisionObject**（直方体）を 1 つ入れ、`frame_id = "base_link"`、位置・サイズを指定しています。
- **デフォルトの位置・サイズ**  
  base_link 基準で **位置 (0.4, 0.0, 0.7) [m]**、**サイズ (0.12, 0.12, 0.22) [m]** です。アームの正面の「通り道」付近に置いてあり、軌道が障害物を避けて回り込む様子が見やすいようにしてあります。
- **MoveIt 側の挙動**  
  move_group は計画時にこの障害物を含めたプランニングシーンを使うため、**衝突しない経路**だけを返します。解が見つからない場合はエラー（FAILURE）になります。

障害物を**目で見る**ためには、可視化ツール側の設定が必要です。

- **Foxglove**  
  Planning Scene をそのまま表示する機能はないため、**同じ位置・サイズの直方体**を `visualization_msgs/MarkerArray` で **`/obstacle_markers`** にパブリッシュしています。Foxglove で「3D」パネルを追加し、トピック **`/obstacle_markers`** を追加すると、オレンジ色の箱として表示されます。デモ中も表示を維持するため、バックグラウンドで約 2 秒ごとに再パブリッシュしています。
- **RViz**  
  「Add」→「By topic」→「`/planning_scene`」の **PlanningScene** を追加するか、既存の Planning Scene 表示で **Scene Geometry** を有効にすると、ApplyPlanningScene で追加した障害物が表示されます。

### 4.6 デモのシーケンス（整理版）

名前付きポーズのシーケンスは、「ほぼ同じような位置にしか移動しない」ポーズを省略して整理しています。

- **流れのイメージ**  
  home → vertical → shoulder_30 → elbow_bent → arm_reach → arm_folded → ready → tilt_down / tilt_up（手先の向き）→ wrist_roll_90 → wrist_yaw_left / right → ready → point_right / point_left（ベース回転）→ pick_pose → place_pose → home
- **省略しているポーズ**  
  shoulder_60, shoulder_90, shoulder_100, arm_down, arm_forward など、肩・肘の段階の細かいステップと、`ready` の重複を減らしています。動きのバリエーション（joint2 を曲げる → 手首・手先 → 左右 → ピック/プレース）は残しつつ、デモ時間と「毎回ほぼ同じ姿勢」の繰り返しを抑えています。

---

## 5. まとめ（自分用）

- **MoveIt** は「目標まで、衝突とかリミットを考えた軌道を出してくれる」やつ、という理解。
- 実際に動いてる中心は **move_group** ノードで、**MoveGroup アクション**に目標を送ると計画された軌道が返ってくる。
- ロボットは **URDF/SRDF** で定義して、**プランニングシーン**に今の状態や障害物を入れておくと、その前提で経路を計算してくれる。
- このプロジェクトでは、**plan_only** で軌道だけもらって、障害物を 1 つ置いた状態で `run_named_poses.py` のシーケンスを回し、軌道を `/joint_states` で再生して RViz や Foxglove で見ている。障害物は **ApplyPlanningScene** で追加し、Foxglove 用に **`/obstacle_markers`** でも同じ形を出している。

このくらい押さえておくと、設定やコードをいじるときに追いやすかった、というメモ。  
`run_named_poses.py` のブロックごとの解説は **付録 A** にまとめた。

---

## 付録 A. run_named_poses.py のコード解説

**run_named_poses.py** の「ここで何してるか」をブロックごとにメモしたもの。MoveIt を起動したあと、このスクリプトでデモの動作確認をしている想定。本編を読み終えたあとでコードを追うとき用。

### A.1 ファイル全体の役割

- **目的**: SRDF で定義した「名前付きポーズ」を **順番に** 目標として MoveIt に送り、計画された軌道を取得して **RViz や Foxglove 上でアームを動かして見せる**。あわせて**障害物を 1 つ追加**し、MoveIt が衝突を避けて計画する様子を確認できる。
- **流れの要約**:  
  ROS 2 を初期化 → MoveIt のアクションサーバーに接続 → 再生速度を設定 → **障害物をプランニングシーンに追加**（ApplyPlanningScene）→ **Foxglove 用に `/obstacle_markers` をパブリッシュ** → 整理したポーズのシーケンスを 1 つずつ「目標」として送信 → 各目標ごとに軌道をもらい、`/joint_states` で再生 → 最後にノード終了。

---

### A.2 冒頭（import まわり）

```python
#!/usr/bin/env python3
"""
6-DoFアームの名前付きポーズ（group_state）を順にパスプランニングするデモ。
...
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
import time
```

- **`sys.path.insert(...)`**  
  このスクリプトが置いてあるディレクトリ（`workspace/src`）を Python の検索パスに追加しています。  
  同じフォルダにある **moveit_advanced_demo** を `from moveit_advanced_demo import ...` で読み込むためです（後述の `main()` 内で import）。
- **`rclpy`**  
  ROS 2 の Python クライアントライブラリ。ノードの作成・スピン・シャットダウンに使います。
- **`time`**  
  ポーズとポーズのあいだで `time.sleep(1.5)` するために使います。

---

### A.3 NAMED_POSES 辞書

```python
# 2dof_arm.srdf の group_state と同一の関節角度（rad）: (j1, j2, j3, j4, j5, j6)
NAMED_POSES = {
    "home": (0.0, 0.8, 0.2, 0.0, 0.0, 0.0),
    "ready": (0.0, 0.7, 0.15, 0.0, 0.0, 0.0),
    "vertical": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    ...
}
```

- **何をしているか**  
  SRDF の **group_state**（名前付きポーズ）と同じ関節角度を、Python の辞書で持っています。
- **タプルの意味**  
  各値は **ラジアン** で、順に **joint1, joint2, joint3, joint4, joint5, joint6** の目標角度です。  
  例: `"home"` は (0, 0.8, 0.2, 0, 0, 0) → ベースは 0、肩(j2) 0.8 rad、肘(j3) 0.2 rad、手首系は 0。
- **なぜ Python にも持つか**  
  MoveIt に「この関節角度を目標にして計画して」と送るには、**数値のリスト**が必要です。SRDF は XML なので、スクリプトから使いやすいように同じ値を辞書で定義しています。SRDF を変更したら、ここも同じ値に揃える必要があります。

### A.3.1 障害物の定数と追加処理

- **`OBSTACLE_BOX_POSITION`**  
  base_link 基準の障害物の中心位置 (x, y, z) [m]。デフォルトは **(0.4, 0.0, 0.7)** で、アームの正面・やや高い位置にあり、軌道が避けて回り込む様子が見やすいようにしてあります。
- **`OBSTACLE_BOX_SIZE`**  
  直方体の辺の長さ (x, y, z) [m]。デフォルトは **(0.12, 0.12, 0.22)** です。
- **`add_obstacle_box(node, ...)`**  
  `ApplyPlanningScene` サービスで、上記の位置・サイズの **CollisionObject**（直方体）をプランニングシーンに追加します。MoveIt はこの障害物を避けて軌道を計画します。
- **`publish_obstacle_marker_for_foxglove(node, ...)`**  
  同じ位置・サイズの直方体を **`visualization_msgs/MarkerArray`** で **`/obstacle_markers`** にパブリッシュします。Foxglove は Planning Scene を表示しないため、3D パネルでこのトピックを追加すると障害物がオレンジの箱として表示されます。デモ中も約 2 秒ごとに再パブリッシュして表示を維持しています。

---

### A.4 main() の前半：初期化と接続確認

```python
def main():
    rclpy.init()
    from moveit_advanced_demo import MoveItAdvancedDemo

    demo = MoveItAdvancedDemo()
    if demo.move_group_client is None:
        demo.get_logger().error(
            "MoveGroup action server not available. Start MoveIt first: ..."
        )
        ...
        return

    demo.trajectory_time_scale = 2.0
```

- **`rclpy.init()`**  
  ROS 2 のコンテキストを初期化します。ノードを作る前に 1 回だけ呼びます。
- **`from moveit_advanced_demo import MoveItAdvancedDemo`**  
  実際に MoveIt の **MoveGroup アクション** に送りつけたり、軌道を `/joint_states` で再生したりする処理は、すべて **moveit_advanced_demo.py** の `MoveItAdvancedDemo` クラスにあります。ここではそのクラスをインポートしています。
- **`demo = MoveItAdvancedDemo()`**  
  ノードを 1 つ作り、その中で MoveGroup のアクションクライアントが **`/move_group/move_action`**（またはフォールバックで `/move_action`）に接続を試みます。接続に成功すると `demo.move_group_client` が None ではなくなります。
- **`if demo.move_group_client is None: ... return`**  
  MoveIt の move_group が起動していないとアクションサーバーに繋がりません。その場合はエラーメッセージを出して **main を終了** します。  
  なので、「先に `ros2 launch moveit_config demo.launch.py` で MoveIt を立ち上げる」必要があります。
- **`demo.trajectory_time_scale = 2.0`**  
  計画された軌道を **何倍の時間で再生するか** を指定しています。`2.0` なら「計画の 2 倍の時間をかけて動かす」ので、見た目はゆっくりになります。  
  この値は `moveit_advanced_demo.py` 側で参照され、軌道再生のタイマー計算に使われます。

- **障害物の追加と Foxglove 用マーカー**  
  そのあと `add_obstacle_box(demo, ...)` でプランニングシーンに直方体の障害物を追加し、`publish_obstacle_marker_for_foxglove(demo, ...)` で同じ位置・サイズのマーカーを **`/obstacle_markers`** にパブリッシュしています。Foxglove の 3D パネルでこのトピックを追加すると、障害物がオレンジの箱として表示されます。

---

### A.5 sequence：どの順でどのポーズに動かすか

```python
    sequence = [
        ("home", NAMED_POSES["home"]),
        ("vertical", NAMED_POSES["vertical"]),
        ("shoulder_30", NAMED_POSES["shoulder_30"]),
        ...
        ("home", NAMED_POSES["home"]),
    ]
```

- **何をしているか**  
  「ポーズ名」と「そのポーズの関節角度のタプル」の組を **順番に** 並べたリストです。
- **各要素**  
  `("home", (0.0, 0.8, 0.2, 0.0, 0.0, 0.0))` のように、  
  - 1 つ目: ログやデバッグ用の **名前**、  
  - 2 つ目: **joint1〜6 の目標角度（rad）** です。  
  この 2 つ目が、これから MoveIt に送る「目標」そのものです。
- **順序の意味**  
  スクリプトはこのリストを **上から 1 つずつ** 処理します。  
  現在のシーケンスは**整理版**で、ほぼ同じ位置にしか動かないポーズ（shoulder_60/90/100、arm_down/arm_forward や `ready` の重複など）を省略しています。  
  home → vertical → shoulder_30 → elbow_bent → arm_reach → arm_folded → ready → tilt_down/up → wrist_roll/yaw → point_right/left → pick_pose → place_pose → home という流れで、肩を曲げる→手先の向き→左右→ピック/プレース が分かりやすく見えるようにしてあります。

---

### A.6 ループ：各ポーズへ「移動」して軌道を再生

```python
    demo.get_logger().info("=== 6-DoF Named Poses Demo ===")
    ...
    for i, (name, joints) in enumerate(sequence, 1):
        demo.get_logger().info(f"[{i}/{len(sequence)}] Moving to: {name}")
        ok = demo.move_to_joint_positions(*joints)
        if not ok:
            demo.get_logger().warn(f"  Failed to reach '{name}', continuing...")
        time.sleep(1.5)
```

- **`for i, (name, joints) in enumerate(sequence, 1):`**  
  `sequence` の要素を 1 つずつ取り出し、  
  - `i`: 何番目か（1 始まり）、  
  - `name`: ポーズ名（例: `"home"`）、  
  - `joints`: そのポーズの 6 つの関節角度のタプル、  
  としてループします。
- **`demo.move_to_joint_positions(*joints)`**  
  ここが **MoveIt とのやり取りの中心** です。  
  - `*joints` でタプルを展開し、`move_to_joint_positions(j1, j2, j3, j4, j5, j6)` として渡しています。  
  - `moveit_advanced_demo.py` 側では、この 6 つの値を **MoveGroup の Goal** の `goal_constraints`（6 本の JointConstraint）に詰めて、**MoveGroup アクション** で送信します。  
  - MoveIt は「現在の joint_states」から「この目標」まで衝突回避で軌道を計画し、**計画された軌道** を返します。  
  - 返ってきた軌道は **plan_only** なので実機は動かさず、代わりに `moveit_advanced_demo.py` 内で **時間スケールをかけて `/joint_states` にパブリッシュ** し、RViz 上のロボットが動いて見えるようにしています。  
  - 戻り値は「計画・再生がうまくいったか」の bool です。
- **`if not ok: ... warn(...)`**  
  計画失敗やエラーで `ok` が False のときは警告を出しますが、**ループは止めず** 次のポーズに進みます。
- **`time.sleep(1.5)`**  
  1 つのポーズの「移動」が終わってから、次のポーズに進むまで **1.5 秒** 待ちます。  
  見た目の区切りと、MoveIt 側の状態が落ち着くのを待つための余裕です。

---

### A.7 終了処理

```python
    demo.get_logger().info("=== Demo finished ===")
    demo.destroy_node()
    rclpy.shutdown()
```

- **`demo.destroy_node()`**  
  MoveItAdvancedDemo のノード（タイマー、パブリッシャ、アクションクライアントなど）をきちんと片付けます。
- **`rclpy.shutdown()`**  
  ROS 2 のコンテキストを終了させ、プロセスがきれいに抜けるようにします。

---

### A.8 実行の流れのまとめ（1 ポーズあたり）

1. **run_named_poses.py** が `move_to_joint_positions(j1, j2, j3, j4, j5, j6)` を呼ぶ。  
   → そのときの「目標」は、例えば `("vertical", (0,0,0,0,0,0))` の右側のタプル。
2. **moveit_advanced_demo.py** が、その 6 つの値を **MoveGroup.Goal** の関節制約に詰め、**MoveGroup アクション** で move_group に送る。
3. **move_group**（MoveIt）が現在の `/joint_states` を「現在状態」として、目標の関節角度まで軌道を計画し、**planned_trajectory** を返す。
4. **moveit_advanced_demo.py** が、その軌道に `trajectory_time_scale` をかけて `/joint_states` に流し、RViz でアームが動いて見えるようにする。
5. **run_named_poses.py** は `ok` を受け取り、`time.sleep(1.5)` のあと次の `(name, joints)` で 1 に戻る。

つまり、**run_named_poses.py** は「どのポーズを、どの順で送るか」を決めているだけで、実際の「MoveIt への送信」と「軌道の再生」はすべて **moveit_advanced_demo.py** の `MoveItAdvancedDemo` が行っています。  
この 2 つのファイルの役割を分けておくと、コードの見通しがよくなります。
