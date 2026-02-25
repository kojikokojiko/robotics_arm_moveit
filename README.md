# MoveIt 2 を使った 6-DoF ロボットアーム可視化・プランニング環境

## 概要

このプロジェクトは、**macOS 上の Docker コンテナ**で **ROS 2 Jazzy + MoveIt 2** を動かし、

- 6-DoF ロボットアームの **URDF/SRDF モデル**
- MoveIt 2 による **軌道計画（MoveGroup アクション）**
- `/joint_states` を使った **軌道の再生と可視化**
- **Foxglove Studio / RViz** 上での 3D 表示

を一通り試すための最小構成のワークスペースです。

MoveIt の詳細な仕組みや、このリポジトリのコードが何をしているかは  
`MOVEIT_TECHNICAL_GUIDE.md` にメモとしてまとめています。

---

## プロジェクト構成

### ディレクトリ構成

```text
.
├── README.md
├── docker-compose.yml          # ROS 2 コンテナ起動用
├── Dockerfile                  # ros:jazzy ベースの開発用イメージ
├── INSTALL_MOVEIT.md           # MoveIt パッケージ手動インストール手順
├── MOVEIT_TECHNICAL_GUIDE.md   # MoveIt の仕組みとこのプロジェクトでの使い方メモ
└── workspace/
    └── src/
        ├── robot_description/  # アームの URDF/xacro（6-DoF）
        ├── moveit_config/      # MoveIt 設定一式（SRDF, joint_limits など）
        ├── moveit_advanced_demo.py   # MoveGroup アクションで軌道を取り出し /joint_states で再生するノード
        ├── run_named_poses.py        # 名前付きポーズ＋障害物追加を含むデモのエントリポイント
        ├── find_joint_states_publishers.sh
        ├── find_and_stop_joint_state_publisher.sh
        └── stop_joint_state_publisher.sh
```

- **robot_description**: 6 関節アームの URDF/xacro モデル
- **moveit_config**:  
  - `config/2dof_arm.srdf`: 実際には 6-DoF アーム用の SRDF（プランニンググループ `arm_group` と多数の名前付きポーズ定義）
  - `joint_limits.yaml`, `kinematics.yaml`, `ompl_planning.yaml` などの MoveIt 設定
- **moveit_advanced_demo.py**:  
  MoveGroup アクションを叩いて軌道を取得し、自前で `/joint_states` をパブリッシュして可視化するノード
- **run_named_poses.py**:  
  SRDF で定義した **名前付きポーズのシーケンス**を順番に MoveIt に送り、障害物を 1 個置いた状態で軌道を計画・再生するスクリプト  
  → 日常的にはこちらを実行すればデモが動きます。

MoveIt の設定内容や `run_named_poses.py` / `moveit_advanced_demo.py` の役割分担は  
`MOVEIT_TECHNICAL_GUIDE.md` に詳しく書いてあります。

---

## クイックスタート

### 1. Docker コンテナのビルド・起動

ホスト（macOS）側で:

```bash
docker-compose up -d --build
```

起動後、以下でコンテナに入ります。

```bash
docker-compose exec ros2 bash
```

以降、特に断りがなければ「コンテナ内」での操作です。

---

### 2. MoveIt パッケージのインストール（初回のみ）

コンテナ内で、MoveIt 関連のパッケージをインストールします。

```bash
apt-get update
apt-get install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-core \
  ros-jazzy-moveit-ros-planning-interface \
  ros-jazzy-moveit-ros-move-group \
  ros-jazzy-moveit-planners \
  ros-jazzy-moveit-planners-ompl \
  ros-jazzy-moveit-kinematics \
  ros-jazzy-moveit-visual-tools \
  ros-jazzy-moveit-common
```

依存関係のエラーやネットワークエラーが出た場合のリトライ手順などは  
**`INSTALL_MOVEIT.md`** を参照してください。

---

### 3. ROS 2 ワークスペースのビルド

```bash
cd /workspace
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

以降、新しいシェルを開くたびに:

```bash
cd /workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

を実行しておきます。

---

### 4. MoveIt + デモの起動

#### ターミナル 1: Foxglove Bridge（任意）

```bash
docker-compose exec ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 run foxglove_bridge foxglove_bridge"
```

Foxglove Studio から `ws://localhost:8765` へ接続することで、トピックや `/joint_states`、3D 表示などをブラウザで確認できます。

#### ターミナル 2: MoveIt（move_group + RViz）

```bash
docker-compose exec ros2 bash
cd /workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch moveit_config demo.launch.py
```

これにより、

- MoveIt の `move_group` ノード
- RViz によるロボットモデルと PlanningScene の表示

が立ち上がります。

#### ターミナル 3: 名前付きポーズ＋障害物デモ（推奨）

```bash
docker-compose exec ros2 bash
cd /workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/run_named_poses.py
```

このスクリプトは:

- PlanningScene に **直方体の障害物を 1 つ追加**
- SRDF で定義した **名前付きポーズのシーケンス**を、MoveGroup アクションで順番に目標として送信
- 返ってきた軌道を **`/joint_states`** に流し直して、RViz / Foxglove 上でアームの動きを再生

するデモです。MoveIt 自体は「plan_only」で動き、実機はつながっていない前提です。

`run_named_poses.py` / `moveit_advanced_demo.py` の中身やシーケンスの意図は  
`MOVEIT_TECHNICAL_GUIDE.md` の付録を参照してください。

---

### 5. Foxglove Studio からの確認（任意）

1. ブラウザで `https://studio.foxglove.dev` を開く
2. 「Open Connection」→「Foxglove WebSocket」→ `ws://localhost:8765`
3. 以下のようなパネルを追加
   - **TF** パネル
   - **3D** パネル（`/joint_states` によるロボット表示、`/obstacle_markers` を 3D オブジェクトとして表示）

`run_named_poses.py` は Foxglove 用に `/obstacle_markers`（オレンジ色の箱）もパブリッシュしているため、  
障害物の位置と、MoveIt がそれを避ける軌道の両方をブラウザ上で確認できます。

---

## MoveIt 設定ファイルの場所

`workspace/src/moveit_config/config/` に MoveIt 関連の設定ファイルがあります。

- **2dof_arm.srdf**  
  実際には 6-DoF アーム用の SRDF です。  
  - プランニンググループ `arm_group`
  - エンドエフェクタ `tool_link`
  - 多数の **名前付きポーズ (group_state)**  
    例: `home`, `ready`, `vertical`, `elbow_bent`, `tilt_down`, `pick_pose`, `place_pose` など
- **joint_limits.yaml**: 各関節の位置 / 速度 / 加速度リミット
- **kinematics.yaml**: KDL ベースの運動学プラグイン設定
- **ompl_planning.yaml** / **planning_pipelines.yaml**: OMPL を用いたプランニングの設定
- **moveit_controllers.yaml**: MoveIt 側から見るコントローラ設定

MoveIt の概念（URDF / SRDF / PlanningScene / MoveGroup アクションなど）は  
`MOVEIT_TECHNICAL_GUIDE.md` に図やサンプルと一緒にまとめています。

---

## MoveIt 設定のカスタマイズ

- **プランニンググループの変更**  
  `moveit_config/config/2dof_arm.srdf` の `<group>` / `<group_state>` を編集します。

- **関節制限の変更**  
  `moveit_config/config/joint_limits.yaml` の各ジョイントの範囲を変更します。

- **プランナーの変更**  
  `moveit_config/config/ompl_planning.yaml` を編集し、使用するプランナー（RRT*, PRM など）やパラメータを調整します。

---

## トラブルシューティング

- **MoveIt（demo.launch.py）が立ち上がらない**  
  - `cd /workspace && colcon build` が通っているか
  - `source /opt/ros/jazzy/setup.bash` と `source install/setup.bash` を実行しているか

- **MoveGroup アクションに接続できない / デモがすぐ終了する**  
  - 先に `ros2 launch moveit_config demo.launch.py` を立ち上げているか
  - `ros2 action list -t | grep MoveGroup` で MoveGroup アクションが見えているか

- **軌道計画が頻繁に失敗する**  
  - 目標姿勢がジョイントリミット内か（`joint_limits.yaml`）
  - SRDF の group_state と `run_named_poses.py` の姿勢定義がずれていないか
  - 障害物の位置が極端に近すぎないか（`run_named_poses.py` 内の定数）

- **Foxglove Studio からつながらない**  
  - Foxglove Bridge が起動しているか  
    `ros2 run foxglove_bridge foxglove_bridge`
  - `docker-compose.yml` で `8765:8765` が公開されているか

---

## 参考資料

- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [Foxglove Studio](https://foxglove.dev/)
- [OMPL Documentation](https://ompl.kavrakilab.org/)

---

## ライセンス

MIT License
