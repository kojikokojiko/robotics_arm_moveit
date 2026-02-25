# MoveItを使ったロボットアーム制御プロジェクト

## 概要

このプロジェクトは、**macOS上のDockerベース開発環境**で**ROS 2 Jazzy**を使用して、**MoveIt 2**による**2自由度ロボットアーム制御システム**を構築するプロジェクトです。

システムは**MoveIt 2**を使用してモーションプランニングと実行を行い、`foxglove_bridge`を使用して**Foxglove**経由で内部状態（TF、JointState）を可視化します。

### プロジェクトの構成

- **robot_description**: 2自由度ロボットアームのURDF/xacroモデル
- **moveit_config**: MoveIt設定ファイル（SRDF、joint_limits、kinematics等）
- **可視化**: Foxglove Studioを使用したTFツリーと3Dビューの表示

### 主な機能

- ✅ 2自由度ロボットアームのURDF/xacroモデル
- ✅ MoveIt 2によるモーションプランニング
- ✅ TFツリーの可視化（Foxglove）
- ✅ 3Dビューでのロボットアーム表示（Foxglove）
- ✅ JointStateのパブリッシュと可視化
- ✅ MoveIt Python API（moveit_commander）の使用例

### 背景・制約

- **ホストOS**: macOS
- **制約**: macOSではLinuxのGUIツール（RViz、Gazebo等）が直接動作しない
- **解決策**: GUIツールとしてFoxglove Studioを使用（WebベースのためMacでも利用可能）
- **ROS 2**: Jazzy Jalisco
- **MoveIt**: MoveIt 2

### 要件

#### 1. Docker環境
- ROS2 Jazzyが動作するDockerコンテナ
- MoveIt 2パッケージがインストール済み
- コンテナ内でROS2ノードの実行が可能
- ネットワーク設定により、ホストマシンや他のコンテナと通信可能

#### 2. Foxglove統合
- ROS2のトピックをFoxglove Studioに接続可能
- Foxglove BridgeまたはFoxglove WebSocketサーバーを使用
- コンテナ内のROS2トピックを外部（Foxglove Studio）に公開

#### 3. 開発環境
- コードの編集・デバッグが容易
- ボリュームマウントにより、ホスト側のコードをコンテナ内で実行可能

### 技術スタック

- **ROS 2**: Jazzy Jalisco
- **MoveIt**: MoveIt 2
- **Docker**: Docker Desktop for Mac
- **GUIツール**: Foxglove Studio（Webアプリケーション）
- **通信**: Foxglove Bridge（ROS2 ↔ Foxglove Studio）
- **プランニング**: OMPL（Open Motion Planning Library）

### 構成要素

#### Docker Compose構成
- ROS2コンテナ（ベースイメージ: `ros:jazzy`）
- MoveIt 2パッケージがインストール済み
- Foxglove BridgeはROS2コンテナ内で直接実行
- ポート8765をホストに公開（Foxglove Studio接続用）
- `workspace/src`のみをホストと共有（build/install/logはコンテナ内のみ）

#### ファイル構成
```
.
├── README.md                 # このファイル
├── docker-compose.yml        # Docker Compose設定
├── Dockerfile                # ROS2 Dockerイメージ定義
├── .dockerignore            # Dockerビルド時の除外ファイル
├── .gitignore               # Git除外ファイル
└── workspace/               # ROS2ワークスペース
    └── src/                  # ソースコード（ホストと共有）
        ├── robot_description/  # ロボットモデル（URDF/xacro）
        │   ├── urdf/
        │   │   └── arm.urdf.xacro
        │   └── launch/
        │       └── display.launch.py
        ├── moveit_config/      # MoveIt設定
        │   ├── config/
        │   │   ├── 2dof_arm.srdf
        │   │   ├── joint_limits.yaml
        │   │   ├── kinematics.yaml
        │   │   ├── moveit_controllers.yaml
        │   │   ├── planning_pipelines.yaml
        │   │   └── ompl_planning.yaml
        │   └── launch/
        │       ├── move_group.launch.py
        │       └── demo.launch.py
        ├── moveit_demo.py           # MoveItデモスクリプト
        ├── moveit_simple_demo.py    # moveit_commander使用例
        ├── moveit_advanced_demo.py  # MoveIt高度デモ（衝突回避、複雑な動作計画）
        ├── move_arm_simple.py       # 基本的な動作スクリプト（直接/joint_statesにパブリッシュ）
        └── move_arm_with_moveit.py  # MoveItアクションAPI使用例
```

## クイックスタート

### 1. コンテナのビルドと起動

```bash
# Dockerイメージをビルドしてコンテナを起動
docker-compose up -d --build
```

### 2. MoveItパッケージのインストール（コンテナ内で手動実行）

```bash
# コンテナに入る
docker-compose exec ros2 bash

# MoveItパッケージをインストール
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

# 注意: moveit_commanderはROS 2 Jazzyでは別パッケージとして提供されていない可能性があります
# MoveItの基本機能（move_groupノード、プランニング等）は上記のパッケージで動作します

**注意**: インストールに失敗する場合は、リトライしてください：
```bash
apt-get update
apt-get install -y --fix-broken
apt-get install -y ros-jazzy-moveit*
```

### 3. ワークスペースのビルド

```bash
# ワークスペースをビルド
cd /workspace
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### 4. MoveItデモの起動（3つのターミナルで実行）

**ターミナル1: Foxglove Bridge**
```bash
docker-compose exec ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 run foxglove_bridge foxglove_bridge"
```

**ターミナル2: MoveItデモ（robot_state_publisher + move_group）**
```bash
docker-compose exec ros2 bash
cd /workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch moveit_config demo.launch.py
```

**ターミナル3: アームを動かすスクリプト**

基本的な動作（直接`/joint_states`にパブリッシュ）:
```bash
docker-compose exec ros2 bash
cd /workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/move_arm_simple.py
```

MoveItを使った高度なデモ（衝突回避、複雑な動作計画）:
```bash
docker-compose exec ros2 bash
cd /workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/moveit_advanced_demo.py
```

**注意**: `moveit_advanced_demo.py`は、MoveItのアクションサーバーが起動している必要があります（ターミナル2で`demo.launch.py`を実行していること）。

### 5. Foxglove Studioで確認

1. [https://studio.foxglove.dev](https://studio.foxglove.dev) にアクセス
2. 「Open Connection」→「Foxglove WebSocket」→`ws://localhost:8765`
3. 「TF」パネルと「3D」パネルを追加してロボットアームを確認

---

## MoveItの使い方

### MoveIt設定ファイル

MoveIt設定は`moveit_config/config/`ディレクトリにあります：

- **2dof_arm.srdf**: セマンティックロボット記述（プランニンググループ、エンドエフェクター定義）
- **joint_limits.yaml**: 関節の制限（速度、加速度、位置範囲）
- **kinematics.yaml**: 運動学ソルバーの設定
- **moveit_controllers.yaml**: MoveItで使用するコントローラーの設定
- **planning_pipelines.yaml**: プランニングパイプラインの設定
- **ompl_planning.yaml**: OMPLプランナーの設定

### MoveIt Python API

**注意**: `moveit_commander`はROS 2 Jazzyでは利用できない可能性があります。MoveItの基本機能（move_groupノード、プランニング等）はC++ APIやROS 2のアクション/サービスインターフェースを通じて使用できます。

PythonからMoveItを使用する場合は、以下の方法があります：
- ROS 2のアクションクライアントを使用（`moveit_msgs.action.MoveGroup`）
- C++ APIをPythonから呼び出す（pybind11等を使用）
- moveit_commanderをソースからビルドする

### MoveIt C++ API

C++でMoveItを使用する場合は、`moveit_ros_move_group`パッケージの`MoveGroupInterface`を使用します。

---

## セットアップ手順（詳細）

### 前提条件

- Docker Desktop for Macがインストールされていること
- インターネット接続があること

### 1. 環境変数の設定（オプション）

`.env`ファイルを作成して、必要に応じて環境変数を設定します：

```bash
# .envファイルを作成（オプション）
cat > .env << EOF
ROS_DOMAIN_ID=0
EOF
```

**注意**: `ROS_DOMAIN_ID`はデフォルトで0が設定されているため、`.env`ファイルは必須ではありません。

### 2. Dockerイメージのビルドとコンテナの起動

```bash
# Docker Composeでコンテナをビルド・起動
docker-compose up -d --build

# ROS2コンテナに入る
docker-compose exec ros2 bash
```

### 3. MoveItパッケージのインストール（コンテナ内で手動実行）

コンテナ内で以下のコマンドを実行：

```bash
# MoveItパッケージをインストール
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

# moveit_commanderはROS 2 Jazzyでは利用できない可能性があります
# MoveItの基本機能は上記のパッケージで動作します

# インストールに失敗する場合は、以下を試してください：
# apt-get install -y --fix-broken
# apt-get install -y ros-jazzy-moveit*
```

### 4. ROS2ワークスペースのビルド

コンテナ内で以下のコマンドを実行：

```bash
# ワークスペースディレクトリに移動
cd /workspace

# ROS2環境をセットアップ
source /opt/ros/jazzy/setup.bash

# ワークスペースをビルド
colcon build

# ビルドしたパッケージを環境に追加
source install/setup.bash
```

### 5. MoveItの動作確認

```bash
# MoveItデモを起動
ros2 launch moveit_config demo.launch.py
```

別のターミナルで：

```bash
# 関節状態を確認
ros2 topic echo /joint_states

# MoveItの状態を確認
ros2 topic list | grep move_group
```

---

## MoveIt高度デモ

`moveit_advanced_demo.py`は、MoveItの効果を最大限実感できるデモです。

### デモの内容

1. **障害物の追加と削除**: Planning Sceneに障害物を動的に追加/削除
2. **衝突回避**: MoveItが自動で衝突を回避する経路を計画
3. **複雑な動作計画**: ピックアンドプレース風の動作シーケンス
4. **最適化された軌道**: 障害物の有無による経路の違いを実感

### 実行方法

```bash
# ターミナル1: MoveItを起動
ros2 launch moveit_config demo.launch.py

# ターミナル2: 高度デモを実行
cd /workspace
source install/setup.bash
python3 src/moveit_advanced_demo.py
```

詳細は [`MOVEIT_ADVANCED_DEMO.md`](MOVEIT_ADVANCED_DEMO.md) を参照してください。

---

## MoveIt高度デモ

`moveit_advanced_demo.py`は、MoveItの効果を最大限実感できるデモです。

### デモの内容

1. **障害物の追加と削除**: Planning Sceneに障害物を動的に追加/削除
2. **衝突回避**: MoveItが自動で衝突を回避する経路を計画
3. **複雑な動作計画**: ピックアンドプレース風の動作シーケンス
4. **最適化された軌道**: 障害物の有無による経路の違いを実感

### 実行方法

```bash
# ターミナル1: MoveItを起動
ros2 launch moveit_config demo.launch.py

# ターミナル2: 高度デモを実行
cd /workspace
source install/setup.bash
python3 src/moveit_advanced_demo.py
```

詳細は [`MOVEIT_ADVANCED_DEMO.md`](MOVEIT_ADVANCED_DEMO.md) を参照してください。

---

## MoveIt設定のカスタマイズ

### プランニンググループの変更

`moveit_config/config/2dof_arm.srdf`を編集して、プランニンググループを変更できます。

### 関節制限の変更

`moveit_config/config/joint_limits.yaml`を編集して、関節の制限を変更できます。

### プランナーの変更

`moveit_config/config/ompl_planning.yaml`を編集して、使用するプランナーを変更できます。

---

## トラブルシューティング

### MoveItが起動しない

1. ワークスペースが正しくビルドされているか確認：
   ```bash
   colcon build
   source install/setup.bash
   ```

2. MoveItパッケージがインストールされているか確認：
   ```bash
   apt list --installed | grep moveit
   ```

### プランニングが失敗する

1. 関節制限が正しく設定されているか確認（`joint_limits.yaml`）
2. SRDFファイルが正しく設定されているか確認（`2dof_arm.srdf`）
3. 運動学ソルバーが正しく設定されているか確認（`kinematics.yaml`）

### Foxglove Studioに接続できない

1. Foxglove Bridgeが起動しているか確認：
   ```bash
   ros2 run foxglove_bridge foxglove_bridge
   ```

2. ポート8765が正しく公開されているか確認：
   ```bash
   docker-compose ps
   ```

---

## 参考資料

- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [Foxglove Studio](https://foxglove.dev/)
- [OMPL Documentation](https://ompl.kavrakilab.org/)

---

## ライセンス

MIT License
