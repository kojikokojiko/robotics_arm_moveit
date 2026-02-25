# MoveIt手動インストール手順

このドキュメントでは、コンテナ内でMoveItパッケージを手動でインストールする手順を説明します。

## インストール手順

### 1. コンテナに入る

```bash
docker-compose exec ros2 bash
```

### 2. MoveItパッケージのインストール

```bash
# apt-getを更新
apt-get update

# MoveItパッケージをインストール
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
```

### 3. インストールエラーが発生した場合

パッケージのダウンロードエラーが発生する場合は、以下を試してください：

```bash
# 壊れた依存関係を修正
apt-get install -y --fix-broken

# 再度インストールを試みる
apt-get update
apt-get install -y ros-jazzy-moveit*
```

### 4. インストール確認

インストールが完了したら、以下のコマンドで確認できます：

```bash
# MoveItパッケージがインストールされているか確認
dpkg -l | grep moveit

# MoveItコマンドが使用可能か確認
ros2 pkg list | grep moveit

# move_groupノードが利用可能か確認
ros2 run moveit_ros_move_group move_group --help
```

## moveit_commanderについて

**注意**: ROS 2 Jazzyでは、`moveit_commander`は別パッケージとして提供されていない可能性があります。

MoveItの基本機能は以下の方法で使用できます：
- **move_groupノード**: C++/Pythonからアクションインターフェース経由で使用
- **ROS 2アクション**: `moveit_msgs.action.MoveGroup`を使用
- **ROS 2サービス**: MoveItの各種サービスを使用

PythonからMoveItを使用する場合は、ROS 2のアクションクライアントを使用することを推奨します。

## トラブルシューティング

### パッケージが見つからない

```bash
# ROS 2リポジトリが正しく設定されているか確認
apt-cache search ros-jazzy-moveit

# リポジトリを再設定（必要に応じて）
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update
```

### 依存関係のエラー

```bash
# 依存関係を解決
apt-get install -y --fix-broken
apt-get update
apt-get upgrade -y
```

### ネットワークエラー

パッケージのダウンロードに失敗する場合は、時間をおいて再試行してください：

```bash
# 少し待ってから再試行
sleep 30
apt-get update
apt-get install -y ros-jazzy-moveit*
```
