# ROS2 Humble Setup Manual

## OSのバージョン確認

```bash
lsb_release -a
```

出力例
```text
Distributor ID: Ubuntu
Description:    Ubuntu 22.04.x LTS
Release:        22.04
Codename:       jammy
```

## パッケージの更新

```bash
sudo apt update
sudo apt upgrade
```

## 必要パッケージのインストール

```bash
sudo apt install software-properties-common curl
```

- `software-properties-common`: APTリポジトリの管理ツール
- `curl`: HTTP・HTTPS経由でファイルを取得するツール

## UbuntuのUniverseリポジトリの有効化

```bash
sudo add-apt-repository universe
```

## ROS2公式リポジトリの登録

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
```

取得したバージョンの確認
```bash
echo "$ROS_APT_SOURCE_VERSION"
```

インストールパッケージのダウンロード
```bash
curl -L -o /tmp/ros2-apt-source.deb \
"https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
```

ダウンロードしたパッケージをインストール
```bash
sudo dpkg -i /tmp/ros2-apt-source.deb
```

## ROS2 Humbleのインストール

パッケージ情報の更新
```bash
sudo apt update
```

ROS2 Humble Desktopをインストール
```bash
sudo apt install ros-humble-desktop
```

## 開発ツールのインストール

```bash
sudo apt install ros-dev-tools
```

基本ツール
```bash
sudo apt install \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

Nav2
```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup
```

点群処理・座標変換
```bash
sudo apt install \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf2-tools \
  ros-humble-tf2-geometry-msgs
```

センサデータの可視化・解析
```bash
sudo apt install \
  ros-humble-rqt \
  ros-humble-rqt-graph \
  ros-humble-rqt-image-view \
  ros-humble-plotjuggler-ros
```

## ROS2環境変数の設定

### ROS2の有効化

```bash
source /opt/ros/humble/setup.bash
```

確認
```bash
echo $ROS_DISTRO
```

### ターミナル起動時に自動で読み込む
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

設定の反映
```bash
source ~/.bashrc
```

## rosdepの初期設定

初期化
```bash
sudo rosdep init
```

データベースの更新
```bash
rosdep update
```

## ROS2ワークスペースの作成

ディレクトリの作成
```bash
mkdir -p ~/ros2_ws/src
```

ビルド
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

ワークスペースの有効化
```bash
source ~/ros2_ws/install/setup.bash
```

自動読み込みの設定
```bash
echo 'if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi' >> ~/.bashrc
```

ワークスペースの設定ファイルが存在する場合だけ読み込む設定
