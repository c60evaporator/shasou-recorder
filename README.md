# shasou-recorder

[![CI](https://github.com/c60evaporator/shasou-recorder/actions/workflows/ci.yml/badge.svg)](https://github.com/c60evaporator/shasou-recorder/actions/workflows/ci.yml)

shasou (車窓, *shasō*) エコシステムの車載収録ツールキット。Jetson 等でセンサや
CARLA に接続し、End-to-end 自動運転向けのデータを ROS 2 / MCAP で収録する。

- **規律**: `core/` は純 Python で ROS に依存しない。`ros/` → `core/` の一方向依存のみ
  (CLAUDE.md §1.1。`scripts/check_dependencies.py` が CI で機械検証する)
- **契約の正は shasou-core**: トピック名・型・QoS・manifest スキーマは core が定義し、
  recorder は従うだけ (CLAUDE.md §1.2)
- 収録の状態機械: `core/session.py` / データレイアウト: `core/layout.py` /
  成果物: `core/manifest.py`・`core/events.py`・`core/catalog.py`

## 開発セットアップ

shasou-core は PyPI 未公開なので、**先に editable で入れる** (このリポジトリと
並べてチェックアウトしている前提):

```bash
pip install -e ../shasou-core     # 契約の正。先に入れる
pip install -e ".[dev]"
```

```bash
pytest                                   # テスト (ros マーカーは既定で除外)
python scripts/check_dependencies.py     # core/ が ROS に依存していないことの検証
```

## ROS 2 環境について

`rclpy` / `rosbag2_py` は apt で入る ROS 2 のパッケージで、pip の依存にできない
(このため ros extra は用意していない)。`ros/` 配下を動かすには **ROS 2 Humble を
source した環境**が要る:

```bash
source /opt/ros/humble/setup.bash
pytest -m ros                            # ROS 2 が要るテストだけを回す
```

`ros/converters.py` は [shasou_msgs](https://github.com/c60evaporator/shasou-msgs)
(ROS メッセージ定義) にも依存する。ワークスペースでビルドして source していないと、
そのテストは skip される (エラーにはならない)。

ROS 2 を source したシェルでは、ROS 側の pytest プラグイン (launch_testing 系) が
新しい pytest とフック定義が合わず pytest 自体が起動しないことがある。このリポジトリは
pytest プラグインを使わないので、自動ロードを切って回せばよい:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest
```

`core/` の開発とテストは ROS 2 無しで完結する。これは意図した設計で、将来 ROS 以外の
ミドルウェアからの収録や、MCAP を直接読む変換器へ `core/` をそのまま流用するため。
