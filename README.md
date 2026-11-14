# ros2AckermannRobotFromScratch

前輪で舵を切るアッカーマン式の台車を、ROS 2 Jazzy と Gazebo（Harmonic）のシミュレーションの中でゼロから一段階ずつ組み立て、最後に Nav2 で目的地まで自律で移動させる学習教材です。WSL2 + Ubuntu 24.04 で作ります。

> **作成中です。** 手順書は、フェーズ0から順に、サンプルを使い捨ての環境で確かめてから足していきます。各フェーズの内容と完了条件は、[学習計画](docs/learning_plan.md) にあります。

## 読む順番

作成した手順書を、読む順番に並べています。まだ無いフェーズの予定は、[学習計画](docs/learning_plan.md) の2節にあります。

| 順番 | 手順書 | 内容 | 前提 |
|---|---|---|---|
| 1 | [フェーズ0](docs/phase0_overview.md) | アッカーマン式の台車と、自律の移動の全体の構成（読み物） | なし |
| 2 | [フェーズ1-1](docs/phase1_1_urdf.md) | URDF で台車の形と関節を書き、RViz2 で見る | フェーズ0（環境構築の手順書は作成中。ROS 2 Jazzy が入っていればよい）。親プロジェクトの [フェーズ2](https://github.com/SanghunIm1991/ros2MinimalPhysicalAi/blob/main/docs/phase2_packages.md) の2-1・2-2・2-4・2-5節と [フェーズ4](https://github.com/SanghunIm1991/ros2MinimalPhysicalAi/blob/main/docs/phase4_launch.md) の4-1節 |
| 3 | [フェーズ1-2](docs/phase1_2_gazebo.md) | 台車を Gazebo に出し、ROS 2 から舵と後輪の関節を動かす | フェーズ1-1（環境構築の手順書は作成中。Gazebo Harmonic と `ros-jazzy-ros-gz` が入っていればよい）。親プロジェクトの節はフェーズ1-1と同じ |
| 4 | [フェーズ2-1](docs/phase2_1_kinematics.md) | `v`・`ω` から左右の舵の角度と後輪の速さを求めるクラスを書き、ROS 2 を使わずに確かめる | フェーズ0の2節、フェーズ1-2。親プロジェクトの [フェーズ2](https://github.com/SanghunIm1991/ros2MinimalPhysicalAi/blob/main/docs/phase2_packages.md) の2-1・2-2・2-4・2-5節 |
| 5 | [フェーズ2-2](docs/phase2_2_drive_node.md) | `cmd_vel` を受けて関節に指令するノードを書き、Gazebo の台車を円を描いて走らせ、半径を測る | フェーズ2-1、フェーズ1-2（Gazebo の設定と中継の設定に足す）。親プロジェクトの [フェーズ3-1](https://github.com/SanghunIm1991/ros2MinimalPhysicalAi/blob/main/docs/phase3_1_pubsub.md) の2節・3節 |
| 6 | [フェーズ3-1](docs/phase3_1_odometry.md) | 関節の状態から位置と向きを積算するオドメトリのノードを書き、`/odom` と TF を出して、真の位置と比べる | フェーズ2-2、フェーズ1-1（TF の木）。親プロジェクトの節はフェーズ2-2と同じ |
| 7 | [フェーズ3-2](docs/phase3_2_ros2_control.md)（任意） | 同じ台車を ros2_control の標準のコントローラで動かし、計算した指令とオドメトリを、自作のものと比べる | フェーズ3-1、フェーズ2-1の2節、フェーズ1-2・2-2（模型と Gazebo の設定に足す）。親プロジェクトの節はフェーズ3-1と同じ。`ros-jazzy-gz-ros2-control` と `ros-jazzy-ros2-controllers` が入っていること。フェーズ4以降は、この手順書に依存しない |

各手順書では、コマンドを実行する手順の直後に「期待する結果」として、表示の例と、その読み方を載せます。実行の環境を用意せずに、読み物として読むこともできます。表示の出どころ（実際に確かめた表示か、仕様から想定した表示か）は、各手順書の冒頭に書きます。時刻・版の数字・パスなど、環境や実行ごとに変わる部分は、その旨を書きます。

## ターミナルの番号

各手順書では、同時に動かすプログラムごとに、別のターミナルを開きます。コマンドの例の `# T1` のような行は、どのターミナルで実行するかを示します。フェーズ1-2以降は、T1 をシミュレーションに使います。T2 以降は、その手順書の作業（`ros2` のコマンド・自作のノード等）に使い、各手順書の中で割り振ります。

| ターミナル | 動かすもの | 使う手順書 |
|---|---|---|
| T1 | シミュレーション（`ros2 launch ackermann_gazebo sim.launch.py`。Gazebo・台車・中継） | フェーズ1-2以降の、シミュレーションを使う手順書（フェーズ1-1では、RViz2 で見るための launch に使う。フェーズ2-1はシミュレーションを使わない。任意のフェーズ3-2では、ros2_control で動かす `sim_control.launch.py` に使う） |
| T2 以降 | その手順書の作業 | 各手順書のとおり |

## 参考にしたもの

この教材の台車の構成（前輪で舵を切る台車に LiDAR と深度カメラを載せ、地図を作って自律で走らせる、という全体の形）は、市販の教育用ロボット Hiwonder 社の MentorPi A1（深度カメラ版、アッカーマンの台車）を参考にしました。この教材は、そのメーカーとは無関係で、公式の教材ではありません。製品の名前・ソフトウェア・寸法・設定の値は使わず、一般的なアッカーマン式の台車として、名前と値を自分で決めて書いています。

## 親プロジェクトとの関係

この教材は、ROS 2 の最低限の構成で、フィジカル AI の開発に必要な技術スタックを一通り身に着ける教材 [ros2MinimalPhysicalAi](https://github.com/SanghunIm1991/ros2MinimalPhysicalAi)（以下「親プロジェクト」）と同じ型で作っています。ROS 2 の基礎（ワークスペースとパッケージ・ノード・トピック・launch 等）は親プロジェクトの手順書の節を参照し、この教材では、アッカーマン式の台車に固有の部分（模型・運動学・オドメトリ・センサ・地図・自律の移動）を扱います。同じ型の派生の教材に、ドローンのシミュレーションを扱う [ros2Px4DroneSim](https://github.com/SanghunIm1991/ros2Px4DroneSim) があります。

## 計画と記録の文書

学習者向けの文書（手順書・学習計画・図）は `docs/` に、教材を作る側の文書（構想・判断の記録・点検の記録など）は `dev/` に置いています。教材を進めるのに `dev/` は必須ではありません。

| 文書 | 内容 |
|---|---|
| [docs/learning_plan.md](docs/learning_plan.md) | 学習計画（各フェーズの内容・前提・完了条件、作る予定の手順書） |
| [dev/idea_origin.md](dev/idea_origin.md) | 構想（目的・ゴール・段階・技術を選んだ理由・先に確かめること） |
| [dev/qa_log.md](dev/qa_log.md) | 判断・承認の記録 |
| [dev/trial_notes.md](dev/trial_notes.md) | 手順書を書く前の予備の実験の記録（台車の駆動の方法の比べ方と結果） |
| [dev/public_release_review.md](dev/public_release_review.md) | 公開前と更新のたびの点検の観点と実施記録 |

## ライセンス

- サンプルコードとコマンド: [Apache License 2.0](LICENSE-APACHE-2.0)
- 文章と図: [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)

詳しくは [LICENSE](LICENSE) を参照してください。著作権者は SanghunIm1991 です。
