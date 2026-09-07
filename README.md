# wheel_lidar_localization

ROS 2 Humble 用の車輪オドメトリ + PCL NDT 自己位置推定パッケージです。
`lidar_localization_ros2` / FAST-LIO への依存はありません。

## ビルド

```bash
source /opt/ros/humble/setup.bash
cd /home/takumi/ros2_ws
colcon build --symlink-install --packages-select wheel_lidar_localization --parallel-workers 1
source install/setup.bash
```

依存: rclcpp、tf2_ros、PCL、pcl_conversions、標準 ROS メッセージ。
テストには ament_cmake_gtest を使用します。

## 今回の bag で起動

ターミナル1:

```bash
source /opt/ros/humble/setup.bash
source /home/takumi/ros2_ws/install/setup.bash
ros2 launch wheel_lidar_localization localization.launch.py use_sim_time:=true
```

ターミナル2:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/takumi/bags/tsudanuma_3d_data_2 --clock
```

RViz の `2D Pose Estimate` で、現在再生中の地点の X/Y/向きを指定します。
地図は初期位置やセンサ入力がなくても表示されます。
点群表示に必要な `map -> odom` は、最初の照合が採用された後に配信されます。
地図が画面外ならマウスのホイールで縮小して探してください。
この bag は約45分・19GBです。初期位置を指定する間はスペースキーで再生を一時停止できます。
地図と開始地点の対応は既知とは仮定していません。オドメトリの座標を地図上の初期位置として
自動使用することはありません。

`config/rfans_wheel.yaml` の地図:
`/home/takumi/map/tsudanuma_merged_map/merged.pcd`

bag の実際のトピックは `/surestar_points` と `/odometry/filtered` です。
実機側が別名なら引数で指定してください:

```bash
ros2 launch wheel_lidar_localization localization.launch.py \
  cloud_topic:=/rfans_points odom_topic:=/odometory/filterd
```

実機でトピックを変えた場合は RViz の RFANS Scan の Topic も変更してください。
その他の引数: `params_file`（YAMLファイルのフルパス）、`map_path`、`rviz:=false`、
`map_frame`、`odom_frame`、`base_frame`、`rviz_config`。
フレームを変更する場合は RViz の Fixed Frame / Target Frame も合わせてください。

## 入出力と TF

| 入出力 | トピック | 型 |
| --- | --- | --- |
| 点群入力 | `/surestar_points` | PointCloud2（XYZ float32/float64必須） |
| 車輪オドメトリ入力 | `/odometry/filtered` | Odometry（poseを使用） |
| 初期位置入力 | `/initialpose` | PoseWithCovarianceStamped（map） |
| 地図表示 | `/localization/map` | PointCloud2（Transient Local） |
| 採用した姿勢 | `/localization/pose` | PoseWithCovarianceStamped |
| 状態 | `/localization/status` | DiagnosticArray |

ノードは `map -> odom` だけを配信します。外部または bag が `odom -> base_link` と
`base_link -> 点群frame_id` を配信する必要があります。Odometry は header.frame_id=odom、
child_frame_id=base_link とし、外部の同名 TF と同じ軌跡を表す必要があります。
他の自己位置推定ノードと同時に同じ map->odom を配信しないでください。

## 処理と失敗時の動作

- 地図: PCD 読み込み → 有限座標のみ抽出 → VoxelGrid → 最低点数検査。元ファイルは変更しません。
- 点群: 有効XYZと距離フィルタ → base_linkへ変換 → VoxelGrid → 周辺地図とNDT照合。
- 予測: 点群時刻を挟む Odometry.pose を線形補間 + SLERP。外挿はしません。
- 初回: `/initialpose` の X/Y/yaw と initial_z/roll/pitch を使用。以後は6自由度照合。
  完全な6自由度の初期姿勢を送る場合は `initial_pose_use_2d: false` にします。
- ワーカーは最新点群1件を処理し、ROS のオドメトリ受信を止めません。
- 収束、有限変換、近傍点距離RMSE、対応点割合、予測からの補正量で採否判定します。
  RMSE は correspondence_distance 内の対応点だけで計算するため、対応点割合も必須です。
- 不採用なら map->odom を保持。連続失敗または補正途絶で LOST。再度 `/initialpose` が必要です。
- LOST 中も最後の補正TFは配信しますが、位置の信頼性は保証しません。制御側は status を参照してください。
- オドメトリの急変/時刻リセット時は補正を無効化し、TF配信を停止します。
  過去に配信したTFは受信側キャッシュにしばらく残るため、状態も確認してください。
- TF定周期配信は同じ map->odom を再配信します。古い地図上姿勢と最新odomを合成しません。
- 初期位置更新中に古い照合が完了しても世代番号で破棄します。

診断例:

```bash
ros2 topic echo /localization/status --once --qos-durability transient_local
ros2 run tf2_ros tf2_echo map base_link
```

## 制約と検証

初版は点ごとの移動ひずみ補正、IMU融合、自動全域再探索、地図更新を行いません。
RFANS の header timestamp を基準に扱い、必要なら scan_time_offset で一定オフセットを指定します。
移動ひずみや時刻の定義は、旋回時の実データで検証が必要です。
出力共分散は設定した標準偏差の二乗を対角に入れた値です。NDTから推定した共分散ではなく、
入力初期姿勢の共分散も探索範囲には使用しません。
単一のNDT解と距離閾値による判定なので、似た形状への誤収束や幾何学的な退化を完全には検出できません。
パラメータは初期値であり、実機での精度・リアルタイム性を保証する値ではありません。

```bash
cd /home/takumi/ros2_ws
colcon test --packages-select wheel_lidar_localization --event-handlers console_direct+
colcon test-result --verbose
```

テスト: 時刻補間・回転の折り返し・時刻逆行・不正入力・オドメトリ急変、TF合成、
照合不採用と再初期化、既知の変換を与えた非対称3D点群のNDT照合。

合成データによるROS結合テスト（専用ROSドメインを使用）:

```bash
source /opt/ros/humble/setup.bash
python3 /home/takumi/ros2_ws/src/wheel_lidar_localization/test/test_node_integration.py \
  --executable /home/takumi/ros2_ws/install/wheel_lidar_localization/lib/wheel_lidar_localization/localization_node
```

2026-09-07 の検証結果:

- ROS 2 Humble / PCL 1.12 でビルド成功、C++単体テスト8件成功。
- ROS結合テスト成功: 地図の遅延購読、初期位置、時刻補間、センサTF、NDT姿勢出力、
  不採用時のTF保持、LOST、再初期化を確認。
- 実地図 `merged.pcd`: 2,105,479点を読み込み、0.2m間引き後2,089,714点。
  表示用0.4m間引き後972,335点を `map` フレームで配信。
- 指定bagの冒頭約12秒で点群116フレーム、オドメトリ、外部TFの受信を確認。
  オドメトリのフレーム不一致・不正入力による拒否は0件。
- 実bagでは地図上の開始姿勢を未指定のため、WAITING_INITIAL_POSEまでの入力検証。
  実bagでの位置精度、追跡継続性、NDT処理時間は未検証。45分全体の再生も未実施。
