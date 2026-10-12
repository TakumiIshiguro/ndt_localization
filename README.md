# ndt_localization

ROS 2 Humble 用の自己位置推定パッケージです。Autoware Core の `localization` を参考にした構成で、
固定の 3D 点群地図に対して LiDAR の NDT 照合を行い、車輪・IMU オドメトリと EKF で融合します。

## 構成

```
 /surestar_points ─▶ ndt_scan_matcher ─▶ /localization/pose_estimator/pose_with_covariance ─┐
                         ▲ /localization/kinematic_state                                      ▼
 /odometry/filtered ─────────────────────────────────────────────────────────▶ ekf_localizer ─▶ TF map→odom
 /initialpose（RViz）─────────────────────────────────────────────────────────────────────────┘
```

ノード名、名前空間、トピック名は `autoware_core_localization.launch.xml` に合わせています。

| ノード（名前空間） | Autoware Core の対応 | 役割 |
| --- | --- | --- |
| `ndt_scan_matcher`（`/localization/pose_estimator`） | `autoware_ndt_scan_matcher` | multigrid ndt_omp による照合 |
| `ekf_localizer`（`/localization/pose_twist_fusion_filter`） | `autoware_ekf_localizer` → robot_localization で代替 | オドメトリの速度と NDT の姿勢を融合し、`map→odom` を配信 |

車輪と IMU の融合は、既存の odom 用 robot_localization（`odom→base_link` と `/odometry/filtered`）が
担います。Autoware の `gyro_odometer` と、`ekf_localizer` が状態に持つ速度の推定がこれに当たります。
そのため、本パッケージでは速度を推定し直しません（REP-105 の 2 段 EKF 構成）。

## ソースと Autoware Core の対応

参照したのは autoware_core（commit `e0a8ab84`、2026-10-08）と autoware_utils（commit `6b89a2e1`）です。
各ファイルの先頭に参照元のパスを書き、移植した関数には `// Ref:` で対応する関数名と差分を書いています。
Autoware にないものには `(this package only)` と付けています。

| 本パッケージ | 参照元（autoware_core） |
| --- | --- |
| `include/.../ndt_scan_matcher/ndt_scan_matcher_core.hpp`、`src/ndt_scan_matcher/ndt_scan_matcher_core.cpp` | `localization/autoware_ndt_scan_matcher/{include/autoware/ndt_scan_matcher,src}/ndt_scan_matcher_core.*` |
| `include/.../ndt_scan_matcher/hyper_parameters.hpp` | `localization/autoware_ndt_scan_matcher/include/autoware/ndt_scan_matcher/hyper_parameters.hpp` |
| `src/ndt_scan_matcher/ndt_scan_matcher_helper.*` | `localization/autoware_ndt_scan_matcher/src/ndt_scan_matcher_helper.*` |
| `include/.../ndt_scan_matcher/ndt_omp/`、`src/ndt_scan_matcher/ndt_omp/` | 同パッケージの `ndt_omp/`（同梱。include パス以外は変更なし） |
| `include/.../localization_util/{smart_pose_buffer,util_func}.hpp`、`src/localization_util/` | `localization/autoware_localization_util/` |
| `include/.../utils_diagnostics/diagnostics_interface.hpp` | autoware_utils `autoware_utils_diagnostics/diagnostics_interface.hpp` |
| `src/map_loader/pointcloud_map_loader.*` | `map/autoware_map_loader/src/pointcloud_map_loader/pointcloud_map_loader.*`（`load_pointcloud_map`） |
| `src/pointcloud_preprocessor/pointcloud_preprocessor.*` | `sensing/autoware_crop_box_filter` と `sensing/autoware_downsample_filters`（voxel grid）の処理をノード内で行う |
| `config/ndt_scan_matcher.param.yaml` | `localization/autoware_ndt_scan_matcher/config/ndt_scan_matcher.param.yaml` |
| `config/ekf_localizer.param.yaml` | robot_localization の形式。役割は `autoware_ekf_localizer` |
| `launch/ndt_localization.launch.py` | `localization/autoware_core_localization/launch/autoware_core_localization.launch.xml` |
| `test/test_{util_func,smart_pose_buffer,ndt_scan_matcher_helper}.cpp` | 各パッケージの同名テスト（テスト名も合わせています） |

## ビルド

```bash
source /opt/ros/humble/setup.bash
cd /home/takumi/ros2_ws
colcon build --symlink-install --packages-select ndt_localization --parallel-workers 1
source install/setup.bash
```

依存: rclcpp、tf2_ros、PCL、pcl_conversions、OpenMP、Boost、robot_localization（実行時）。

## bag での起動

```bash
# ターミナル1
ros2 launch ndt_localization ndt_localization.launch.py use_sim_time:=true
# ターミナル2
ros2 bag play /home/takumi/bags/tsudanuma_3d_data_2 --clock
```

起動時の姿勢は `config/ekf_localizer.param.yaml` の `initial_state` で指定します（x, y, z, roll, pitch, yaw, ...）。
RViz の `2D Pose Estimate` でいつでも上書きできます（`/initialpose` を EKF の `set_pose` に直接渡します）。
RViz の姿勢は z/roll/pitch が 0 です。Autoware は map_height_fitter で z を地図から補いますが、
本パッケージにはないので、地図の地面の高さが 0 から離れている場合は NDT が z を合わせるまで少し時間がかかります。

launch 引数:

| 引数 | 既定値 | 内容 |
| --- | --- | --- |
| `lidar_input_topic` | `/surestar_points` | LiDAR 点群 |
| `vehicle_odometry_input_topic` | （YAML の `odom0`） | odom フレームのオドメトリ |
| `map_path` | （YAML の `map.path`） | PCD 地図 |
| `ndt_scan_matcher_param_path` / `ekf_localizer_param_path` | `config/` 内の各ファイル | |
| `use_sim_time` / `rviz` / `rviz_config` | | |

## 入出力

| トピック | 型 | 内容 |
| --- | --- | --- |
| `/surestar_points` | PointCloud2 | 入力。x/y/z は FLOAT32（`pcl::fromROSMsg` で読む） |
| `/odometry/filtered` | Odometry | 入力。odom→base_link。速度（twist）を ekf_localizer が使う |
| `/initialpose` | PoseWithCovarianceStamped | 入力。map フレーム。ekf_localizer の `set_pose` が直接受け取る |
| `/localization/pose_estimator/pose_with_covariance` | PoseWithCovarianceStamped | 収束した NDT の姿勢。共分散は固定値を map フレームへ回転したもの |
| `/localization/kinematic_state` | Odometry | EKF の出力（map→base_link） |
| `/localization/pose_estimator/debug/loaded_pointcloud_map` | PointCloud2 | 表示用の地図（Transient Local） |
| `/localization/pose_estimator/points_aligned` | PointCloud2 | 照合後の点群（購読者がいる場合だけ配信） |
| `/diagnostics` | DiagnosticArray | `ndt_scan_matcher: scan_matching_status` など |
| TF `map→odom` | | ekf_localizer が配信 |

外部（ロボットまたは bag）から `odom→base_link` と `base_link→LiDAR` の TF が必要です。

## NDT の処理（`ndt_scan_matcher_core.cpp`）

Autoware の `callback_sensor_points_main` と同じ順に処理します。

1. 点群の検査（空でないこと、遅延は警告のみ）
2. 前処理: `pcl::fromROSMsg` → 距離による切り出し → base_link への変換 → VoxelGrid → 最大距離（`required_distance`）の検査。
   切り出しと VoxelGrid は、Autoware では別ノード（crop_box_filter、voxel_grid_downsample_filter）が行います。
3. `SmartPoseBuffer` で EKF の姿勢をスキャン時刻に補間します。時間差が `initial_pose_timeout_sec` を超える場合や、
   前後の姿勢が `initial_pose_distance_tolerance_m` 以上飛んでいる場合（再初期化の直後など）は照合しません。
4. NDT（multigrid ndt_omp、OpenMP で並列化）で照合します。
5. 採否判定: `(反復回数 < 上限 || 局所解で振動) && スコア > 閾値`。スコアは既定で NVTL です。
   初期値からの移動量と処理時間は、Autoware と同じく警告だけです。
6. 収束した場合だけ姿勢を配信します。不採用が `skipping_publish_num` 回続くと WARN になります。

地図は起動時に PCD を間引かずに読み、1 回だけ全体を NDT に登録します（Autoware の動的な地図読み込みは行いません）。
表示用には同じ点群を `debug/loaded_pointcloud_map` に 1 回配信します。
ndt_omp は `include/ndt_localization/ndt_scan_matcher/ndt_omp/` と `src/ndt_scan_matcher/ndt_omp/` に同梱しています
（出典とライセンスは同ディレクトリの README.md と LICENSE を参照）。

## 調整の目安

- NVTL の閾値 `converged_param_nearest_voxel_transformation_likelihood`（Autoware の既定は 2.3）:
  この地図はスキャンとの重なりが 50〜85% と小さいので、bag で `/diagnostics` の NVTL を見て、
  正しい照合まで不採用になっている場合は下げてください。
- `sensor_points.max_range` は v0.1 で PCL NDT の処理時間に合わせて 18 m にした値です。ndt_omp は速いので、
  `execution_time` を見ながら広げる余地があります。
- EKF の外れ値除去は `pose0_rejection_threshold`（マハラノビス距離）、遅延した観測の扱いは
  `smooth_lagged_data` と `history_length` で調整します。
- `ekf_localizer` の `odom0` には odom 用 EKF の出力をそのまま入れているので、雑音が時間的に相関しています。
  生の車輪オドメトリと IMU のトピックがある場合は、それらを `odom0` と `imu0` に直接入れる構成が理想です。

## v0.1 から変わった点

- 自作の状態機械（TRACKING/LOST）とワーカースレッドをやめました。現在は Autoware と同じく、
  callback group と MultiThreadedExecutor で動かし、状態は診断情報で表します。
- `map→odom` は「NDT の結果で丸ごと置き換える」方式から、EKF で融合する方式に変わりました。
- 初期化中の予測位置が初期姿勢に固定される問題はなくなりました。EKF がオドメトリで予測し続けるためです。

## 制約

- 点ごとの移動ひずみ補正、Monte Carlo による初期位置探索（`ndt_align_srv`）、
  動的な地図読み込み、Laplace 近似などによる共分散推定、GNSS による正則化は未実装です。
- 単一の NDT 解とスコアの閾値で判定するので、似た形状への誤収束を完全には検出できません。
- Ctrl-C で終了すると、robot_localization の ekf_node が exit code -2 を報告します（robot_localization 側の挙動です）。
- **未解決の問題**：結合テストを繰り返すと、15 回中 2 回程度の割合で `map→odom` が一時的に 0.6〜0.7 m 進行方向へ跳ね、
  約 0.5 秒で NDT が引き戻します。robot_localization のデバッグログでは、ekf_node の周期処理が約 1.2 秒止まっています
  （その間はコールバックも実行されません）。その後、予測を一度に 1.2 秒分進めています。
  別プロセスの監視ではシステム全体の停止は見つからず、Fast DDS の共有メモリを無効にしても再現しました。
  `smooth_lagged_data: false` でも起きます。原因は特定できていません。

## テスト

```bash
cd /home/takumi/ros2_ws
colcon test --packages-select ndt_localization --event-handlers console_direct+
colcon test-result --verbose
# launch ファイルを使った結合テスト（合成データ、専用の ROS ドメインで実行）
source install/setup.bash
python3 src/ndt_localization/test/test_ndt_localization_launch.py
```

- 単体テスト: 角度差の正規化、姿勢の補間（yaw の折り返し、外挿）、共分散の回転、振動回数、
  SmartPoseBuffer（補間、保持、時刻の逆行、急変）、距離による切り出しと間引き、ndt_omp による既知の変換の復元。
- 結合テスト: 2 ノードを launch で起動し、地図を Transient Local で受け取れること、`/initialpose` による EKF のリセット、
  NDT の姿勢、EKF による融合で `map→odom` が真値に収束して 5 秒間追跡が続くことを確認します。

実際の bag と地図での精度・処理時間はまだ検証していません。
