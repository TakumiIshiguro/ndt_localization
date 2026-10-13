# ndt_localization

ROS 2 Humble 用の自己位置推定パッケージです。Autoware Core の `localization` を移植した構成で、
固定の 3D 点群地図に対する LiDAR の NDT 照合と、車輪・IMU の速度を EKF で融合します。
Nav2 用のオドメトリ（`odom→base_link` と `/odometry/filtered`）も本パッケージが出すので、robot_localization は使いません。

## 構成

```
 /odom（車輪）─▶ vehicle_velocity_converter ─┐
 /imu/data ─▶ imu_corrector ─────────────────▶ gyro_odometer ─▶ /localization/twist_estimator/twist_with_covariance
                                                                     │                          │
                                         odometry_integrator ◀───────┘                          ▼
                                           ├ TF odom→base_link        /surestar_points ─▶ ndt_scan_matcher ─▶ ekf_localizer
                                           └ /odometry/filtered（Nav2）          ▲ biased_pose_with_covariance ◀─┤
                                                                                                    ├ TF map→odom
                                                                                                    └ /localization/kinematic_state
 /initialpose（RViz）、起動時の姿勢 ─▶ pose_initializer ─▶ /initialpose3d、EKF の trigger_node
```

ノード名、名前空間、トピック名は `autoware_core_localization.launch.xml` に合わせています。

| ノード（名前空間） | Autoware Core の対応 | 役割 |
| --- | --- | --- |
| `vehicle_velocity_converter`（`/sensing`） | `autoware_vehicle_velocity_converter` | 車輪オドメトリの速度に固定の共分散を付ける（入力は VelocityReport ではなく Odometry） |
| `imu_corrector`（`/sensing/imu`） | `autoware_imu_corrector`（Universe。Core にはない） | IMU の角速度の共分散をパラメータの値で上書きし、base_link に回す |
| `gyro_odometer`（`/localization/twist_estimator`） | `autoware_gyro_odometer` | 車輪の vx と IMU の角速度を 1 つの twist にまとめる |
| `ndt_scan_matcher`（`/localization/pose_estimator`） | `autoware_ndt_scan_matcher` | multigrid ndt_omp による照合 |
| `ekf_localizer`（`/localization/pose_twist_fusion_filter`） | `autoware_ekf_localizer` | twist と NDT の姿勢を融合する。TF は `map→base_link` ではなく `map→odom` を配信 |
| `pose_initializer`（`/localization/util`） | `autoware_pose_initializer`（簡易版） | 起動時の姿勢と RViz の姿勢で EKF をリセットし、`trigger_node` で動かす |
| `odometry_integrator`（`/localization/util`） | なし（本パッケージ独自） | twist を 2D で積分し、`odom→base_link` と `/odometry/filtered` を配信 |

Core との主な違い:

- **TF**: Core の ekf_localizer は odom フレームを持たず `map→base_link` を出しますが、orne-box の Nav2 が odom フレーム
  （local_costmap と behavior_server の `global_frame: odom`）を使うので、`map→odom = map→base_link × (odom→base_link)⁻¹`
  を出します（`misc.odom_frame_id`。空にすると Core と同じ `map→base_link`）。
- **オドメトリ**: Core には odom フレームがないので、`odometry_integrator` を追加しました。odom と EKF は同じ twist を使います。
- **IMU**: Core の launch は IMU を使わず、車輪の速度をそのまま EKF に渡します（twist_estimator は relay の仮置き）。
  本パッケージは gyro_odometer で IMU の角速度を使うので、IMU のドライバ（adi_driver2）が出す共分散 0 を上書きするために、
  Universe の `imu_corrector` を `/sensing/imu` に置いています（Universe の sensing と同じ場所）。
- **なし**: stop_filter、twist2accel、map_loader ノード、voxel_grid_downsample_filter ノード（NDT の中で前処理）、
  NDT の `trigger_node` と `ndt_align_srv`、pose_initializer の AD API・GNSS・map_height_fitter。

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
| `include/.../localization_util/{smart_pose_buffer,util_func,covariance_ellipse}.hpp`、`src/localization_util/` | `localization/autoware_localization_util/` |
| `include/.../kalman_filter/`、`src/kalman_filter/` | `common/autoware_kalman_filter/`（同梱。名前空間と include パス以外は変更なし） |
| `src/ekf_localizer/`（`utils/` を含む） | `localization/autoware_ekf_localizer/src/`（同じファイル構成。agnocast を rclcpp に置き換え、TF を `map→odom` に） |
| `src/gyro_odometer/` | `localization/autoware_gyro_odometer/src/`（同じファイル構成。agnocast を rclcpp に置き換え） |
| `src/vehicle_velocity_converter/` | `sensing/autoware_vehicle_velocity_converter/src/`（入力を Odometry に変更） |
| `src/imu_corrector/` | autoware_universe（commit `2ee1a85`）`sensing/autoware_imu_corrector/{include/autoware/imu_corrector,src}/imu_corrector_core.*`（imu_corrector だけ。動的バイアスとスケールの補正は除く） |
| `src/pose_initializer/` | `localization/autoware_pose_initializer/src/`（起動時の姿勢と EKF の起動の手順だけ） |
| `src/odometry_integrator/` | なし（本パッケージ独自） |
| `include/.../utils_diagnostics/diagnostics_interface.hpp` | autoware_utils `autoware_utils_diagnostics/diagnostics_interface.hpp` |
| `include/.../utils_{geometry,system,tf}/` | autoware_utils の `autoware_utils_{geometry,system,tf}`（使う関数だけ） |
| `src/map_loader/pointcloud_map_loader.*` | `map/autoware_map_loader/src/pointcloud_map_loader/pointcloud_map_loader.*`（`load_pointcloud_map`） |
| `src/pointcloud_preprocessor/pointcloud_preprocessor.*` | `sensing/autoware_downsample_filters`（voxel grid）の処理をノード内で行う |
| `config/ndt_scan_matcher.param.yaml` | `localization/autoware_ndt_scan_matcher/config/ndt_scan_matcher.param.yaml` |
| `config/{ekf_localizer,gyro_odometer,vehicle_velocity_converter,pose_initializer}.param.yaml` | 各パッケージの `config/`（同じ名前と値。追加したものには `(this package only)`） |
| `config/imu_corrector.param.yaml` | autoware_universe `sensing/autoware_imu_corrector/config/imu_corrector.param.yaml`（同じ名前と値） |
| `launch/ndt_localization.launch.py` | `localization/autoware_core_localization/launch/autoware_core_localization.launch.xml` |
| `test/test_{util_func,smart_pose_buffer,covariance_ellipse,ndt_scan_matcher_helper,kalman_filter,time_delay_kalman_filter}.cpp`、`test/ekf_localizer/`、`test/gyro_odometer/`、`test/vehicle_velocity_converter/` | 各パッケージの同名テスト（テスト名も合わせています） |

## ビルド

```bash
source /opt/ros/humble/setup.bash
cd /home/takumi/ros2_ws
colcon build --symlink-install --packages-select ndt_localization --parallel-workers 1
source install/setup.bash
```

依存: rclcpp、tf2_ros、std_srvs、PCL、pcl_conversions、OpenMP、Boost、Eigen、fmt。

## bag での起動

```bash
# ターミナル1
ros2 launch ndt_localization ndt_localization.launch.py use_sim_time:=true
# ターミナル2
ros2 bag play /home/takumi/bags/tsudanuma_3d_data_2 --clock
```

起動時の姿勢は `config/pose_initializer.param.yaml` の `user_defined_initial_pose.pose`（x, y, z, qx, qy, qz, qw）か、
launch 引数 `initial_pose:="[x, y, z, qx, qy, qz, qw]"` で指定します（Core の `initial_pose` 引数と同じ）。
RViz の `2D Pose Estimate` でいつでも初期化し直せます。どちらも pose_initializer が、EKF を止める →
`/initialpose3d` で EKF をリセットする → EKF を動かす、の順に処理します（Core の `set_user_defined_initial_pose`）。
EKF は初期化されるまで何も出力しません（Core と同じ）。
RViz の姿勢は z/roll/pitch が 0 です。Autoware は map_height_fitter で z を地図から補いますが、
本パッケージにはないので、地図の地面の高さが 0 から離れている場合は NDT が z を合わせるまで少し時間がかかります。

launch 引数:

| 引数 | 既定値 | 内容 |
| --- | --- | --- |
| `lidar_input_topic` | `/surestar_points` | LiDAR 点群 |
| `vehicle_odometry_input_topic` | `/odom` | 車輪オドメトリ（Odometry。twist の vx を使う） |
| `imu_input_topic` | `/imu/data` | IMU（imu_corrector の入力。角速度を使う） |
| `initial_pose` | （YAML の `user_defined_initial_pose`） | 起動時の姿勢 `[x, y, z, qx, qy, qz, qw]` |
| `map_path` | （YAML の `map.path`） | PCD 地図 |
| `*_param_path` | `config/` 内の各ファイル | vehicle_velocity_converter、imu_corrector、gyro_odometer、odometry_integrator、ndt_scan_matcher、ekf_localizer、pose_initializer |
| `use_sim_time` / `rviz` / `rviz_config` | | |

## 入出力

| トピック | 型 | 内容 |
| --- | --- | --- |
| `/surestar_points` | PointCloud2 | 入力。x/y/z は FLOAT32（`pcl::fromROSMsg` で読む） |
| `/odom` | Odometry | 入力。車輪オドメトリ。twist（base_link）だけを使う |
| `/imu/data` | Imu | 入力。imu_corrector が受け取る |
| `/sensing/imu/imu_data` | Imu | imu_corrector の出力（base_link。角速度の共分散はパラメータの値） |
| `/initialpose` | PoseWithCovarianceStamped | 入力。map フレーム。pose_initializer が受け取る |
| `/sensing/vehicle_velocity_converter/twist_with_covariance` | TwistWithCovarianceStamped | 共分散を付けた車輪の速度 |
| `/localization/twist_estimator/twist_with_covariance` | TwistWithCovarianceStamped | gyro_odometer の出力（車輪の vx と IMU の角速度） |
| `/localization/pose_estimator/pose_with_covariance` | PoseWithCovarianceStamped | 収束した NDT の姿勢。共分散は固定値を map フレームへ回転したもの |
| `/localization/kinematic_state` | Odometry | EKF の出力（map→base_link） |
| `/localization/pose_with_covariance`、`/localization/pose_twist_fusion_filter/biased_pose_with_covariance` | PoseWithCovarianceStamped | EKF の姿勢（biased は yaw のずれを補正する前で、NDT の初期値に使う） |
| `/odometry/filtered` | Odometry | odometry_integrator の出力（odom→base_link）。Nav2 が使う |
| `/localization/pose_estimator/debug/loaded_pointcloud_map` | PointCloud2 | 表示用の地図（Transient Local） |
| `/localization/pose_estimator/points_aligned` | PointCloud2 | 照合後の点群（購読者がいる場合だけ配信） |
| `/diagnostics` | DiagnosticArray | `ndt_scan_matcher: scan_matching_status` など |
| TF `map→odom` | | ekf_localizer が配信（`odom→base_link` を読んで計算する） |
| TF `odom→base_link` | | odometry_integrator が配信 |

外部（ロボットまたは bag）から `base_link→LiDAR` と `base_link→IMU` の TF が必要です。
`odom→base_link` を出すノード（robot_localization の odom 用 EKF や、icart のドライバの `publish_odom_tf`）を
同時に動かすと TF が衝突するので、止めてください。

## NDT の処理（`ndt_scan_matcher_core.cpp`）

Autoware の `callback_sensor_points_main` と同じ順に処理します。

1. 点群の検査（空でないこと、遅延は警告のみ）
2. 前処理: `pcl::fromROSMsg` → VoxelGrid（センサ座標系、`voxel_size` 1.5 m）→ base_link への変換 → 最大距離（`required_distance`）の検査。
   VoxelGrid は、Autoware Core では NDT の手前の別ノード（voxel_grid_downsample_filter）がセンサ座標系のまま行います。
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
- `misc.odom_tf_timeout_sec`（本パッケージ独自、0.2 s）: EKF の時刻より古い `odom→base_link` で `map→odom` を
  計算すると、その間に進んだ分だけ `map→odom` がずれます（プロセスが一時停止した直後など）。この時間より古い間は
  `map→odom` を配信せず、直前の値を使い続けます。
- `config/ekf_localizer.param.yaml` は Core の既定値のままです。外れ値除去は `pose_gate_dist` と `twist_gate_dist`
  （マハラノビス距離の 2 乗）、遅延の補償の範囲は `extend_state_step`（× 1 / `predict_frequency` 秒）で調整します。
- `max_twist_queue_size: 2`（Core の既定値）は、速度が `predict_frequency`（50 Hz）より速く届くと超えて、
  古い速度が捨てられ「Twist queue size ... is exceeding max_queue_size」の警告が出ます。gyro_odometer の出力は
  車輪と IMU の遅い方の周期になるので、実機の周期を見て、必要なら大きくしてください。
- 車輪の速度の共分散は `vehicle_velocity_converter.param.yaml` の標準偏差、IMU の角速度の共分散は
  `imu_corrector.param.yaml` の `angular_velocity_stddev_*`（Universe の既定値 0.03 rad/s。3 軸の最大値が使われる）から
  決まります。IMU が出す共分散は使いません。共分散が 0 だと EKF が角速度を完全に信じて、速度の観測を
  マハラノビス距離で棄却し続けます。この値はセンサーの雑音（ADIS16465 を止めた状態で 1〜4 mrad/s）より大きく、
  振動やモデルの誤差も含めた値として扱います。

## v0.1 から変わった点

- 自作の状態機械（TRACKING/LOST）とワーカースレッドをやめました。現在は Autoware と同じく、
  callback group と MultiThreadedExecutor で動かし、状態は診断情報で表します。
- `map→odom` は「NDT の結果で丸ごと置き換える」方式から、EKF で融合する方式に変わりました。
- 初期化中の予測位置が初期姿勢に固定される問題はなくなりました。EKF がオドメトリで予測し続けるためです。

## 制約

- 点ごとの移動ひずみ補正、Monte Carlo による初期位置探索（`ndt_align_srv`）、
  動的な地図読み込み、Laplace 近似などによる共分散推定、GNSS による正則化は未実装です。
- 単一の NDT 解とスコアの閾値で判定するので、似た形状への誤収束を完全には検出できません。
- odometry_integrator は、速度が途切れた後に届いた速度で、途切れていた区間も積分します（`max_dt` 2.0 s まで。
  robot_localization の odom 用 EKF が途切れの間も速度で予測を続けるのと同じ扱い）。それより長い途切れは積分しません。
- この環境（WSL2）では、マシン全体が 1.2〜1.4 秒ほど止まることがあります（EKF の「EKF period may be too slow」の警告で分かります）。
  以前の構成で結合テストの `map→odom` が 0.6〜0.7 m 跳ねていたのはこれが原因で、再開直後の 1 周期だけ古い
  `odom→base_link` で `map→odom` を計算していました。現在は `misc.odom_tf_timeout_sec` と `max_dt` で対処しています。
- odometry_integrator は 2D で積分するので、`/odometry/filtered` と `odom→base_link` の z、roll、pitch は 0 で、
  姿勢の共分散は 0 です（Nav2 は使いません）。
- pose_initializer は簡易版なので、NDT による初期位置探索、GNSS、地図からの z の補正、停止の確認はありません。

## テスト

```bash
cd /home/takumi/ros2_ws
colcon test --packages-select ndt_localization --event-handlers console_direct+
colcon test-result --verbose
# launch ファイルを使った結合テスト（合成データ、専用の ROS ドメインで実行）
source install/setup.bash
python3 src/ndt_localization/test/test_ndt_localization_launch.py
```

- 単体テスト: Core から移植したテスト（localization_util、kalman_filter、ekf_localizer、gyro_odometer、
  vehicle_velocity_converter）と、本パッケージのテスト（間引き、ndt_omp による既知の変換の復元、
  odometry_integrator の積分、起動時の姿勢の検証）。
- 結合テスト: launch で全ノードを起動し、車輪の `/odom` と IMU と点群を流して、地図を Transient Local で受け取れること、
  `/initialpose` まで EKF が動かないこと、`/initialpose` による初期化、NDT の姿勢、`map→odom` と `odom→base_link` を
  つないだ姿勢と `/localization/kinematic_state` が真値に収束して 5 秒間追跡が続くことを確認します。

実際の bag と地図での精度・処理時間はまだ検証していません。
