// (本パッケージ独自) Autoware Core では、点群の前処理を NDT の手前の別ノードで行う
// （autoware_core_localization.launch.xml: voxel_grid_downsample_filter → ndt_scan_matcher）。
// 本パッケージでは同じ処理を ndt_scan_matcher の中で行う。前処理ノードを別に立てれば不要になる。
#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ndt_localization::pointcloud_preprocessor
{
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

/**
 * @brief センサ原点からの距離で点を絞り込む。
 * @details autoware_core sensing/autoware_crop_box_filter（CropBoxFilter::filter）に当たる処理だが、
 *          箱ではなく球殻で切り出す。NaN の点は比較が偽になるので自然に除かれる。
 * @param[in] input 入力点群（センサ座標系）。
 * @param[in] min_range 残す距離の下限 [m]。
 * @param[in] max_range 残す距離の上限 [m]。
 * @return min_range <= |p| <= max_range の点だけを残した点群。
 */
PointCloud::Ptr crop_by_range(const PointCloud::ConstPtr & input, double min_range, double max_range);

/**
 * @brief ボクセルグリッドで点群を間引く。
 * @details autoware_core sensing/autoware_downsample_filters の
 *          VoxelGridDownsampleFilter::filter に当たる処理。Autoware の
 *          FasterVoxelGridDownsampleFilter ではなく pcl::VoxelGrid を使う。
 * @param[in] input 入力点群。
 * @param[in] voxel_size ボクセルの一辺 [m]。0 以下なら間引かない。
 * @return 各ボクセルの重心を 1 点とした点群（voxel_size <= 0 なら入力のコピー）。
 */
PointCloud::Ptr voxel_grid_downsample_filter(const PointCloud::ConstPtr & input, float voxel_size);
}  // namespace ndt_localization::pointcloud_preprocessor
