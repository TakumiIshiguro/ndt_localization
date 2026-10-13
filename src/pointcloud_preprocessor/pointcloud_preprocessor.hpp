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
 * @brief ボクセルグリッドで点群を間引く。
 * @details autoware_core sensing/autoware_downsample_filters の
 *          VoxelGridDownsampleFilter::filter に当たる処理。Autoware の
 *          FasterVoxelGridDownsampleFilter ではなく pcl::VoxelGrid を使う。
 *          NaN の点は、input の is_dense が false のときだけ除かれる（pcl::VoxelGrid の仕様）。
 * @param[in] input 入力点群（Autoware と同じくセンサ座標系）。
 * @param[in] voxel_size ボクセルの一辺 [m]。0 以下なら間引かない。
 * @return 各ボクセルの重心を 1 点とした点群（voxel_size <= 0 なら入力のコピー）。
 */
PointCloud::Ptr voxel_grid_downsample_filter(const PointCloud::ConstPtr & input, float voxel_size);
}  // namespace ndt_localization::pointcloud_preprocessor
