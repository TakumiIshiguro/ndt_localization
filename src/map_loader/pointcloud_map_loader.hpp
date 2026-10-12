// 参照元: autoware_core (commit e0a8ab84)
//   map/autoware_map_loader/src/pointcloud_map_loader/pointcloud_map_loader.hpp の load_pointcloud_map
// (本パッケージ独自の使い方) Autoware では map_loader ノードが地図を読み、map_update_module 経由で
// ndt_scan_matcher に少しずつ渡す（差分読み込み）。本パッケージでは ndt_scan_matcher が PCD を 1 つ自分で読む。
#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace ndt_localization::map_loader
{
/**
 * @brief PCD ファイルを読み込む。
 * @details Autoware との違い: PCD は 1 ファイルだけ、間引きはしない、戻り値は PointCloud2 ではなく
 *          PCL の点群、読めないときは on_error コールバックではなく例外を投げる。
 * @param[in] pcd_path PCD ファイルのパス。
 * @return 地図の点群（map 座標系）。
 * @throws std::runtime_error パスが空、またはファイルを読めない場合。
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr load_pointcloud_map(const std::string & pcd_path);
}  // namespace ndt_localization::map_loader
