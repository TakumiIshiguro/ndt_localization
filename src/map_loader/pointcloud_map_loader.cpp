// 参照元: autoware_core (commit e0a8ab84)
//   map/autoware_map_loader/src/pointcloud_map_loader/pointcloud_map_loader.cpp
// 入力・出力・処理と Autoware との違いはヘッダーを参照。
#include "pointcloud_map_loader.hpp"
#include <pcl/io/pcd_io.h>
#include <stdexcept>

namespace ndt_localization::map_loader
{
pcl::PointCloud<pcl::PointXYZ>::Ptr load_pointcloud_map(const std::string & pcd_path)
{
  auto map = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcd_path.empty() || pcl::io::loadPCDFile(pcd_path, *map) < 0) {
    throw std::runtime_error("Cannot read PCD map: " + pcd_path);
  }
  return map;
}
}  // namespace ndt_localization::map_loader
