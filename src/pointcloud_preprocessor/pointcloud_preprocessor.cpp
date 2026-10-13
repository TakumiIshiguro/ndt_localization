// 各関数の入力・出力・処理と Autoware Core の対応は pointcloud_preprocessor.hpp を参照。
#include "pointcloud_preprocessor.hpp"
#include <pcl/filters/voxel_grid.h>

namespace ndt_localization::pointcloud_preprocessor
{
PointCloud::Ptr voxel_grid_downsample_filter(const PointCloud::ConstPtr & input, float voxel_size)
{
  auto output = std::make_shared<PointCloud>();
  if (voxel_size <= 0) {*output = *input; return output;}
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  filter.setInputCloud(input);
  filter.filter(*output);
  return output;
}
}  // namespace ndt_localization::pointcloud_preprocessor
