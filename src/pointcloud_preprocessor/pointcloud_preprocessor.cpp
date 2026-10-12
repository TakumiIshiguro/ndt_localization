// 各関数の入力・出力・処理と Autoware Core の対応は pointcloud_preprocessor.hpp を参照。
#include "pointcloud_preprocessor.hpp"
#include <pcl/filters/voxel_grid.h>

namespace ndt_localization::pointcloud_preprocessor
{
PointCloud::Ptr crop_by_range(const PointCloud::ConstPtr & input, double min_range, double max_range)
{
  auto output = std::make_shared<PointCloud>();
  output->reserve(input->size());
  const double min_squared = min_range * min_range, max_squared = max_range * max_range;
  for (const auto & point : *input) {
    const double range_squared = point.getVector3fMap().cast<double>().squaredNorm();
    if (range_squared >= min_squared && range_squared <= max_squared) {output->push_back(point);}
  }
  return output;
}

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
