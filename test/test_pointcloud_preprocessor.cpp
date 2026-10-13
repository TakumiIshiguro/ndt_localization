// (本パッケージ独自) Autoware に対応するテストはない（Autoware は前処理を別のフィルタノードで行う）。
#include "pointcloud_preprocessor/pointcloud_preprocessor.hpp"
#include <gtest/gtest.h>
using namespace ndt_localization::pointcloud_preprocessor;

// 1 つのボクセルに入る点が 1 点にまとまり、voxel_size 0 なら間引かれないこと
TEST(PointcloudPreprocessor, VoxelGridDownsampleFilter)
{
  auto input = std::make_shared<PointCloud>();
  for (int i = 0; i < 10; ++i) {input->push_back(pcl::PointXYZ(0.01f * i, 0, 0));}
  EXPECT_EQ(voxel_grid_downsample_filter(input, 1.0f)->size(), 1u);
  EXPECT_EQ(voxel_grid_downsample_filter(input, 0.0f)->size(), 10u);
}
