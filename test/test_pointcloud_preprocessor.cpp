// (本パッケージ独自) Autoware に対応するテストはない（Autoware は前処理を別のフィルタノードで行う）。
#include "pointcloud_preprocessor/pointcloud_preprocessor.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace ndt_localization::pointcloud_preprocessor;

// 距離の範囲外の点と NaN の点が除かれること
TEST(PointcloudPreprocessor, CropByRange)
{
  auto input = std::make_shared<PointCloud>();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (const auto & p : {pcl::PointXYZ(2, 0, 0), pcl::PointXYZ(0.5f, 0, 0), pcl::PointXYZ(30, 0, 0),
      pcl::PointXYZ(nan, 0, 0)})
  {
    input->push_back(p);
  }
  const auto cropped = crop_by_range(input, 1.0, 20.0);
  ASSERT_EQ(cropped->size(), 1u);
  EXPECT_FLOAT_EQ((*cropped)[0].x, 2.0f);
}

// 1 つのボクセルに入る点が 1 点にまとまり、voxel_size 0 なら間引かれないこと
TEST(PointcloudPreprocessor, VoxelGridDownsampleFilter)
{
  auto input = std::make_shared<PointCloud>();
  for (int i = 0; i < 10; ++i) {input->push_back(pcl::PointXYZ(0.01f * i, 0, 0));}
  EXPECT_EQ(voxel_grid_downsample_filter(input, 1.0f)->size(), 1u);
  EXPECT_EQ(voxel_grid_downsample_filter(input, 0.0f)->size(), 10u);
}
