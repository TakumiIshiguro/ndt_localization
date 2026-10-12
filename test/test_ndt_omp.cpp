// (本パッケージ独自) 同梱した ndt_omp を合成データで確認する。Autoware で近いテストは
// localization/autoware_ndt_scan_matcher/test/test_ndt_scan_matcher_characteristics.cpp。
#include "ndt_localization/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <gtest/gtest.h>
#include <random>
using PointSource = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointSource>;
namespace
{
/**
 * @brief テスト用の合成地図を作る。±8 m の範囲に 140 個の塊を置き、各塊に σ=0.18 m で 80 点を散らす。
 * @return 地図の点群（乱数の種は固定なので毎回同じ）。
 */
Cloud::Ptr clustered_map()
{
  auto map = std::make_shared<Cloud>();
  std::mt19937 generator(42);
  std::uniform_real_distribution<float> center(-8, 8);
  std::normal_distribution<float> noise(0, 0.18);
  for (int cluster = 0; cluster < 140; ++cluster) {
    const Eigen::Vector3f c(center(generator), center(generator), center(generator) * 0.3f);
    for (int point = 0; point < 80; ++point) {
      map->push_back(PointSource(c.x() + noise(generator), c.y() + noise(generator), c.z() + noise(generator)));
    }
  }
  return map;
}
/// テスト用の NDT パラメータを返す（解像度 1 m、反復上限 30、2 スレッド）。
pclomp::NdtParams params()
{
  pclomp::NdtParams p{};
  p.trans_epsilon = 0.01; p.step_size = 0.1; p.resolution = 1.0f;
  p.max_iterations = 30; p.num_threads = 2;
  return p;
}
}  // namespace

// 既知の剛体変換（並進約 0.24 m、yaw 2°）をかけた点群から、その変換を復元できること
TEST(NdtOmp, RecoversKnownRigidTransform)
{
  const auto map = clustered_map();
  Eigen::Isometry3f truth = Eigen::Isometry3f::Identity();
  truth.translation() << 0.2f, -0.12f, 0.05f;
  truth.linear() = Eigen::AngleAxisf(0.035f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  auto source = std::make_shared<Cloud>();
  pcl::transformPointCloud(*map, *source, truth.inverse().matrix());

  pclomp::MultiGridNormalDistributionsTransform<PointSource, PointSource> ndt;
  ndt.setParams(params());
  ndt.setInputTarget(map);
  ASSERT_TRUE(ndt.hasTarget());
  Cloud output;
  ndt.align(output, Eigen::Matrix4f::Identity(), source);
  const auto result = ndt.getResult();
  EXPECT_LT(result.iteration_num, ndt.getMaximumIterations());
  EXPECT_LT((result.pose.block<3, 1>(0, 3) - truth.translation()).norm(), 0.06f);
  const Eigen::Quaternionf q(result.pose.block<3, 3>(0, 0));
  EXPECT_LT(q.angularDistance(Eigen::Quaternionf(truth.linear())), 0.02f);
  // 照合後のスコアは初期値のスコアより高くなるはず（絶対値はデータ次第なので閾値とは比べない）
  ASSERT_FALSE(result.nearest_voxel_transformation_likelihood_array.empty());
  EXPECT_GT(result.nearest_voxel_transformation_likelihood,
    result.nearest_voxel_transformation_likelihood_array.front());
}

// 地図と 3 m ずれた点群は、ずれていない点群より NVTL が低くなること
TEST(NdtOmp, MisalignedScanScoresLower)
{
  const auto map = clustered_map();
  pclomp::MultiGridNormalDistributionsTransform<PointSource, PointSource> ndt;
  ndt.setParams(params());
  ndt.setInputTarget(map);
  Cloud shifted;
  Eigen::Matrix4f offset = Eigen::Matrix4f::Identity();
  offset(0, 3) = 3.0f;
  pcl::transformPointCloud(*map, shifted, offset);
  EXPECT_LT(ndt.calculateNearestVoxelTransformationLikelihood(shifted),
    ndt.calculateNearestVoxelTransformationLikelihood(*map));
}
