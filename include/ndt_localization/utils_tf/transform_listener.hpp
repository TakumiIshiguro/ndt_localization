// 参照元: autoware_utils (commit 6b89a2e1)
//   autoware_utils_tf/include/autoware_utils_tf/transform_listener.hpp
// 動作もメソッド名も同じ。違い: NodeT / BufferT / ListenerT のテンプレートではなく rclcpp::Node と tf2_ros を直接使う、
// このパッケージで使う get_latest_transform だけを移植した。
#pragma once
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>

namespace ndt_localization::utils_tf
{
/// TF を購読して、最新の変換を例外ではなく nullptr で失敗を返す形で取り出す補助クラス。
class TransformListener
{
public:
  /// @param[in] node TF の購読、時計、ロガーに使うノード。
  explicit TransformListener(rclcpp::Node * node) : clock_(node->get_clock()), logger_(node->get_logger())
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  /**
   * @brief 最新の変換を返す。tf2_ros::Buffer::lookupTransform(from, to, TimePointZero) と同じ向き
   *        （from が target、to が source）。
   * @param[in] from 変換先（target）のフレーム。
   * @param[in] to 変換元（source）のフレーム。
   * @return 変換。取れなければ nullptr（5 秒に 1 回警告を出す）。
   */
  geometry_msgs::msg::TransformStamped::ConstSharedPtr get_latest_transform(
    const std::string & from, const std::string & to)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(from, to, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "failed to get transform from %s to %s: %s", from.c_str(), to.c_str(), ex.what());
      return {};
    }

    return std::make_shared<const geometry_msgs::msg::TransformStamped>(tf);
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
}  // namespace ndt_localization::utils_tf
