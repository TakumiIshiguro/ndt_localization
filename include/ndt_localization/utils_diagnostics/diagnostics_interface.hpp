// 参照元: autoware_utils (commit 6b89a2e1)
//   autoware_utils_diagnostics/include/autoware_utils_diagnostics/diagnostics_interface.hpp
//   class BasicDiagnosticsInterface / DiagnosticsInterface
// 動作もメソッド名も同じ。違い: NodeT テンプレートではなく rclcpp::Node を直接使う、
// 使っていなかった clock メンバーを省いた。
#pragma once
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <string>

namespace ndt_localization::utils_diagnostics
{
/**
 * @brief 1 つのモジュールの診断（DiagnosticStatus）を、コールバックの中で組み立てて
 *        最後に /diagnostics へ配信するための補助クラス。
 *        使い方: clear() → add_key_value() / update_level_and_message() → publish()。
 */
class DiagnosticsInterface
{
public:
  /**
   * @brief /diagnostics への配信を作り、診断の名前を決める。
   * @param[in] node 配信を作るノード。
   * @param[in] diagnostic_name 診断名。DiagnosticStatus.name は "<ノード名>: <diagnostic_name>" になる。
   */
  DiagnosticsInterface(rclcpp::Node * node, const std::string & diagnostic_name)
  {
    diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
    diagnostics_status_msg_.name = std::string(node->get_name()) + std::string(": ") + diagnostic_name;
    diagnostics_status_msg_.hardware_id = node->get_name();
  }

  /// @brief 診断を初期状態（レベル OK、メッセージなし、キーと値なし）に戻す。
  void clear()
  {
    diagnostics_status_msg_.values.clear();
    diagnostics_status_msg_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics_status_msg_.message = "";
  }

  /**
   * @brief キーと値を追加する。同じキーがすでにあれば値を上書きする。
   * @param[in] key_value_msg 追加するキーと値。
   */
  void add_key_value(const diagnostic_msgs::msg::KeyValue & key_value_msg)
  {
    auto it = std::find_if(std::begin(diagnostics_status_msg_.values), std::end(diagnostics_status_msg_.values),
      [key_value_msg](const auto & arg) {return arg.key == key_value_msg.key;});
    if (it != std::cend(diagnostics_status_msg_.values)) {
      it->value = key_value_msg.value;
    } else {
      diagnostics_status_msg_.values.push_back(key_value_msg);
    }
  }

  /**
   * @brief 数値の値を文字列にして追加する（std::to_string を使う）。
   * @param[in] key キー。
   * @param[in] value 値（int、double など）。文字列リテラルは std::string にしてから渡すこと。
   */
  template<typename T>
  void add_key_value(const std::string & key, const T & value)
  {
    diagnostic_msgs::msg::KeyValue key_value;
    key_value.key = key;
    key_value.value = std::to_string(value);
    add_key_value(key_value);
  }

  /**
   * @brief 文字列の値を追加する。
   * @param[in] key キー。
   * @param[in] value 値。
   */
  void add_key_value(const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue key_value;
    key_value.key = key;
    key_value.value = value;
    add_key_value(key_value);
  }

  /**
   * @brief 真偽値を "True" / "False" として追加する。
   * @param[in] key キー。
   * @param[in] value 値。
   */
  void add_key_value(const std::string & key, bool value)
  {
    diagnostic_msgs::msg::KeyValue key_value;
    key_value.key = key;
    key_value.value = value ? "True" : "False";
    add_key_value(key_value);
  }

  /**
   * @brief 診断レベルを引き上げ、メッセージを追記する。
   * @param[in] level 新しいレベル（OK / WARN / ERROR）。現在より高いときだけレベルを上げる。
   * @param[in] message レベルが OK より上のときに、"; " 区切りでメッセージに追記する。
   */
  void update_level_and_message(const int8_t level, const std::string & message)
  {
    if (level > diagnostic_msgs::msg::DiagnosticStatus::OK) {
      if (!diagnostics_status_msg_.message.empty()) {diagnostics_status_msg_.message += "; ";}
      diagnostics_status_msg_.message += message;
    }
    if (level > diagnostics_status_msg_.level) {diagnostics_status_msg_.level = level;}
  }

  /**
   * @brief 組み立てた診断を /diagnostics に配信する。レベルが OK ならメッセージは "OK" にする。
   * @param[in] publish_time_stamp 配信する DiagnosticArray の stamp。
   */
  void publish(const rclcpp::Time & publish_time_stamp)
  {
    diagnostic_msgs::msg::DiagnosticArray diagnostics_msg;
    diagnostics_msg.header.stamp = publish_time_stamp;
    diagnostics_msg.status.push_back(diagnostics_status_msg_);
    if (diagnostics_msg.status.at(0).level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      diagnostics_msg.status.at(0).message = "OK";
    }
    diagnostics_pub_->publish(diagnostics_msg);
  }

private:
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  diagnostic_msgs::msg::DiagnosticStatus diagnostics_status_msg_;
};
}  // namespace ndt_localization::utils_diagnostics
