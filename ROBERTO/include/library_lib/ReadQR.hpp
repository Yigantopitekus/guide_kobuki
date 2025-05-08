#ifndef LIBRARY_LIB__READQR_HPP_
#define LIBRARY_LIB__READQR_HPP_

#include <string>
#include <iostream>
#include <vector>


#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
namespace library_lib
{

class ReadQR : public BT::ActionNodeBase
{
public:
  explicit ReadQR(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
      });
  }

private:
  void object_callback(std_msgs::msg::Float32MultiArray::UniquePtr msg);

  geometry_msgs::msg::PoseStamped wp_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr object_sub_;

  double cienciax_;
  double cienciay_;
  double cienciaw_;

  double literaturax_;
  double literaturay_;
  double literaturaw_;

  double infantilx_;
  double infantily_;
  double infantilw_;

  double historiax_;
  double historiay_;
  double historiaw_;

  int id_;

  rclcpp::Node::SharedPtr node_;

  std_msgs::msg::Float32MultiArray::UniquePtr last_object_;
};

}
#endif

