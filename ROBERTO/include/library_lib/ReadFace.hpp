#ifndef LIBRARY_LIB__READFACE_HPP_
#define LIBRARY_LIB__READFACE_HPP_

#include <string>
#include <iostream>
#include <vector>


#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
namespace library_lib
{

class ReadFace : public BT::ActionNodeBase
{
public:
  explicit ReadFace(
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
  void object_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr object_sub_;

  int id_;
  rclcpp::Node::SharedPtr node_;

  std_msgs::msg::Float32MultiArray last_object_;
};

}
#endif

