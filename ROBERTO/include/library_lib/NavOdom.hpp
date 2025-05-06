#ifndef LIBRARY_LIB__NAVODOM_HPP_
#define LIBRARY_LIB__NAVODOM_HPP_

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "library_lib/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace library_lib
{

using namespace std::chrono_literals;

class NavOdom : public library_lib::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  explicit NavOdom(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus on_success() override;
  void on_tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("odom")});
  }

};

}

#endif
