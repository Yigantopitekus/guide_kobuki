#ifndef LIBRARY_LIB__NAVODOM_HPP_
#define LIBRARY_LIB__NAVODOM_HPP_

#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "library_lib/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace library_lib
{

class NavOdom : public library_lib::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  explicit NavOdom(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList();  // No se usan puertos en este nodo
  }
};

}  // namespace library_lib

#endif  // LIBRARY_LIB__NAVODOM_HPP_
