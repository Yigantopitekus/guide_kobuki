#include "library_lib/NavOdom.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
namespace library_lib
{

NavOdom::NavOdom(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: library_lib::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void
NavOdom::on_tick()
{
  RCLCPP_INFO(node_->get_logger(), "**ENTRO A ODOM**");

  geometry_msgs::msg::PoseStamped target_pose;


  target_pose.header.frame_id = "map";
  target_pose.header.stamp = node_->now();

  target_pose.pose.position.x = 0;
  target_pose.pose.position.y = 0;
  target_pose.pose.orientation.w = 0;

  goal_.pose = target_pose;

}

BT::NodeStatus
NavOdom::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "** NAVIGATION SUCCEEDED **");
  return BT::NodeStatus::SUCCESS;
}

}  


BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<library_lib::NavOdom>(
        name, "navigate_to_odom", config);
    };

  factory.registerBuilder<library_lib::NavOdom>("NavOdom", builder);
}
