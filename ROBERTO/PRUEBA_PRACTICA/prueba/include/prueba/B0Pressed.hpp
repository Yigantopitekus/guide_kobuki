#ifndef B0_HPP_
#define B0_HPP_

#include "kobuki_ros_interfaces/msg/button_event.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


namespace prueba
{


class B0Pressed : public BT::ConditionNode
{
public:
  explicit B0Pressed(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

    static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("waypoint") };
  }

private:
  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg);
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;


  kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr last_button_;

};

}  // namespace practica

#endif  // BUTTON_HPP_