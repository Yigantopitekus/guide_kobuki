#ifndef B1_HPP_
#define B1_HPP_

#include "kobuki_ros_interfaces/msg/button_event.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


namespace prueba
{


class B1Pressed : public BT::ConditionNode
{
public:
  explicit B1Pressed(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
    return { 
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint"),
    BT::InputPort<size_t>("i"),

    BT::InputPort<std::vector<std::string>>("nombres"),
    BT::OutputPort<size_t>("i")
    };
    }


private:
  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg);

  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;
    rclcpp::Node::SharedPtr node_;

  kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr last_button_;
  size_t i;

};

}  // namespace practica

#endif  // BUTTON_HPP_