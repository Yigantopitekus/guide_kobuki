#ifndef LIST_HPP_
#define LIST_HPP_

#include "kobuki_ros_interfaces/msg/button_event.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


namespace prueba
{


class ShowList : public BT::ActionNodeBase
{
public:
  explicit ShowList(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();
  void halt();


    static BT::PortsList providedPorts()
  {
    return { 
        
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints"),
    BT::OutputPort<std::vector<std::string>>("nombres"),
    BT::OutputPort<size_t>("i")
  };
  }

private:

void printWaypoint(const geometry_msgs::msg::PoseStamped & wp, std::string str);
void printLista(const std::vector<geometry_msgs::msg::PoseStamped>& lista);

geometry_msgs::msg::PoseStamped create_pose(double x, double y);

std::vector<geometry_msgs::msg::PoseStamped> poses;
std::vector<std::string> nombres;
rclcpp::Node::SharedPtr node_;

};

}  // namespace practica

#endif  // BUTTON_HPP_