#ifndef LIBRARY_LIB__STOREOBJECT_HPP_
#define LIBRARY_LIB__STOREOBJECT_HPP_

#include <string>
#include <iostream>
#include <vector>

#include "library_lib/StoreObject.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kobuki_ros_interfaces/msg/button_event.hpp"
#include "rclcpp/rclcpp.hpp"
namespace library_lib
{

class StoreObject : public BT::ActionNodeBase
{
public:
  explicit StoreObject(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();
  static BT::PortsList providedPorts()
  {
    return BT::PortsList();
  }
private:

  rclcpp::Node::SharedPtr node_;

};

}

#endif
