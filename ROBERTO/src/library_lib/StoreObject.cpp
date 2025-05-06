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

StoreObject::StoreObject(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf)
{


    rclcpp::Node::SharedPtr node;
    node_ = node;
    config().blackboard->get("node", node);

}
void library_lib::StoreObject::halt()
{
  // Aquí puedes limpiar o resetear estados si hace falta
}

BT::NodeStatus
StoreObject::tick()
{
    return BT::NodeStatus::SUCCESS;
}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<library_lib::StoreObject>("StoreObject");
}

