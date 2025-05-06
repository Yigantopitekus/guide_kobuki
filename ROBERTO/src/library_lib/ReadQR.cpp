#include <string>
#include <iostream>
#include <vector>

#include "library_lib/Search.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kobuki_ros_interfaces/msg/button_event.hpp"
#include "rclcpp/rclcpp.hpp"

namespace library_lib
{

Search::Search(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf)
{


    config().blackboard->get("node", node_);

    node_->declare_parameter("ciencia.x", cienciax_);
    node_->declare_parameter("ciencia.y", cienciay_);
    node_->declare_parameter("ciencia.w", cienciaw_);

    node_->declare_parameter("historia.x", historiax_);
    node_->declare_parameter("historia.y", historiay_);
    node_->declare_parameter("historia.w", historiaw_);

    node_->declare_parameter("literatura.x", literaturax_);
    node_->declare_parameter("literatura.y", literaturay_);
    node_->declare_parameter("literatura.w", literaturaw_);

    node_->declare_parameter("infantil.x", infantilx_);
    node_->declare_parameter("infantil.y", infantily_);
    node_->declare_parameter("infantil.w", infantilw_);

    node_->declare_parameter("id_infantil", id_infantil_);
    node_->declare_parameter("id_literatura", id_literatura_);
    node_->declare_parameter("id_ciencia", id_ciencia_);
    node_->declare_parameter("id_historia", id_historia_);

    object_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
     "/objects", 10, std::bind(&Search::object_callback,this, std::placeholders::_1));
    idx_ = 0;
    last_object_ = NULL;
}


void
Search::object_callback(std_msgs::msg::Float32MultiArray::UniquePtr msg)
{
  last_object_ = std::move(msg);
  id_ = msg->data[0];
}
void library_lib::Search::halt()
{

}
BT::NodeStatus
Search::tick()
{
  switch(id_)
  {
    case id_literatura_:
      node_->get_parameter("literatura.x", x);
      node_->get_parameter("literatura.y", y);
      node_->get_parameter("literatura.w", w);

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = w;
      wp_.pose.position.x = x;
      wp_.pose.position.y = y;

      setOutput("waypoint", wp_);

      return BT::NodeStatus::SUCCESS;
    case id_historia_:
      node_->get_parameter("historia.x", x);
      node_->get_parameter("historia.y", y);
      node_->get_parameter("historia.w", w);

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = w;
      wp_.pose.position.x = x;
      wp_.pose.position.y = y;

      setOutput("waypoint", wp_);

      return BT::NodeStatus::SUCCESS;
    case id_infantil_:
      node_->get_parameter("infantil.x", x);
      node_->get_parameter("infantil.y", y);
      node_->get_parameter("infantil.w", w);

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = w;
      wp_.pose.position.x = x;
      wp_.pose.position.y = y;

      setOutput("waypoint", wp_);

      return BT::NodeStatus::SUCCESS;
    case id_ciencia_:
      node_->get_parameter("ciencia.x", x);
      node_->get_parameter("ciencia.y", y);
      node_->get_parameter("ciencia.w", w);

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = w;
      wp_.pose.position.x = x;
      wp_.pose.position.y = y;

      setOutput("waypoint", wp_);
      return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<library_lib::Search>("SearchQR");
}
