
#include "library_lib/ReadQR.hpp"

enum{
  id_ciencia_ = 2,
  id_literatura_ = 3,
  id_historia_ = 4,
  id_infantil_ = 5,
};
namespace library_lib
{

ReadQR::ReadQR(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf)
{


    config().blackboard->get("node", node_);

    object_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
     "/objects", 10, std::bind(&ReadQR::object_callback,this, std::placeholders::_1));
    last_object_ = NULL;
}


void
ReadQR::object_callback(std_msgs::msg::Float32MultiArray::UniquePtr msg)
{
  last_object_ = std::move(msg);
  id_ = msg->data[0];
}
void library_lib::ReadQR::halt()
{

}
BT::NodeStatus
ReadQR::tick()
{
  double x,y,w;
  switch(id_)
  {
    case id_literatura_:

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = w;
      wp_.pose.position.x = x;
      wp_.pose.position.y = y;

      setOutput("waypoint", wp_);

      return BT::NodeStatus::SUCCESS;
    case id_historia_:

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = w;
      wp_.pose.position.x = x;
      wp_.pose.position.y = y;

      setOutput("waypoint", wp_);

      return BT::NodeStatus::SUCCESS;
    case id_infantil_:

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = w;
      wp_.pose.position.x = x;
      wp_.pose.position.y = y;

      setOutput("waypoint", wp_);

      return BT::NodeStatus::SUCCESS;
    case id_ciencia_:

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
  factory.registerNodeType<library_lib::ReadQR>("ReadQR");
}
