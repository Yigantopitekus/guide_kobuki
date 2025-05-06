
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
  factory.registerNodeType<library_lib::ReadQR>("ReadQR");
}
