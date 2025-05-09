
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
  node_->get_parameter("ciencia.x", cienciax_);
  node_->get_parameter("ciencia.y", cienciay_);
  node_->get_parameter("ciencia.w", cienciaw_);

  node_->get_parameter("literatura.x", literaturax_);
  node_->get_parameter("literatura.y", literaturay_);
  node_->get_parameter("literatura.w", literaturaw_);

  node_->get_parameter("infantil.x", infantilx_);
  node_->get_parameter("infantil.y", infantily_);
  node_->get_parameter("infantil.w", infantilw_);

  node_->get_parameter("historia.x", historiax_);
  node_->get_parameter("historia.y", historiay_);
  node_->get_parameter("historia.w", historiaw_);

  object_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/objects", 10, std::bind(&ReadQR::object_callback,this, std::placeholders::_1));
  turn_ = false;



}


void
ReadQR::object_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg)
{

  if(msg != nullptr && !msg->data.empty())
  {
    last_object_ = *msg;
    id_ = msg->data[0];
  }
}
void library_lib::ReadQR::halt()
{

}
BT::NodeStatus
ReadQR::tick()
{

  if(last_object_.data.empty())
  {  
    return BT::NodeStatus::RUNNING;
  }
  switch(id_)
  {
    case id_literatura_:
      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = literaturaw_;
      wp_.pose.position.x = literaturax_;
      wp_.pose.position.y = literaturay_;

      config().blackboard->set("waypoint", wp_);

      return BT::NodeStatus::SUCCESS;
    case id_historia_:

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = historiaw_;
      wp_.pose.position.x = historiax_;
      wp_.pose.position.y = historiay_;

      config().blackboard->set("waypoint", wp_);

      return BT::NodeStatus::SUCCESS;
    case id_infantil_:

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = infantilw_;
      wp_.pose.position.x = infantilx_;
      wp_.pose.position.y = infantily_;

      config().blackboard->set("waypoint", wp_);

      return BT::NodeStatus::SUCCESS;
    case id_ciencia_:

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = cienciaw_;
      wp_.pose.position.x = cienciax_;
      wp_.pose.position.y =  cienciay_;

      config().blackboard->set("waypoint", wp_);
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
