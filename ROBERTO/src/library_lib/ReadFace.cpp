
#include "library_lib/ReadFace.hpp"

enum{
  id_Roberto_ = 6,
  id_Hugo_ = 8,
  id_Mateo_ = 7,
};
namespace library_lib
{

ReadFace::ReadFace(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf)
{
    config().blackboard->get("node", node_);

  object_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/objects", 10, std::bind(&ReadFace::object_callback,this, std::placeholders::_1));
    std::cout << "Acerque su cara a la camara" << std::endl;

}


void
ReadFace::object_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if(msg != nullptr && !msg->data.empty())
  {
    last_object_ = *msg;
    id_ = msg->data[0];
  }
}
void library_lib::ReadFace::halt()
{

}
BT::NodeStatus
ReadFace::tick()
{

  if(last_object_.data.empty())
  {  
    return BT::NodeStatus::RUNNING;
  }
  switch(id_)
  {
    case id_Roberto_:
    std::cout << "Bienvenido Roberto" << std::endl;
      return BT::NodeStatus::SUCCESS;
    case id_Hugo_:
    std::cout << "Bienvenido Hugo" << std::endl;

        return BT::NodeStatus::SUCCESS;  
    case id_Mateo_:
    std::cout << "Bienvenido Mateo" << std::endl;

        return BT::NodeStatus::SUCCESS;
    
  }
  return BT::NodeStatus::RUNNING;

}
}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<library_lib::ReadFace>("ReadFace");
}
