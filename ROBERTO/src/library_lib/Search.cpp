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

Search::Search
(
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

    node_->declare_parameter("arr", arr_);
    node_->declare_parameter("size", size_);

    node_->get_parameter("arr", arr_);
    node_->get_parameter("size", size_);

    button_sub_ = node_->create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
     "/events/button", 10, std::bind(&Search::button_callback,this, std::placeholders::_1));
    idx_ = 0;
    last_button_ = NULL;
    Search::print_interface();

}
void Search::print_interface()
{
    system("clear");
    std::cout << "========= MENU DE DESTINOS =========" << std::endl;
    for (size_t i = 0; i < arr_.size(); ++i)
    {
        if (i == idx_)
        {
            std::cout << " -> [" << arr_[i] << "]" << std::endl;
        }
        else
        {
            std::cout << "    " << arr_[i] << std::endl;
        }
    }
    std::cout << "====================================" << std::endl;
    std::cout << "Botón 0: Siguiente destino" << std::endl;
    std::cout << "Botón 1: Destino anterior" << std::endl;
    std::cout << "Botón 2: Confirmar selección" << std::endl;
    std::cout << "====================================" << std::endl;
    std::cout << "Seleccionado: " << arr_[idx_] << std::endl;
}

void
Search::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  Search::print_interface();

  last_button_ = std::move(msg);
}
void library_lib::Search::halt()
{

}
BT::NodeStatus
Search::tick()
{
  if(last_button_ == NULL)
  {
      return BT::NodeStatus::RUNNING;
  }
   switch(last_button_->button)
        {
          case 0:
            if(last_button_->state == kobuki_ros_interfaces::msg::ButtonEvent::PRESSED)
            {
              idx_++;
              if(idx_ >= size_)
              {
                idx_ = 0;
              }
            }
            break;
          case 1:
            if(last_button_->state == kobuki_ros_interfaces::msg::ButtonEvent::PRESSED)
              {
                idx_--;
                if(idx_ < 0)
                {
                  idx_ = size_-1;
                }

              }
            break;
          case 2:
            last_button_ = NULL;
            std::string name = arr_[idx_];
            if(idx_ == 4)
            {
              idx_ = 0;
              return BT::NodeStatus::FAILURE;
            }
            idx_ = 0;


            double x, y, w;

            node_->get_parameter(name + ".x", x);
            node_->get_parameter(name + ".y", y);
            node_->get_parameter(name + ".w", w);

            wp_.header.frame_id = "map";
            wp_.pose.orientation.w = w;
            wp_.pose.position.x = x;
            wp_.pose.position.y = y;
            
            config().blackboard->set("waypoint", wp_);
            idx_ = 0;
            return BT::NodeStatus::SUCCESS;
      }
  return BT::NodeStatus::RUNNING;

}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<library_lib::Search>("Search");
}
