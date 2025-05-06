// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <iostream>

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "kobuki_ros_interfaces/msg/button_event.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "prueba/B1Pressed.hpp"
#include <vector>
#include <iostream>

namespace prueba
{

using namespace std::chrono_literals;
using namespace std::placeholders;

B1Pressed::B1Pressed(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
    
    button_sub_ = node_->create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "/events/button", 10, std::bind(&B1Pressed::button_callback, this, _1));
}

void
B1Pressed::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  last_button_ = std::move(msg);
}

BT::NodeStatus
B1Pressed::tick()
{
  

  if (last_button_->button == 1 && last_button_->state == 1) {
    
    geometry_msgs::msg::PoseStamped wp;
    std::vector<geometry_msgs::msg::PoseStamped> wps;
    size_t i;
    std::vector<std::string> nombres;

    if (!getInput("waypoints", wps)) {
      RCLCPP_ERROR(node_->get_logger(), "No se pudo obtener 'waypoints'");
      return BT::NodeStatus::FAILURE;
    }

    if (!getInput("i", i)) {
      RCLCPP_ERROR(node_->get_logger(), "No se pudo obtener 'i'");
      return BT::NodeStatus::FAILURE;
    }

    if (!getInput("nombres", nombres)) {
      RCLCPP_ERROR(node_->get_logger(), "No se pudo obtener 'nombres'");
      return BT::NodeStatus::FAILURE;
    }


    if (i < wps.size() - 1)
      i++;
    else
      i = 0;

    wp = wps[i];

    std::cout << "\n       Localización:         \n" << std::endl;
    std::cout << nombres[i] << ":\n";
    std::cout << "  Position -> x: " << wp.pose.position.x
      << ", y: " << wp.pose.position.y << "\n";

    setOutput("waypoint", wp);
    setOutput("i", i);


    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<prueba::B1Pressed>("B1Pressed");
}
