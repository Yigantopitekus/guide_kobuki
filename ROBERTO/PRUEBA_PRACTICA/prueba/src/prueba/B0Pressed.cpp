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
#include "prueba/B0Pressed.hpp"
#include <vector>
#include <iostream>

namespace prueba
{

using namespace std::chrono_literals;
using namespace std::placeholders;

B0Pressed::B0Pressed(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

    waypoint_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    
    button_sub_ = node_->create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "/events/button", 10, std::bind(&B0Pressed::button_callback, this, _1));
}


void
B0Pressed::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  last_button_ = std::move(msg);
}

BT::NodeStatus
B0Pressed::tick()
{
  

  if (last_button_->button == 0 && last_button_->state == 1) {
    
    geometry_msgs::msg::PoseStamped wp;
    if (!getInput("waypoint", wp)) {
      RCLCPP_ERROR(node_->get_logger(), "No se pudo obtener el waypoint de la blackboard.");
      return BT::NodeStatus::FAILURE;
    }

    waypoint_pub_->publish(wp);

    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<prueba::B0Pressed>("B0Pressed");
}
