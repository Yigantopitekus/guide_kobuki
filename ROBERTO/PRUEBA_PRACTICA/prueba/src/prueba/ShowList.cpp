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
#include "prueba/ShowList.hpp"
#include <vector>
#include <iostream>

namespace prueba
{

using namespace std::chrono_literals;

ShowList::ShowList(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

    poses.push_back(create_pose(5.431477069854736, -19.797990798950195));
  poses.push_back(create_pose(3.2335450649261475, 2.0638961791992188));
  poses.push_back(create_pose(21.521137237548828, -21.340070724487305));
  poses.push_back(create_pose(22.671804428100586, -7.758427143096924));
  poses.push_back(create_pose(41.00392532348633, -7.544219970703125));
  poses.push_back(create_pose(40.77971649169922,-20.293777465820312));

  nombres.push_back("laboratorios linux");
  nombres.push_back("mesa profesor");
  nombres.push_back("baño hombres");
  nombres.push_back("escalera");
  nombres.push_back("clase probabilidad");
  nombres.push_back("clase enrique");
}

geometry_msgs::msg::PoseStamped 
ShowList::create_pose(double x, double y) {

  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}

void ShowList::halt()
{

}

void 
ShowList::printWaypoint(const geometry_msgs::msg::PoseStamped & wp, std::string str) 
{
    std::cout << str << ":\n";
    std::cout << "  Position -> x: " << wp.pose.position.x
      << ", y: " << wp.pose.position.y << "\n";
}

void 
ShowList::printLista(const std::vector<geometry_msgs::msg::PoseStamped>& lista) {

  std::cout << "Botón 0 para navegar. 1 para bajar posicion en la lista, 2 para subir" << ":\n";

  for (size_t i = 0; i < lista.size(); ++i) {
    printWaypoint(lista[i], nombres[i]);
  }
}

BT::NodeStatus
ShowList::tick()
{
    printLista(poses);

    setOutput("waypoints", poses);
    setOutput("nombres", nombres);

    return BT::NodeStatus::SUCCESS;
  
}

}  // namespace bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<prueba::ShowList>("ShowList");
}