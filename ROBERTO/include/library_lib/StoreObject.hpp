#ifndef LIBRARY_LIB__STOREOBJECT_HPP_
#define LIBRARY_LIB__STOREOBJECT_HPP_

#include <string>
#include <iostream>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

namespace library_lib {

class StoreObject : public BT::ActionNodeBase {
public:
    explicit StoreObject(const std::string &xml_tag_name, const BT::NodeConfiguration &conf);

    void halt() override;

    // Se cambia on_tick() en lugar de tick() para el patrón de comportamiento
    void on_tick() override;
    
    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
      return BT::PortsList(
        {
        });
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    bool image_saved_;  // Indica si la imagen ha sido guardada

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    std::string getTimestamp() const;
};

} // namespace library_lib

#endif
