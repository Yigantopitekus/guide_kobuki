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

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts()
    {
      return BT::PortsList(
        {
        });
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    bool image_saved_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    std::string getTimestamp() const;
};

} // namespace library_lib

#endif 
