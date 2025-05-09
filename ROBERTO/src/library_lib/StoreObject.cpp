#include "library_lib/StoreObject.hpp"

namespace library_lib {

StoreObject::StoreObject(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf), image_saved_(false) {
    config().blackboard->get("node", node_);
    RCLCPP_INFO(node_->get_logger(), "Nodo StoreObject inicializado");

    // Suscripción a la imagen
    image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/rgbd_camera/image", 10,
        std::bind(&StoreObject::imageCallback, this, std::placeholders::_1)
    );
}

void StoreObject::halt() {
    RCLCPP_INFO(node_->get_logger(), "StoreObject halted");
}

void StoreObject::on_tick() {
    RCLCPP_INFO(node_->get_logger(), "StoreObject ejecutándose...");

    // Si aún no se ha guardado la imagen, el nodo sigue ejecutándose
    if (!image_saved_) {
        RCLCPP_INFO(node_->get_logger(), "Esperando a capturar imagen...");
    }
}

BT::NodeStatus StoreObject::on_success() {
    // Si la imagen ha sido guardada correctamente, el nodo finaliza con éxito
    if (image_saved_) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

void StoreObject::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!image_saved_) {
        try {
            // Convertir la imagen de ROS a OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            // Nombre del archivo con timestamp
            std::string filename = "/home/hugo1234/Pictures/" + getTimestamp() + ".jpg";
            cv::imwrite(filename, cv_ptr->image);

            RCLCPP_INFO(node_->get_logger(), "Imagen guardada en: %s", filename.c_str());
            image_saved_ = true;  // Marca la imagen como guardada
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "Error al procesar la imagen: %s", e.what());
        }
    }
}

std::string StoreObject::getTimestamp() const {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&now_c));
    return std::string(buffer);
}

} // namespace library_lib

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<library_lib::StoreObject>("StoreObject");
}
