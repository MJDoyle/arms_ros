#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>

// Minimal ROS2 includes - avoid the heavy rclcpp.hpp
#include <rclcpp/node.hpp>
#include <rclcpp/init.hpp>
#include <rclcpp/spin.hpp>
#include <rclcpp/executors.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <iostream>
#include <string>
#include <memory>

using namespace webots;

class WebotsSupervisorController : public rclcpp::Node
{
private:
    std::unique_ptr<Supervisor> supervisor_;
    Node* root_node_;
    Field* children_field_;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
    
    int time_step_;

public:
    WebotsSupervisorController() : 
        Node("webots_supervisor_controller"),
        supervisor_(std::make_unique<Supervisor>()),
        time_step_(32)
    {
        if (!initializeWebots()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Webots supervisor");
            return;
        }
        
        initializeROS();
        RCLCPP_INFO(this->get_logger(), "Webots Supervisor Controller initialized");
    }

private:
    bool initializeWebots() {
        try {
            root_node_ = supervisor_->getRoot();
            if (!root_node_) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get root node");
                return false;
            }
            
            children_field_ = root_node_->getField("children");
            if (!children_field_) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get children field");
                return false;
            }
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during Webots initialization: %s", e.what());
            return false;
        }
    }
    
    void initializeROS() {
        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "webots_supervisor_commands", 10,
            std::bind(&WebotsSupervisorController::commandCallback, this, std::placeholders::_1));
        
        status_pub_ = this->create_publisher<std_msgs::msg::Bool>("webots_supervisor_status", 10);
        
        RCLCPP_INFO(this->get_logger(), "ROS2 communications initialized");
    }
    
    void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received command: %s", msg->data.c_str());
        
        try {
            if (msg->data.find("DEF") != std::string::npos) {
                addNodeToScene(msg->data);
            } else if (msg->data.find("REMOVE") != std::string::npos) {
                removeNodeFromScene(msg->data);
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown command type");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing command: %s", e.what());
        }
    }
    
    void addNodeToScene(const std::string& node_definition) {
        try {
            children_field_->importMFNodeFromString(-1, node_definition);
            RCLCPP_INFO(this->get_logger(), "Successfully added node to scene");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to add node: %s", e.what());
        }
    }
    
    void removeNodeFromScene(const std::string& command) {
        std::string node_name = command.substr(7);
        try {
            Node* node = supervisor_->getFromDef(node_name);
            if (node) {
                node->remove();
                RCLCPP_INFO(this->get_logger(), "Removed node: %s", node_name.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to remove node: %s", e.what());
        }
    }

public:
    void step() {
        if (supervisor_->step(time_step_) == -1) {
            RCLCPP_INFO(this->get_logger(), "Simulation ended");
            rclcpp::shutdown();
            return;
        }
        
        // Publish status
        std_msgs::msg::Bool status_msg;
        status_msg.data = true;
        status_pub_->publish(status_msg);
        
        rclcpp::spin_some(this->shared_from_this());
    }
    
    bool isRunning() {
        return rclcpp::ok() && supervisor_->step(0) != -1;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto controller = std::make_shared<WebotsSupervisorController>();
        
        while (controller->isRunning()) {
            controller->step();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}