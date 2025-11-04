#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <ctime>

// Your custom service interface
#include "assembler_interfaces/srv/load_model.hpp"

class WebotsModelLoader : public rclcpp::Node
{
public:
    WebotsModelLoader() : Node("webots_model_loader")
    {
        // Create the service to load models into Webots
        load_model_service_ = this->create_service<assembler_interfaces::srv::LoadModel>(
            "load_model_to_webots",
            std::bind(&WebotsModelLoader::loadModelCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Publisher to send commands to the Webots supervisor controller
        //webots_command_pub_ = this->create_publisher<std_msgs::msg::String>(
        //    "webots_supervisor_commands", 10);
        
        // Create temporary directory for STL files if it doesn't exist
        temp_dir_ = "/tmp/webots_models";
        std::filesystem::create_directories(temp_dir_);
        
        RCLCPP_INFO(this->get_logger(), "Webots Model Loader service started");
        RCLCPP_INFO(this->get_logger(), "Service available at: load_model_to_webots");
        RCLCPP_INFO(this->get_logger(), "Publishing commands to: webots_supervisor_commands");
    }

private:
    void loadModelCallback(
        const std::shared_ptr<assembler_interfaces::srv::LoadModel::Request> request,
        std::shared_ptr<assembler_interfaces::srv::LoadModel::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to load model: %s", 
                   request->model_name.c_str());
        
        try {
            // Convert the model data to STL format
            //std::string stl_path = processModelData(request->model_data, request->model_name);
            
            //if (stl_path.empty()) {
            //    throw std::runtime_error("Failed to process model data");
            //}
            
            // // Create the Webots node definition
            // std::string webots_def = createWebotsNodeDefinition(
            //     stl_path, 
            //     request->pose, 
            //     request->model_name,
            //     request->scale
            // );
            
            // // Send the command to the Webots supervisor
            // std_msgs::msg::String command_msg;
            // command_msg.data = webots_def;
            // webots_command_pub_->publish(command_msg);
            
            // Prepare successful response
            response->success = true;
            response->message = "Model '" + request->model_name + "' loaded successfully";
            response->webots_node_name = request->model_name;
            
            RCLCPP_INFO(this->get_logger(), "Successfully processed model: %s", 
                       request->model_name.c_str());
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Failed to load model: ") + e.what();
            response->webots_node_name = "";
            
            RCLCPP_ERROR(this->get_logger(), "Model loading failed: %s", e.what());
        }
    }
    
    // std::string processModelData(const std::string& model_data, const std::string& model_name)
    // {
    //     // Generate unique filename with timestamp
    //     auto timestamp = std::time(nullptr);
    //     std::string filename = model_name + "_" + std::to_string(timestamp) + ".stl";
    //     std::string output_path = temp_dir_ + "/" + filename;
        
    //     // If model_data is a file path, copy/convert it
    //     if (std::filesystem::exists(model_data)) {
    //         RCLCPP_INFO(this->get_logger(), "Processing model file: %s", model_data.c_str());
            
    //         // Check if it's already an STL file
    //         if (model_data.substr(model_data.find_last_of(".") + 1) == "stl") {
    //             // Just copy the STL file
    //             std::filesystem::copy_file(model_data, output_path);
    //             RCLCPP_INFO(this->get_logger(), "Copied STL file to: %s", output_path.c_str());
    //         } else {
    //             // Here you would implement your OpenCascade conversion
    //             // For now, placeholder implementation
    //             RCLCPP_WARN(this->get_logger(), 
    //                        "Non-STL file conversion not yet implemented. Expected STL file.");
                
    //             // TODO: Implement OpenCascade to STL conversion
    //             /*
    //             Example OpenCascade conversion code:
                
    //             #include <STEPCAFControl_Reader.hxx>
    //             #include <StlAPI_Writer.hxx>
    //             #include <TopoDS_Shape.hxx>
                
    //             STEPCAFControl_Reader reader;
    //             if (reader.ReadFile(model_data.c_str()) == IFSelect_RetDone) {
    //                 reader.TransferRoots();
    //                 TopoDS_Shape shape = reader.OneShape();
                    
    //                 StlAPI_Writer stl_writer;
    //                 stl_writer.Write(shape, output_path.c_str());
    //             }
    //             */
                
    //             return ""; // Return empty on failure for now
    //         }
    //     } else {
    //         // Assume model_data contains raw geometry data
    //         RCLCPP_INFO(this->get_logger(), "Processing raw model data");
            
    //         // TODO: Process raw geometry data and convert to STL
    //         // This would depend on your specific data format
    //         RCLCPP_WARN(this->get_logger(), "Raw geometry processing not yet implemented");
    //         return "";
    //     }
        
    //     return output_path;
    // }
    
    // std::string createWebotsNodeDefinition(const std::string& stl_path, 
    //                                       const geometry_msgs::msg::Pose& pose,
    //                                       const std::string& name,
    //                                       double scale)
    // {
    //     std::stringstream def;
        
    //     // Create a Webots Solid node definition
    //     def << "DEF " << sanitizeNodeName(name) << " Solid {\n";
        
    //     // Set position
    //     def << "  translation " << pose.position.x << " " 
    //         << pose.position.y << " " << pose.position.z << "\n";
        
    //     // Convert quaternion to axis-angle for Webots rotation
    //     auto [axis_x, axis_y, axis_z, angle] = quaternionToAxisAngle(pose.orientation);
    //     def << "  rotation " << axis_x << " " << axis_y << " " << axis_z << " " << angle << "\n";
        
    //     // Add scaling if specified
    //     if (scale != 1.0) {
    //         def << "  scale " << scale << " " << scale << " " << scale << "\n";
    //     }
        
    //     // Add the geometry
    //     def << "  children [\n";
    //     def << "    Shape {\n";
    //     def << "      geometry Mesh {\n";
    //     def << "        url \"" << stl_path << "\"\n";
    //     def << "      }\n";
    //     def << "      appearance PBRAppearance {\n";
    //     def << "        baseColor 0.8 0.8 0.8\n";
    //     def << "        metalness 0.0\n";
    //     def << "        roughness 0.4\n";
    //     def << "      }\n";
    //     def << "    }\n";
    //     def << "  ]\n";
        
    //     // Add basic physics if needed
    //     def << "  boundingObject Mesh {\n";
    //     def << "    url \"" << stl_path << "\"\n";
    //     def << "  }\n";
    //     def << "  physics Physics {\n";
    //     def << "    density -1\n";  // Use mesh density
    //     def << "  }\n";
        
    //     def << "}\n";
        
    //     RCLCPP_DEBUG(this->get_logger(), "Generated Webots definition:\n%s", def.str().c_str());
        
    //     return def.str();
    // }
    
    // std::string sanitizeNodeName(const std::string& name)
    // {
    //     std::string sanitized = name;
    //     // Replace invalid characters with underscores
    //     std::replace_if(sanitized.begin(), sanitized.end(), 
    //                    [](char c) { return !std::isalnum(c) && c != '_'; }, '_');
        
    //     // Ensure it starts with a letter or underscore
    //     if (!sanitized.empty() && std::isdigit(sanitized[0])) {
    //         sanitized = "_" + sanitized;
    //     }
        
    //     return sanitized;
    // }
    
    // std::tuple<double, double, double, double> quaternionToAxisAngle(
    //     const geometry_msgs::msg::Quaternion& quat)
    // {
    //     // Convert quaternion to axis-angle representation
    //     double w = quat.w;
    //     double x = quat.x;
    //     double y = quat.y;
    //     double z = quat.z;
        
    //     double angle = 2.0 * std::acos(std::abs(w));
    //     double sin_half_angle = std::sqrt(1.0 - w * w);
        
    //     double axis_x, axis_y, axis_z;
        
    //     if (sin_half_angle < 1e-6) {
    //         // Avoid division by zero - no rotation
    //         axis_x = 1.0;
    //         axis_y = 0.0;
    //         axis_z = 0.0;
    //         angle = 0.0;
    //     } else {
    //         axis_x = x / sin_half_angle;
    //         axis_y = y / sin_half_angle;
    //         axis_z = z / sin_half_angle;
    //     }
        
    //     return {axis_x, axis_y, axis_z, angle};
    // }

    // Member variables
    rclcpp::Service<assembler_interfaces::srv::LoadModel>::SharedPtr load_model_service_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr webots_command_pub_;
    std::string temp_dir_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<WebotsModelLoader>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Webots Model Loader node...");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}