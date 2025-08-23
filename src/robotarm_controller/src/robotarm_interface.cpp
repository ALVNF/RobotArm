#include "robotarm_controller/robotarm_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace robotarm_controller
{
    RobotarmInterface::RobotarmInterface(){}

    RobotarmInterface::~RobotarmInterface(){
        if(arduino_.IsOpen()){
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong while closing the connection with port " << port_);
            }
            
        }
    }

    CallbackReturn RobotarmInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if(result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch(const std::exception& e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("RobotarmInterface"), "No Serial Port provided! Aborting");
            return CallbackReturn::FAILURE;
        }

        position_commands_.assign(info_.joints.size(), 0.0);
        position_states_.assign(info_.joints.size(), 0.0);
        previous_position_commands_.assign(info_.joints.size(), 0.0);

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RobotarmInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION,
                    &position_states_[i]
                )
            );
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotarmInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION,
                    &position_commands_[i]
                )
            );
        }
        return command_interfaces;
    }

    CallbackReturn RobotarmInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotarmInterface"), "Starting the robot hardware");

        std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
        std::fill(previous_position_commands_.begin(), previous_position_commands_.end(), 0.0);
        std::fill(position_states_.begin(), position_states_.end(), 0.0);

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong while interacting with the port " << port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("RobotarmInterface"), "Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotarmInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotarmInterface"), "Stoping the robot hardware");

        try
        {
            arduino_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong while closing the connection with the port " << port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("RobotarmInterface"), "Hardware stopped");
        return CallbackReturn::SUCCESS;
    }


    hardware_interface::return_type RobotarmInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // Not the proper way, harware not providing feedback for reached position, so making assumptions
        position_states_ = position_commands_;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotarmInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        if(position_commands_ == previous_position_commands_)
        {
            return hardware_interface::return_type::OK;
        }

        // Positions structure:
        // b43 --> base<angle>
        // s92 --> shoulder<angle>
        // e30 --> elbow<angle>
        // g0 --> gripper<angle>
        // All together (b43, s92, e30, g0) will be the message send through serial port
        std::string msg;

        int base = static_cast<int>(((position_commands_.at(0) + (M_PI / 2)) * 180) / M_PI);
        msg.append("b");
        msg.append(std::to_string(base));
        msg.append(",");
        
        int shoulder = static_cast<int>(((position_commands_.at(1) + (M_PI / 2)) * 180) / M_PI);
        msg.append("s");
        msg.append(std::to_string(shoulder));
        msg.append(",");

        int elbow = static_cast<int>(((position_commands_.at(2) + (M_PI / 2)) * 180) / M_PI);
        msg.append("e");
        msg.append(std::to_string(elbow));
        msg.append(",");

        int gripper = static_cast<int>((-position_commands_.at(3) * 180) / (M_PI/2));
        msg.append("g");
        msg.append(std::to_string(gripper));
        msg.append(",");

        try
        {
            arduino_.Write(msg);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("RobotarmInterface"), "Writting msg to arduino: " << msg);
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong while sending the message " << msg);
            return hardware_interface::return_type::ERROR;
        }

        previous_position_commands_ = position_commands_;
        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(robotarm_controller::RobotarmInterface, hardware_interface::SystemInterface);
