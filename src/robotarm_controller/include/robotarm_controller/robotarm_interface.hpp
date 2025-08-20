#ifndef ROBOTARM_INTERFACE_H
#define ROBOTARM_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <libserial/SerialPort.h>

#include <vector>
#include <string>


namespace robotarm_controller
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class RobotarmInterface : public hardware_interface::SystemInterface
    {
        public:
            RobotarmInterface();
            virtual ~RobotarmInterface();

            // Activa el hardware, se abren el puerto serie de aarduino, inicializa los valores de los motores o se resetean.
            // ROS llama a esta funcion cuando "arranca" el controlador
            virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
            // Se llama al apagar. Se descactiva el puerto serie, motores y limpia la memoria
            virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

            // Se llama al inicializar el driver, lee los parametros como el nombre de los joints, número de actuadores, puerto de serie
            // Configura los vectores (position_commands, position_states_) según los joints definidos en el URDF
            virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
            // Devuelve que estados ofrece el hardware a ROS2.
            // Por ejemplo: cada joint tiene un hardware_interface::StateInterface(joint_name, "position", &position_states_[i]).
            // Esto permite a ROS leer la posicion real de cada articulación
            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            // Devuelve que comandos acepta el haardware.
            // Por ejemplo: harware_interface::CommandInterface(joint_name, "position", &position_commands_[i])
            // Esto permite a ROS escribir comandos de posición que luego se envia al Arduino
            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            // Lee del hardware y actualiza los estados
            // Recibe del Arduino las posiciones reales de los motores y actualiza position_states_
            virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
            // Envia comandos al hardware
            // Agarra lo que haya en position_commands_ (lo que MoveIt/ros2_control quiere mover) y se manda por serial a Arduino.
            virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        private:
            LibSerial::SerialPort arduino_;
            std::string port_;

            std::vector<double> position_commands_;
            std::vector<double> previous_position_commands_;
            std::vector<double> position_states_;
    };
}

#endif