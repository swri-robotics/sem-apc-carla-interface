#include "rclcpp/rclcpp.hpp"

#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using CarCmd = carla_msgs::msg::CarlaEgoVehicleControl;
using Float64 = std_msgs::msg::Float64;

/**
 * carla_vehicle_interface node:
 * This node is a wrapper used to interface with the Carla ROS bridge.
 * It provides a layer of abstraction for students to control the ego
 * vehicle in a more managable way.
 */

namespace carla_shell_bridge
{
    class CarlaSimulationVehicleInterface : public rclcpp::Node
    {
    public:
        explicit CarlaSimulationVehicleInterface(const rclcpp::NodeOptions &options) : rclcpp::Node("carla_simulation_vehicle_interface", options)
        {
            RCLCPP_INFO(get_logger(), "Initializing simulation vehicle interface");
            // Create publishers before subscribers to prevent issues
            LoadParams();
            EstablishPublishers();
            EstablishSubscriptions();
            EstablishTimers();
        }

    private:
        bool handbrake_ = false;
        bool invert_steering_ = false;
        bool manual_control_ = false;
        float brake_ = 0.0;
        float steering_ = 0.0;
        float throttle_ = 0.0;

        std::string gear_input_;
        std_msgs::msg::Bool manual_override_msg;
        CarCmd carla_control_;

        rclcpp::TimerBase::SharedPtr update_car_cmd_;

        // Publishers
        rclcpp::Publisher<CarCmd>::SharedPtr car_cmd_pub_;

        // Subscribers
        rclcpp::Subscription<Float64>::SharedPtr brake_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gear_sub_;
        rclcpp::Subscription<Float64>::SharedPtr steering_sub_;
        rclcpp::Subscription<Float64>::SharedPtr throttle_sub_;

        void LoadParams()
        {
            RCLCPP_INFO(get_logger(), "Loading parameters");
            this->declare_parameter("ego_vehicle.invert_steering", false);
            this->get_parameter("ego_vehicle.invert_steering", invert_steering_);
        }

        void EstablishPublishers()
        {
            RCLCPP_INFO(get_logger(), "Establishing publishers");
            car_cmd_pub_ = create_publisher<CarCmd>("carla/ego_vehicle/vehicle_control_cmd", 10);
        }

        void EstablishSubscriptions()
        {
            RCLCPP_INFO(get_logger(), "Establishing subscribers");
            brake_sub_ = create_subscription<Float64>("brake_command", 1,
                                                        std::bind(&CarlaSimulationVehicleInterface::HandleBrakeInput, this, std::placeholders::_1));
            gear_sub_ = create_subscription<std_msgs::msg::String>("gear_command", 1,
                                                        std::bind(&CarlaSimulationVehicleInterface::HandleTransmissionInput, this, std::placeholders::_1));
            steering_sub_ = create_subscription<Float64>("steering_command", 1,
                                                        std::bind(&CarlaSimulationVehicleInterface::HandleSteeringInput, this, std::placeholders::_1));
            throttle_sub_ = create_subscription<Float64>("throttle_command", 1,
                                                        std::bind(&CarlaSimulationVehicleInterface::HandleThrottleInput, this, std::placeholders::_1));
        }

        void EstablishTimers()
        {
            RCLCPP_INFO(get_logger(), "Establishing timer");
            // Send the control message to CARLA once every 100ms
            update_car_cmd_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&CarlaSimulationVehicleInterface::update, this));
        }

        // Callback methods
        void update()
        {
            auto car_cmd_msg = CarCmd();

            // Set manual override in Carla to true
            manual_override_msg.data = true;

            // Limit steering to [-1.0, 1.0]
            car_cmd_msg.steer = (invert_steering_ ? -1.0 : 1.0) * fmin(1.0, fmax(-1.0, steering_));

            // Configure throttle, brake, handbrake
            // car_cmd_msg.hand_brake = handbrake_;
            if (handbrake_ || fabs(brake_) >= 0.01)
            {
                // Hand brake is engaged, or brake is engaged
                car_cmd_msg.throttle = 0.0;
                car_cmd_msg.brake = brake_;
            }
            else
            {
                car_cmd_msg.throttle = throttle_;
                car_cmd_msg.brake = 0.0;
            }

            // Configure gearbox
            if (gear_input_ == "reverse")
            {
                // Set gearbox to reverse
                car_cmd_msg.gear = -1;
                car_cmd_msg.reverse = true;
            }
            else if (gear_input_ == "forward")
            {
                // Default case is forwards
                car_cmd_msg.gear = 1;
                car_cmd_msg.reverse = false;
            }
            else
            {
                // Set gearbox to neutral
                car_cmd_msg.gear = 0;
                car_cmd_msg.reverse = false;
            }
            car_cmd_msg.manual_gear_shift = true;

            if (!manual_control_)
            {
                car_cmd_pub_->publish(car_cmd_msg);
            }
        }

        void HandleBrakeInput(const Float64::SharedPtr msg)
        {
            brake_ = msg->data;
        }

        void HandleTransmissionInput(const std_msgs::msg::String::SharedPtr msg)
        {
            gear_input_ = msg->data;
        }

        void HandleSteeringInput(const Float64::SharedPtr msg)
        {
            steering_ = msg->data;
        }

        void HandleThrottleInput(const Float64::SharedPtr msg)
        {
            throttle_ = msg->data;
        }

    }; // class CarlaSimulationVehicleInterface

} // namespace carla_shell_bridge

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(carla_shell_bridge::CarlaSimulationVehicleInterface)