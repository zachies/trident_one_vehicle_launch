#ifndef __INTERFACE_H__
#define __INTERFACE_H__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>

#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>

class TridentOneInterface : public rclcpp::Node
{
    public:
        TridentOneInterface();
    private:
        /* autoware subscribers */
        rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr _control_command_sub;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr _gear_cmd_sub;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr _turn_indicators_cmd_sub;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr _hazard_lights_cmd_sub;

        /* autoware publishers */
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr _control_mode_pub;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr _vehicle_twist_pub;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr _steering_status_pub;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr _gear_status_pub;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr _turn_indicators_status_pub;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr _hazard_lights_status_pub;

        /* publisher to eli's cute microros interface */
        rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr _micros_pub;

        void callbackControlCmd(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
        void callbackGearCmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
        void callbackTurnIndicatorsCommand(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
        void callbackHazardLightsCommand(const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
        void callbackEngage(const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg);

        autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr _control_cmd_ptr;

        float map(float x, float in_min, float in_max, float out_min, float out_max);
};

#endif /* trident_one_interface.h */