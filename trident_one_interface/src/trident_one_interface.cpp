#include "trident_one_interface.hpp"

#include <vector>

TridentOneInterface::TridentOneInterface() : rclcpp::Node("trident_one_interface")
{
    using std::placeholders::_1;

    _control_command_sub = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", 1, bind(&TridentOneInterface::callbackControlCmd, this, _1)
    );

    _micros_pub = create_publisher<std_msgs::msg::Int16MultiArray>(
        "/hardware_subscriber", rclcpp::QoS{1}
    );
}

void TridentOneInterface::callbackControlCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
    _control_cmd_ptr = msg;

    float desired_steering_angle = _control_cmd_ptr->lateral.steering_tire_angle;
    float desired_accel = _control_cmd_ptr->longitudinal.acceleration;

    bool brake = desired_accel < 0;

    float output_throttle = 0, output_brake = 0, output_steer = 0;
    if(brake)
    {   
        output_brake = map(desired_accel, -1.5, 0, 10000, 0);
    }
    else
    {
        output_throttle = map(desired_accel, 0, 1, 0, 100);
    }

    output_steer = map(desired_steering_angle, -0.66, 0.66, -405, 405);

    std_msgs::msg::Int16MultiArray outputs;

    // setup dimensions
    outputs.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    outputs.layout.dim[0].size = 3;
    outputs.layout.dim[0].stride = 1;
    outputs.layout.dim[0].label = "yeehaw";

    // copy in the data
    outputs.data.clear();
    outputs.data.push_back((int16_t) output_steer);
    outputs.data.push_back((int16_t) output_brake);
    outputs.data.push_back((int16_t) output_throttle);

    _micros_pub->publish(outputs);
}

float TridentOneInterface::map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}