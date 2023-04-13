#include <trident_one_interface/trident_one_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TridentOneInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}