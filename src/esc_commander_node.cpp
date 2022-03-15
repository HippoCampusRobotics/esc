
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/rclcpp.hpp>

#include "afro_esc.h"

class ESC : public rclcpp::Node {
 public:
  ESC() : Node("esc_commander") { Init(); }
  void Init() { InitParams(); }
  void InitParams() {
    std::string param_name = "rx_timeout";
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0.0;
    descriptor.floating_point_range[0].to_value = 1.0;
    descriptor.description = "Timeout for motor setpoints.";
    this->declare_parameter<double>(param_name, 0.5, descriptor);

    param_name = "i2c_addresses";
    descriptor.type =
        rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 0;
    descriptor.integer_range[0].to_value = 127;
    descriptor.integer_range[0].step = 1;
    std::vector<int> default_value = {41, 42, 43, 44};
    this->declare_parameter <
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ESC>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
