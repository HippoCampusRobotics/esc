
#include <fcntl.h>

#include <hippo_interfaces/msg/actuator_controls.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include "afro_esc.h"

using std::placeholders::_1;

class ESC : public rclcpp::Node {
 public:
  ESC()
      : Node("esc_commander"),
        i2c_addresses_(8),
        i2c_device_("/dev/i2c-1"),
        timed_out_(false) {
    Init();
  }
  void Init() {
    RCLCPP_INFO(get_logger(), "Opening i2c device: %s", i2c_device_.c_str());
    i2c_handle_ = open(i2c_device_.c_str(), O_RDWR);
    if (i2c_handle_ < 0) {
      RCLCPP_FATAL(get_logger(), "Could not open i2c device: '%s'",
                   i2c_device_.c_str());
      exit(1);
    }
    InitEscs();
    InitParams();
    paramter_callback_handle_ = add_on_set_parameters_callback(std::bind(&ESC::onSetParameters, this, _1));
    control_timeout_timer_ =
        rclcpp::create_timer(this, get_clock(), std::chrono::seconds(1),
                             std::bind(&ESC::onTimeout, this));
    send_thrust_timer_ =
        rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(20),
                             std::bind(&ESC::OnSendThrusts, this));

    actuator_controls_sub_ =
        create_subscription<hippo_interfaces::msg::ActuatorControls>(
            "thruster_controls", 10,
            std::bind(&ESC::OnThrusterControls, this, _1));
  }
  void onTimeout() {
    RCLCPP_WARN(get_logger(), "Thruster controls timed out.");
    for (auto &esc : escs_) {
      esc.SetMotorSpeed(0.0);
    }
    SendThrusts(true);
    timed_out_ = true;
    control_timeout_timer_->cancel();
  }

  void OnSendThrusts() { SendThrusts(); }

  void OnThrusterControls(
      const hippo_interfaces::msg::ActuatorControls::SharedPtr msg) {
    control_timeout_timer_->reset();
    if (timed_out_) {
      timed_out_ = false;
      SetAllThrusts(0.0);
      SendThrusts(true);
    }
    for (int i = 0; i < msg->control.size(); i++) {
      if (msg->control[i] != 0 && !escs_[i].InUse()) {
        RCLCPP_WARN(get_logger(),
                    "Setting non-zero thrust for unused ESC at index %d!",
                    i);
      }
      escs_[i].SetMotorSpeed(msg->control[i]);
    }
  }

  void SetAllThrusts(double _thrust) {
    for (auto &esc : escs_) {
      esc.SetMotorSpeed(_thrust);
    }
  }

  int SendThrusts(bool force = false) {
    int ret = 0;
    if (timed_out_ && !force) {
      return 0;
    }
    for (auto &esc : escs_) {
      if (esc.available() && esc.WriteMotorSpeed() < 0) {
        RCLCPP_ERROR(get_logger(),
                     "Failed to set motor speed for thruster %d at address %X",
                     esc.index(), esc.address());
        ret = -1;
      }
    }
    return ret;
  }

  void InitParams() {
    RCLCPP_INFO(get_logger(), "Declaring parameters...");
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
    std::vector<int64_t> default_value = {41, 42, 43, 44, -1, -1, -1, -1};
    this->declare_parameter<std::vector<int64_t>>(param_name, default_value,
                                                  descriptor);
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool param_valid;
    for (const auto &param : parameters) {
      param_valid = true;
      RCLCPP_INFO(get_logger(), "Received parameter: %s", param.get_name());
      if (param.get_name() == "i2c_addresses") {
        RCLCPP_INFO(get_logger(), "Setting i2c_addresses.");
        try {
          if (param.as_integer_array().size() != 8) {
            throw std::length_error("Need exactly 8 i2c addresses.");
          }
        } catch (const rclcpp::ParameterTypeException &e) {
          RCLCPP_ERROR(get_logger(), "Failed to set i2c_addresses: %s",
                       e.what());
          result.successful = false;
          result.reason = "Wrong paramter type for i2c_addresses";
          param_valid = false;
        } catch (const std::length_error &e) {
          RCLCPP_ERROR(get_logger(), "Failed to set i2c_addresses: %s",
                       e.what());
          result.successful = false;
          result.reason =
              "Wrong length for i2c_addresses. Needs exactly 8 values.";
          param_valid = false;
        }
        if (param_valid) {
          i2c_addresses_ = param.as_integer_array();
          UpdateAddresses();
          DetectEscs();
        }
      }
    }
    return result;
  }

  void InitEscs() {
    RCLCPP_INFO(get_logger(), "Init ESCs....");
    int i = 0;
    escs_.clear();
    for (int i = 0; i < 8; i++) {
      escs_.push_back(AfroESC(i2c_handle_, 0));
      escs_.back().SetAvailable(false);
      escs_.back().SetInUse(false);
      escs_.back().SetMotorSpeed(0.0);
    }
  }

  void UpdateAddresses() {
    RCLCPP_INFO(get_logger(), "Updating ESC addresses...");
    int i = 0;
    for (auto const &address : i2c_addresses_) {
      escs_[i].SetAddress(address);
      escs_[i].SetAvailable(false);
      escs_[i].SetIndex(i);
      escs_[i].SetInUse(address > 0);
      i++;
    }
  }

  void DetectEscs() {
    RCLCPP_INFO(get_logger(), "Detecting ESCs...");
    bool failed = false;
    for (auto &esc : escs_) {
      if (!esc.InUse()) {
        RCLCPP_INFO(get_logger(), "ESC %d unused.", esc.index());
        continue;
      }
      if (!esc.VerifyID()) {
        RCLCPP_ERROR(get_logger(), "Could not find ESC at address %X",
                     esc.address());
        esc.SetAvailable(false);
        failed = true;
      } else {
        esc.SetAvailable(true);
        RCLCPP_INFO(get_logger(), "Detected ESC at address '%X'",
                    esc.address());
      }
    }
    esc_config_valid_ = !failed;
  }

 private:
  std::vector<int64_t> i2c_addresses_;
  bool esc_config_valid_;
  bool timed_out_;
  std::vector<AfroESC> escs_;
  std::string i2c_device_;
  int i2c_handle_;
  rclcpp::TimerBase::SharedPtr control_timeout_timer_;
  rclcpp::TimerBase::SharedPtr send_thrust_timer_;
  rclcpp::Subscription<hippo_interfaces::msg::ActuatorControls>::SharedPtr
      actuator_controls_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramter_callback_handle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ESC>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
