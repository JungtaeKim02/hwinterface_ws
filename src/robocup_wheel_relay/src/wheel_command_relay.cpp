#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robocup_wheel_relay
{
namespace
{
using Clock = std::chrono::steady_clock;
using CallbackReturn = controller_interface::CallbackReturn;
using InterfaceConfiguration = controller_interface::InterfaceConfiguration;
using InterfaceType = controller_interface::interface_configuration_type;
using ReturnType = controller_interface::return_type;
using sensor_msgs::msg::JointState;

const std::array<std::string, 3> joint_names = {"wheel_0_joint", "wheel_1_joint", "wheel_2_joint"};

struct WheelCommand
{
  std::array<double, joint_names.size()> velocity{};
  Clock::time_point received{};
};
}

class WheelCommandRelay : public controller_interface::ControllerInterface
{
public:
  CallbackReturn on_init() override
  {
    auto_declare<std::string>("input_topic", "/sim/wheel_commands");
    auto_declare<double>("command_timeout", 0.12);
    return CallbackReturn::SUCCESS;
  }

  InterfaceConfiguration command_interface_configuration() const override
  {
    InterfaceConfiguration config{InterfaceType::INDIVIDUAL, {}};
    for (const auto & joint : joint_names) {
      config.names.push_back(joint + "/velocity");
    }
    return config;
  }

  InterfaceConfiguration state_interface_configuration() const override
  {
    return {InterfaceType::NONE, {}};
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const auto timeout = get_node()->get_parameter("command_timeout").as_double();
    if (!std::isfinite(timeout) || timeout <= 0.0 || timeout > 1.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "command_timeout must be in (0, 1] seconds");
      return CallbackReturn::ERROR;
    }
    timeout_ = std::chrono::duration<double>(timeout);
    const auto topic = get_node()->get_parameter("input_topic").as_string();
    subscription_ = get_node()->create_subscription<JointState>(
      topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](const JointState & message) { receive(message); });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      const auto name = joint_names[i] + "/velocity";
      const auto found = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&name](const auto & interface) { return interface.get_name() == name; });
      if (found == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Missing command interface %s", name.c_str());
        return CallbackReturn::ERROR;
      }
      command_indexes_[i] = static_cast<std::size_t>(found - command_interfaces_.begin());
    }
    activated_at_ = Clock::now();
    return write({}) == ReturnType::OK ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return write({}) == ReturnType::OK ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    subscription_.reset();
    return CallbackReturn::SUCCESS;
  }

  ReturnType update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    const auto & command = *latest_.readFromRT();
    if (command.received < activated_at_ || Clock::now() - command.received >= timeout_) {
      return write({});
    }
    return write(command.velocity);
  }

private:
  void receive(const JointState & message)
  {
    WheelCommand command;
    if (message.name.size() != joint_names.size() || message.velocity.size() != joint_names.size()) {
      return;
    }
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      const auto found = std::find(message.name.begin(), message.name.end(), joint_names[i]);
      if (found == message.name.end()) {
        return;
      }
      const auto velocity = message.velocity[found - message.name.begin()];
      if (!std::isfinite(velocity)) {
        return;
      }
      command.velocity[i] = velocity;
    }
    command.received = Clock::now();
    latest_.writeFromNonRT(command);
  }

  ReturnType write(const std::array<double, joint_names.size()> & velocity)
  {
    bool written = true;
    for (std::size_t i = 0; i < velocity.size(); ++i) {
      written = command_interfaces_[command_indexes_[i]].set_value(velocity[i]) && written;
    }
    return written ? ReturnType::OK : ReturnType::ERROR;
  }

  std::array<std::size_t, joint_names.size()> command_indexes_{};
  Clock::time_point activated_at_{};
  std::chrono::duration<double> timeout_{0.12};
  realtime_tools::RealtimeBuffer<WheelCommand> latest_;
  rclcpp::Subscription<JointState>::SharedPtr subscription_;
};
}

PLUGINLIB_EXPORT_CLASS(robocup_wheel_relay::WheelCommandRelay, controller_interface::ControllerInterface)
