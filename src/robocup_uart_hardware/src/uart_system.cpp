#include "robocup_uart_hardware/uart_link.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robocup_uart_hardware
{
namespace
{
using CallbackReturn = hardware_interface::CallbackReturn;
using ReturnType = hardware_interface::return_type;
using LifecycleState = rclcpp_lifecycle::State;

struct PreviewMailbox
{
  std::mutex mutex;
  bool available{false};
  UartLink::CanPreview preview;
};

enum class DriveRequest {drive, disarm, rearm};

struct DriveStatus
{
  UartLink::State state;
  bool uart_open{false};
  bool commands_zero{true};
  std::string error;
};

struct DriveStatusMailbox
{
  std::mutex mutex;
  DriveStatus value;
};

diagnostic_msgs::msg::DiagnosticStatus format_drive_status(
  const DriveStatus & snapshot, const std::string & port,
  std::chrono::milliseconds timeout, bool disarm_latched)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  constexpr std::array drive_states{"DISARMED", "ARMING", "ARMED", "STOPPING", "FAULT"};
  constexpr std::array stop_reasons{
    "NONE", "DISARM_REQUEST", "UART_TIMEOUT", "CAN_OFFLINE", "DRIVE_FAULT",
    "FEEDBACK_STALE", "SESSION_CHANGED", "CAN_CONFIG"};
  const auto & state = snapshot.state;
  const auto age = state.received_at ? std::chrono::duration_cast<std::chrono::milliseconds>(
    UartLink::Clock::now() - *state.received_at) : std::chrono::milliseconds{-1};
  const bool fresh = snapshot.uart_open && state.received_at && age <= timeout;
  const bool known_state = state.received_at && state.drive_state < drive_states.size();
  const std::string drive_state = known_state ? drive_states[state.drive_state] : "UNKNOWN";

  DiagnosticStatus status;
  status.name = "robocup/uart_drive";
  status.hardware_id = port;
  if (!snapshot.error.empty()) {
    status.level = DiagnosticStatus::ERROR;
    status.message = snapshot.error;
  } else if (!fresh) {
    status.level = DiagnosticStatus::STALE;
    status.message = snapshot.uart_open ? "Waiting for a fresh MCU STATE" : "UART port closed";
  } else if (drive_state == "FAULT" || state.fault != 0) {
    status.level = DiagnosticStatus::ERROR;
    status.message = drive_state;
  } else {
    status.level = drive_state == "ARMED" && !disarm_latched ?
      DiagnosticStatus::OK : DiagnosticStatus::WARN;
    status.message = drive_state;
  }

  const auto add = [&status](std::string key, std::string value) {
      auto & field = status.values.emplace_back();
      field.key = std::move(key);
      field.value = std::move(value);
    };
  add("drive_state", drive_state);
  add("uart_open", snapshot.uart_open ? "true" : "false");
  add("state_fresh", fresh ? "true" : "false");
  add("joy_disarm_latched", disarm_latched ? "true" : "false");
  add("disarm_latched", disarm_latched ? "true" : "false");
  add("commands_zero", snapshot.commands_zero ? "true" : "false");
  if (state.received_at) {
    add("state_age_ms", std::to_string(age.count()));
    add("stop_reason", state.stop_reason < stop_reasons.size() ?
      stop_reasons[state.stop_reason] : "UNKNOWN");
    add("simulated", state.flags & 1U ? "true" : "false");
    add("online_mask", std::to_string(state.online));
    add("enabled_mask", std::to_string(state.enabled));
    add("feedback_valid_mask", std::to_string(state.feedback_valid));
    add("fault_mask", std::to_string(state.fault));
    for (std::size_t wheel = 0; wheel < UartLink::wheel_count; ++wheel) {
      const auto age_ms = state.feedback_age_ms[wheel];
      add("wheel_" + std::to_string(wheel) + "_feedback_age_ms",
        age_ms == 0xffffU ? "unknown" : std::to_string(age_ms));
    }
  }
  return status;
}

std::string format_preview(const UartLink::CanPreview & preview)
{
  std::ostringstream line;
  line << "CAN dry-run velocity_sequence=";
  if (preview.flags & 1U) {
    line << preview.velocity_sequence;
  } else {
    line << "none";
  }
  line << " rpm_over_limit=" << ((preview.flags & 2U) ? "true" : "false");

  for (std::size_t wheel = 0; wheel < preview.frames.size(); ++wheel) {
    const auto & frame = preview.frames[wheel];
    line << " | wheel_" << std::dec << wheel << " id=0x" << std::hex << std::setfill('0')
         << std::setw(3) << frame.id << " dlc=" << std::dec << unsigned(frame.dlc) << " data=";
    for (const auto byte : frame.data) {
      line << std::hex << std::setw(2) << unsigned(byte) << ' ';
    }
    const std::uint32_t raw_rpm = static_cast<std::uint32_t>(frame.data[2]) |
      (static_cast<std::uint32_t>(frame.data[3]) << 8) |
      (static_cast<std::uint32_t>(frame.data[4]) << 16) |
      (static_cast<std::uint32_t>(frame.data[5]) << 24);
    const auto signed_rpm = static_cast<std::int64_t>(raw_rpm) -
      ((raw_rpm & 0x80000000U) ? (std::int64_t{1} << 32) : 0);
    line << "target_motor_rpm=" << std::dec << signed_rpm;
  }
  return line.str();
}

long integer_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & key, long fallback)
{
  const auto value = info.hardware_parameters.find(key);
  if (value == info.hardware_parameters.end()) {
    return fallback;
  }
  std::size_t parsed = 0;
  const auto milliseconds = std::stol(value->second, &parsed);
  if (parsed != value->second.size()) {
    throw std::invalid_argument(key + " must be an integer");
  }
  return milliseconds;
}
}

class UartSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
    const auto & info = get_hardware_info();
    if (info.joints.size() != UartLink::wheel_count) {
      RCLCPP_ERROR(get_logger(), "UART requires exactly three wheel joints");
      return CallbackReturn::ERROR;
    }
    for (std::size_t index = 0; index < UartLink::wheel_count; ++index) {
      joint_names_[index] = "wheel_" + std::to_string(index) + "_joint";
      const auto joint = std::find_if(info.joints.begin(), info.joints.end(), [&](const auto & item) {
          return item.name == joint_names_[index];
        });
      if (joint == info.joints.end() || joint->command_interfaces.size() != 1 ||
        joint->command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY ||
        joint->state_interfaces.size() != 2 ||
        !std::any_of(joint->state_interfaces.begin(), joint->state_interfaces.end(),
          [](const auto & interface) {return interface.name == hardware_interface::HW_IF_POSITION;}) ||
        !std::any_of(joint->state_interfaces.begin(), joint->state_interfaces.end(),
          [](const auto & interface) {return interface.name == hardware_interface::HW_IF_VELOCITY;}))
      {
        RCLCPP_ERROR(
          get_logger(), "%s requires a velocity command and position/velocity states",
          joint_names_[index].c_str());
        return CallbackReturn::ERROR;
      }
    }

    const auto port = info.hardware_parameters.find("serial_port");
    if (port != info.hardware_parameters.end()) {
      port_ = port->second;
    }
    try {
      options_.command_period = std::chrono::milliseconds(
        integer_parameter(info, "command_period_ms", options_.command_period.count()));
      options_.response_timeout = std::chrono::milliseconds(
        integer_parameter(info, "response_timeout_ms", options_.response_timeout.count()));
      options_.can_preview_period = std::chrono::milliseconds(
        integer_parameter(info, "can_preview_period_ms", options_.can_preview_period.count()));
      const auto button = integer_parameter(info, "disarm_joy_button", 1);
      if (button < 0) {
        throw std::invalid_argument("disarm_joy_button must be nonnegative");
      }
      disarm_button_ = static_cast<std::size_t>(button);
      const auto limit = info.hardware_parameters.find("max_wheel_speed_rad_s");
      if (limit != info.hardware_parameters.end()) {
        std::size_t parsed = 0;
        options_.max_wheel_speed_rad_s = std::stod(limit->second, &parsed);
        if (parsed != limit->second.size()) {
          throw std::invalid_argument("max_wheel_speed_rad_s must be a number");
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "UART parameters: %s", error.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (std::size_t index = 0; index < UartLink::wheel_count; ++index) {
      interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_POSITION, &position_[index]);
      interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_VELOCITY, &velocity_[index]);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (std::size_t index = 0; index < UartLink::wheel_count; ++index) {
      interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_VELOCITY, &command_[index]);
    }
    return interfaces;
  }

  CallbackReturn on_configure(const LifecycleState &) override
  {
    auto node = get_node();
    if (!node) {
      RCLCPP_ERROR(get_logger(), "Cannot create the UART status publisher");
      return CallbackReturn::ERROR;
    }
    joy_subscription_ = node->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SensorDataQoS(),
      [request = drive_request_, button = disarm_button_](const sensor_msgs::msg::Joy & joy) {
        if (button < joy.buttons.size() && joy.buttons[button] == 1) {
          request->store(DriveRequest::disarm);
        }
      });
    disarm_service_ = node->create_service<std_srvs::srv::Trigger>(
      "/uart/disarm", [request = drive_request_](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        request->store(DriveRequest::disarm);
        response->success = true;
        response->message = "DISARM requested; confirm DISARMED on /uart/drive_status";
      });
    auto status_publisher = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/uart/drive_status", rclcpp::QoS(1));
    status_timer_.reset();
    status_ = std::make_shared<DriveStatusMailbox>();
    arm_service_ = node->create_service<std_srvs::srv::Trigger>(
      "/uart/arm", [request = drive_request_, mailbox = status_, timeout = options_.response_timeout](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        DriveStatus snapshot;
        {
          std::lock_guard<std::mutex> lock(mailbox->mutex);
          snapshot = mailbox->value;
        }
        const auto & state = snapshot.state;
        if (!snapshot.uart_open || !snapshot.error.empty() || !state.received_at ||
          UartLink::Clock::now() - *state.received_at > timeout)
        {
          response->message = "ARM requires a healthy, fresh UART connection";
          return;
        }
        if (request->load() == DriveRequest::drive && state.drive_state == 2) {
          response->success = true;
          response->message = "Already ARMED";
          return;
        }
        if (state.drive_state != 0 || state.stop_reason != 1 || state.fault != 0 ||
          !snapshot.commands_zero ||
          std::any_of(state.speed_mrad_s.begin(), state.speed_mrad_s.end(),
            [](auto speed) {return speed != 0;}))
        {
          response->message = "Release motion keys/sticks and wait for normal DISARMED at zero speed";
          return;
        }
        auto expected = DriveRequest::disarm;
        response->success = request->compare_exchange_strong(expected, DriveRequest::rearm);
        response->message = response->success ?
          "ARM requested; confirm ARMED on /uart/drive_status" : "ARM request is already pending";
      });
    status_timer_ = node->create_wall_timer(std::chrono::milliseconds(100),
      [mailbox = status_, status_publisher, clock = node->get_clock(), port = port_,
        timeout = options_.response_timeout, request = drive_request_]() {
        DriveStatus snapshot;
        {
          std::lock_guard<std::mutex> lock(mailbox->mutex);
          snapshot = mailbox->value;
        }
        diagnostic_msgs::msg::DiagnosticArray message;
        message.header.stamp = clock->now();
        message.status.push_back(format_drive_status(snapshot, port, timeout, request->load() != DriveRequest::drive));
        status_publisher->publish(message);
      });
    if (options_.can_preview_period.count() > 0) {
      auto publisher = node->create_publisher<std_msgs::msg::String>("/uart/can_preview", 10);
      mailbox_ = std::make_shared<PreviewMailbox>();
      preview_timer_ = node->create_wall_timer(std::chrono::milliseconds(50),
        [mailbox = mailbox_, publisher]() {
          UartLink::CanPreview preview;
          {
            std::lock_guard<std::mutex> lock(mailbox->mutex);
            if (!mailbox->available) {
              return;
            }
            preview = mailbox->preview;
            mailbox->available = false;
          }
          std_msgs::msg::String msg;
          msg.data = format_preview(preview);
          publisher->publish(msg);
        });
    }
    position_.fill(0.0);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const LifecycleState &) override
  {
    if (drive_request_->load() != DriveRequest::drive) {
      RCLCPP_ERROR(get_logger(), "DISARM is latched; use /uart/arm after a confirmed normal stop");
      return CallbackReturn::ERROR;
    }
    command_.fill(0.0);
    std::string error;
    if (!link_.start(port_, options_, error)) {
      record_drive_status(error);
      RCLCPP_ERROR(get_logger(), "Starting UART: %s", error.c_str());
      return CallbackReturn::ERROR;
    }
    velocity_ = link_.feedback_rad_s();
    record_drive_status({});
    active_ = true;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const LifecycleState &) override
  {
    stop_link();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const LifecycleState &) override
  {
    stop_link();
    clear_preview();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const LifecycleState &) override
  {
    stop_link();
    clear_preview();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const LifecycleState &) override
  {
    stop_link();
    clear_preview();
    return CallbackReturn::SUCCESS;
  }

  ReturnType read(const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    if (active_) {
      velocity_ = link_.feedback_rad_s();
      for (std::size_t index = 0; index < UartLink::wheel_count; ++index) {
        position_[index] += velocity_[index] * period.seconds();
      }
    }
    return ReturnType::OK;
  }

  ReturnType write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!active_) {
      return ReturnType::OK;
    }
    std::string error;
    bool updated;
    auto request = drive_request_->load();
    if (request == DriveRequest::rearm && !commands_zero()) {
      drive_request_->compare_exchange_strong(request, DriveRequest::disarm);
      request = DriveRequest::disarm;
      RCLCPP_WARN(get_logger(), "ARM cancelled because wheel targets are nonzero");
    }
    if (request == DriveRequest::disarm) {
      updated = link_.update_disarmed(error);
    } else if (request == DriveRequest::rearm) {
      updated = link_.update_rearm(error);
      if (updated && !link_.is_rearming() && link_.last_state().drive_state == 2) {
        auto expected = DriveRequest::rearm;
        drive_request_->compare_exchange_strong(expected, DriveRequest::drive);
      }
    } else {
      updated = link_.update(command_, error);
    }
    if (!updated) {
      active_ = false;
      velocity_.fill(0.0);
      record_drive_status(error);
      RCLCPP_ERROR(get_logger(), "UART update failed: %s", error.c_str());
      return ReturnType::ERROR;
    }

    if (status_) {
      std::unique_lock<std::mutex> lock(status_->mutex, std::try_to_lock);
      if (lock.owns_lock()) {
        status_->value.state = link_.last_state();
        status_->value.uart_open = link_.is_open();
        status_->value.commands_zero = commands_zero();
      }
    }
    if (mailbox_) {
      std::unique_lock<std::mutex> lock(mailbox_->mutex, std::try_to_lock);
      if (lock.owns_lock() && link_.take_can_preview(mailbox_->preview)) {
        mailbox_->available = true;
      }
    }
    return ReturnType::OK;
  }

private:
  bool commands_zero() const
  {
    return std::all_of(command_.begin(), command_.end(),
      [](double value) {return std::isfinite(value) && std::abs(value) < 1e-6;});
  }

  void record_drive_status(const std::string & error)
  {
    if (status_) {
      std::lock_guard<std::mutex> lock(status_->mutex);
      status_->value = {link_.last_state(), link_.is_open(), commands_zero(), error};
    }
  }

  void stop_link()
  {
    active_ = false;
    command_.fill(0.0);
    velocity_.fill(0.0);
    const bool uart_was_open = link_.is_open();
    link_.stop();
    if (uart_was_open) {
      record_drive_status({});
    }
    if (mailbox_) {
      std::lock_guard<std::mutex> lock(mailbox_->mutex);
      mailbox_->available = false;
    }
  }

  void clear_preview()
  {
    preview_timer_.reset();
    mailbox_.reset();
  }

  std::string port_{"/dev/ttyACM0"};
  UartLink link_;
  UartLink::Options options_;
  std::array<std::string, UartLink::wheel_count> joint_names_;
  UartLink::WheelSpeeds command_{};
  UartLink::WheelSpeeds position_{};
  UartLink::WheelSpeeds velocity_{};
  bool active_{false};
  std::shared_ptr<DriveStatusMailbox> status_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::size_t disarm_button_{1};
  std::shared_ptr<std::atomic<DriveRequest>> drive_request_{
    std::make_shared<std::atomic<DriveRequest>>(DriveRequest::drive)};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_service_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  std::shared_ptr<PreviewMailbox> mailbox_;
  rclcpp::TimerBase::SharedPtr preview_timer_;
};
}

PLUGINLIB_EXPORT_CLASS(robocup_uart_hardware::UartSystem, hardware_interface::SystemInterface)
