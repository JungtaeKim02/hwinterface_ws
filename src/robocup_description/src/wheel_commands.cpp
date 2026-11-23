#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace robocup_description
{
class WheelCommands final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    const gz::sim::Model model(entity);
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      joints_[i] = model.JointByName(ecm, kJointNames[i]);
      if (joints_[i] == gz::sim::kNullEntity) {
        RCLCPP_ERROR(rclcpp::get_logger("wheel_commands"), "Missing joint %s", kJointNames[i]);
        return;
      }
    }
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    auto options = rclcpp::NodeOptions().context(context_)
      .start_parameter_services(false).start_parameter_event_publisher(false).enable_rosout(false);
    node_ = std::make_shared<rclcpp::Node>("sim_wheel_commands", options);
    publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/sim/wheel_commands", rclcpp::QoS(1));
    message_.header.frame_id = "base_link";
    message_.name.assign(kJointNames.begin(), kJointNames.end());
    message_.velocity.resize(kJointNames.size());
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (!publisher_ || info.paused) {
      return;
    }
    if (info.simTime >= last_publish_ &&
      info.simTime - last_publish_ < std::chrono::milliseconds(20))
    {
      return;
    }
    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto * command = ecm.Component<gz::sim::components::JointVelocityCmd>(joints_[i]);
      if (command == nullptr || command->Data().size() != 1 ||
        !std::isfinite(command->Data().front()))
      {
        return;
      }
      message_.velocity[i] = command->Data().front();
    }
    const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
      info.simTime).count();
    message_.header.stamp = rclcpp::Time(nanoseconds, RCL_ROS_TIME);
    publisher_->publish(message_);
    last_publish_ = info.simTime;
  }

private:
  static constexpr std::array<const char *, 3> kJointNames{
    "wheel_0_joint", "wheel_1_joint", "wheel_2_joint"};
  std::array<gz::sim::Entity, kJointNames.size()> joints_{};
  std::chrono::steady_clock::duration last_publish_{};
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  sensor_msgs::msg::JointState message_;
};
}

GZ_ADD_PLUGIN(
  robocup_description::WheelCommands, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
