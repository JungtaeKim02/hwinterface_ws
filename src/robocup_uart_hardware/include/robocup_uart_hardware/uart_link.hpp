#ifndef ROBOCUP_UART_HARDWARE__UART_LINK_HPP_
#define ROBOCUP_UART_HARDWARE__UART_LINK_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace robocup_uart_hardware
{

class UartLink
{
public:
  using Clock = std::chrono::steady_clock;
  static constexpr std::size_t wheel_count = 3;
  using WheelSpeeds = std::array<double, wheel_count>;

  struct Options
  {
    std::chrono::milliseconds command_period{20};
    std::chrono::milliseconds response_timeout{100};
    std::chrono::milliseconds can_preview_period{0};
    double max_wheel_speed_rad_s{20.0};
  };

  struct CanFrame
  {
    std::uint16_t id{};
    std::uint8_t dlc{};
    std::array<std::uint8_t, 6> data{};
  };

  struct CanPreview
  {
    std::uint8_t flags{};
    std::uint16_t velocity_sequence{};
    std::array<CanFrame, wheel_count> frames{};
  };

  struct State
  {
    std::optional<Clock::time_point> received_at;
    std::uint8_t result{};
    std::uint8_t drive_state{};
    std::uint8_t stop_reason{};
    std::uint8_t flags{};
    std::uint8_t online{};
    std::uint8_t enabled{};
    std::uint8_t feedback_valid{};
    std::uint8_t fault{};
    std::uint16_t last_velocity_sequence{};
    std::array<std::int32_t, wheel_count> speed_mrad_s{};
    std::array<std::uint16_t, wheel_count> feedback_age_ms{};
  };

  UartLink() = default;
  ~UartLink();
  UartLink(const UartLink &) = delete;
  UartLink & operator=(const UartLink &) = delete;

  bool start(const std::string & port, const Options & options, std::string & error);

  bool update(const WheelSpeeds & wheel_command, std::string & error);

  bool update_disarmed(std::string & error);

  bool update_rearm(std::string & error);
  bool is_rearming() const {return rearm_phase_ != RearmPhase::idle;}

  void stop();

  const WheelSpeeds & feedback_rad_s() const {return feedback_rad_s_;}
  const State & last_state() const {return state_;}
  bool is_open() const {return fd_ != -1;}
  bool take_can_preview(CanPreview & preview);

private:
  enum class RearmPhase {idle, waiting_for_arm, waiting_for_zero};
  RearmPhase rearm_phase_{RearmPhase::idle};
  Clock::time_point rearm_deadline_{};

  static constexpr std::size_t raw_capacity = 64;

  bool begin(std::uint8_t request, const WheelSpeeds & wheel_command, std::string & error);
  bool wait_for_state(std::string & error);
  bool read_available(std::string & error);
  void receive_byte(std::uint8_t byte);
  void consume_frame();
  void reset_receiver();
  void close_port();
  void stop_after_error();
  bool accept_velocity_state(std::string & error);

  Options options_;
  int fd_{-1};
  std::uint32_t session_{};
  std::uint16_t sequence_{};
  std::uint16_t state_sequence_{};
  std::uint16_t preview_sequence_{};
  bool pending_{false};
  bool disarm_requested_{false};
  bool preview_requested_{false};
  bool preview_available_{false};
  Clock::time_point deadline_{};
  Clock::time_point next_command_{};
  Clock::time_point next_preview_{};
  State state_{};
  WheelSpeeds feedback_rad_s_{};
  CanPreview preview_{};

  bool inside_frame_{false};
  bool escape_pending_{false};
  std::array<std::uint8_t, raw_capacity> raw_{};
  std::size_t raw_size_{};
};

}

#endif
