#include "robocup_uart_hardware/uart_link.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <random>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace robocup_uart_hardware
{
namespace
{
constexpr std::uint8_t version = 2;
constexpr std::uint8_t frame_start = 0xff;
constexpr std::uint8_t frame_end = 0xfe;
constexpr std::uint8_t frame_escape = 0xfd;
constexpr std::uint8_t escape_xor = 0x20;
constexpr std::uint8_t hello = 0x01;
constexpr std::uint8_t arm = 0x02;
constexpr std::uint8_t set_velocity = 0x03;
constexpr std::uint8_t get_state = 0x04;
constexpr std::uint8_t disarm = 0x05;
constexpr std::uint8_t get_can_preview = 0x06;
constexpr std::uint8_t state_response = 0x81;
constexpr std::uint8_t can_preview_response = 0x82;
constexpr std::uint8_t disarmed = 0;
constexpr std::uint8_t arming = 1;
constexpr std::uint8_t simulated = 1U << 0;
constexpr std::uint8_t armed = 2;
constexpr std::uint8_t faulted = 4;
constexpr std::uint8_t last_command_valid = 1U << 1;
constexpr std::uint8_t all_wheels = 0b111;
constexpr std::size_t header_size = 9;
constexpr std::size_t state_size = 28;
constexpr std::size_t can_preview_size = 31;
constexpr std::size_t can_record_size = 9;
constexpr std::size_t velocity_size = 4 * UartLink::wheel_count;
constexpr std::size_t max_request_size = header_size + velocity_size + 2;
constexpr std::uint16_t unknown_feedback_age = 0xffff;
constexpr auto watchdog_period = std::chrono::milliseconds(200);
constexpr auto handshake_timeout = std::chrono::seconds(7);
constexpr auto can_arm_timeout = std::chrono::seconds(10);
constexpr double can_wheel_speed_limit_rad_s = 8.377;

bool reserved(std::uint8_t value)
{
  return value == frame_start || value == frame_end || value == frame_escape;
}

std::uint32_t read_u32(const std::uint8_t * data)
{
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8) |
         (static_cast<std::uint32_t>(data[2]) << 16) |
         (static_cast<std::uint32_t>(data[3]) << 24);
}

std::uint16_t read_u16(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>(data[0] | (data[1] << 8));
}

std::int32_t read_i32(const std::uint8_t * data)
{
  const auto value = read_u32(data);
  constexpr auto max_signed = static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max());
  if (value <= max_signed) {
    return static_cast<std::int32_t>(value);
  }
  return -1 - static_cast<std::int32_t>(std::numeric_limits<std::uint32_t>::max() - value);
}

void write_le(std::uint8_t * data, std::uint32_t value, std::size_t size)
{
  for (std::size_t index = 0; index < size; ++index) {
    data[index] = static_cast<std::uint8_t>(value >> (8 * index));
  }
}

std::uint16_t crc16(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xffff;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<std::uint16_t>(data[index] << 8);
    for (unsigned bit = 0; bit < 8; ++bit) {
      const bool top_bit = (crc & 0x8000U) != 0;
      crc = static_cast<std::uint16_t>(crc << 1);
      if (top_bit) {
        crc ^= 0x1021U;
      }
    }
  }
  return crc;
}
}

UartLink::~UartLink()
{
  stop();
}

bool UartLink::start(const std::string & port, const Options & options, std::string & error)
{
  stop();
  state_ = {};
  disarm_requested_ = false;
  if (options.command_period.count() <= 0 || options.response_timeout.count() <= 0 ||
    options.can_preview_period.count() < 0 ||
    options.command_period >= watchdog_period ||
    options.response_timeout >= watchdog_period ||
    !std::isfinite(options.max_wheel_speed_rad_s) ||
    options.max_wheel_speed_rad_s <= 0.0 || options.max_wheel_speed_rad_s > 20.0)
  {
    error = "Invalid UART timing or wheel speed limit";
    return false;
  }
  options_ = options;
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ == -1) {
    error = "Opening " + port + ": " + std::strerror(errno);
    return false;
  }
  termios settings{};
  if (::ioctl(fd_, TIOCEXCL) == -1 || ::tcgetattr(fd_, &settings) == -1) {
    error = "Accessing serial port " + port + ": " + std::strerror(errno);
    close_port();
    return false;
  }
  ::cfmakeraw(&settings);
  settings.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
  settings.c_cflag |= CS8 | CLOCAL | CREAD;
  settings.c_iflag &= ~(IXON | IXOFF | IXANY);
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (::cfsetispeed(&settings, B115200) == -1 ||
    ::cfsetospeed(&settings, B115200) == -1 ||
    ::tcsetattr(fd_, TCSANOW, &settings) == -1 || ::tcflush(fd_, TCIOFLUSH) == -1)
  {
    error = "Configuring " + port + ": " + std::strerror(errno);
    close_port();
    return false;
  }

  try {
    std::random_device random;
    session_ = static_cast<std::uint32_t>(random());
  } catch (const std::exception & exception) {
    error = "Choosing UART session: " + std::string(exception.what());
    close_port();
    return false;
  }
  if (session_ == 0) {
    session_ = 1;
  }
  sequence_ = 0;
  reset_receiver();
  feedback_rad_s_.fill(0.0);


  bool connected = false;
  for (unsigned attempt = 0; attempt < 3 && !connected; ++attempt) {
    connected = begin(hello, {}, error) && wait_for_state(error);
  }
  if (!connected) {
    close_port();
    return false;
  }

  const auto stop_deadline = Clock::now() + handshake_timeout;
  while (state_.result == 0 && state_.drive_state != disarmed &&
    state_.drive_state != faulted && Clock::now() < stop_deadline)
  {
    pollfd input{fd_, POLLIN, 0};
    (void)::poll(&input, 1, static_cast<int>(options_.command_period.count()));
    if (!begin(get_state, {}, error) || !wait_for_state(error)) {
      close_port();
      return false;
    }
  }
  const bool is_simulation = (state_.flags & simulated) != 0;
  if (state_.result != 0 || state_.drive_state != disarmed || state_.fault != 0 ||
    (is_simulation && (state_.online != all_wheels || state_.feedback_valid != all_wheels)))
  {
    error = "MCU is not ready to arm (state " + std::to_string(state_.drive_state) +
      ", stop reason " + std::to_string(state_.stop_reason) + ")";
    close_port();
    return false;
  }
  if (!is_simulation) {
    options_.max_wheel_speed_rad_s =
      std::min(options_.max_wheel_speed_rad_s, can_wheel_speed_limit_rad_s);
  }
  if (!begin(arm, {}, error) || !wait_for_state(error)) {
    stop_after_error();
    return false;
  }


  const auto arm_deadline = Clock::now() + can_arm_timeout;
  while (state_.result == 0 && state_.drive_state == arming && Clock::now() < arm_deadline) {
    pollfd input{fd_, POLLIN, 0};
    (void)::poll(&input, 1, static_cast<int>(options_.command_period.count()));
    if (!begin(get_state, {}, error) || !wait_for_state(error)) {
      stop_after_error();
      return false;
    }
  }
  if (state_.result != 0 || state_.drive_state != armed) {
    error = "MCU could not arm (result " + std::to_string(state_.result) + ", state " +
      std::to_string(state_.drive_state) + ", stop reason " +
      std::to_string(state_.stop_reason) + ", drive fault mask " +
      std::to_string(state_.fault) + ")";
    stop_after_error();
    return false;
  }
  if (!begin(set_velocity, {}, error) || !wait_for_state(error) || !accept_velocity_state(error)) {
    stop_after_error();
    return false;
  }
  next_command_ = Clock::now() + options_.command_period;
  next_preview_ = Clock::now() + options_.can_preview_period;
  return true;
}

bool UartLink::begin(std::uint8_t request, const WheelSpeeds & wheel_command, std::string & error)
{
  ++sequence_;
  std::array<std::uint8_t, max_request_size> raw{};
  raw[0] = version;
  raw[1] = request;
  write_le(&raw[2], session_, 4);
  write_le(&raw[6], sequence_, 2);
  if (request == set_velocity) {
    raw[8] = velocity_size;
    for (std::size_t wheel = 0; wheel < wheel_count; ++wheel) {
      if (!std::isfinite(wheel_command[wheel])) {
        error = "A wheel command is not finite";
        return false;
      }
      const double bounded = std::clamp(
        wheel_command[wheel], -options_.max_wheel_speed_rad_s, options_.max_wheel_speed_rad_s);
      const auto mrad_s = static_cast<std::int32_t>(std::lround(bounded * 1000.0));
      write_le(&raw[header_size + 4 * wheel], static_cast<std::uint32_t>(mrad_s), 4);
    }
  }

  const std::size_t body_size = header_size + raw[8];
  write_le(&raw[body_size], crc16(raw.data(), body_size), 2);
  std::array<std::uint8_t, 2 * max_request_size + 2> wire{};
  std::size_t wire_size = 0;
  wire[wire_size++] = frame_start;
  for (std::size_t index = 0; index < body_size + 2; ++index) {
    const auto byte = raw[index];
    if (reserved(byte)) {
      wire[wire_size++] = frame_escape;
      wire[wire_size++] = byte ^ escape_xor;
    } else {
      wire[wire_size++] = byte;
    }
  }
  wire[wire_size++] = frame_end;

  const auto written = ::write(fd_, wire.data(), wire_size);
  if (written != static_cast<ssize_t>(wire_size)) {
    error = written < 0 ? "UART write: " + std::string(std::strerror(errno)) : "Short UART write";
    return false;
  }
  if (request == get_can_preview) {
    preview_sequence_ = sequence_;
    preview_requested_ = true;
  } else {
    state_sequence_ = sequence_;
    pending_ = true;
    deadline_ = Clock::now() + options_.response_timeout;
  }
  return true;
}

bool UartLink::read_available(std::string & error)
{

  for (unsigned iteration = 0; iteration < 4; ++iteration) {
    std::array<std::uint8_t, 128> buffer{};
    const auto count = ::read(fd_, buffer.data(), buffer.size());
    if (count == 0 || (count < 0 && (errno == EAGAIN || errno == EINTR))) {
      return true;
    }
    if (count < 0) {
      error = "UART read: " + std::string(std::strerror(errno));
      return false;
    }
    for (ssize_t index = 0; index < count; ++index) {
      receive_byte(buffer[static_cast<std::size_t>(index)]);
    }
  }
  return true;
}

void UartLink::receive_byte(std::uint8_t byte)
{
  if (byte == frame_start) {
    reset_receiver();
    inside_frame_ = true;
    return;
  }
  if (!inside_frame_) {
    return;
  }
  if (byte == frame_end) {
    if (!escape_pending_) {
      consume_frame();
    }
    reset_receiver();
    return;
  }
  if (escape_pending_) {
    escape_pending_ = false;
    byte ^= escape_xor;
    if (!reserved(byte)) {
      reset_receiver();
      return;
    }
  } else if (byte == frame_escape) {
    escape_pending_ = true;
    return;
  }

  if (raw_size_ == raw_.size()) {
    reset_receiver();
    return;
  }
  raw_[raw_size_++] = byte;
}

void UartLink::consume_frame()
{
  if (raw_size_ < header_size + 2 || raw_size_ != header_size + raw_[8] + 2 ||
    raw_[0] != version || read_u32(&raw_[2]) != session_ ||
    read_u16(&raw_[raw_size_ - 2]) != crc16(raw_.data(), raw_size_ - 2))
  {
    return;
  }
  const auto * payload = &raw_[header_size];
  const auto response_sequence = read_u16(&raw_[6]);
  if (preview_requested_ && raw_[1] == can_preview_response && raw_[8] == can_preview_size &&
    response_sequence == preview_sequence_ && payload[0] == 0)
  {
    CanPreview preview{};
    preview.flags = payload[1];
    preview.velocity_sequence = read_u16(&payload[2]);
    for (std::size_t wheel = 0; wheel < wheel_count; ++wheel) {
      auto & frame = preview.frames[wheel];
      const auto * record = &payload[4 + wheel * can_record_size];
      frame.id = read_u16(record);
      frame.dlc = record[2];
      if (frame.id > 0x7ff || frame.dlc != frame.data.size()) {
        return;
      }
      std::copy_n(record + 3, frame.data.size(), frame.data.begin());
    }
    preview_ = preview;
    preview_available_ = true;
    preview_requested_ = false;
    return;
  }
  if (!pending_ || raw_[1] != state_response || raw_[8] != state_size ||
    response_sequence != state_sequence_)
  {
    return;
  }
  state_.result = payload[0];
  state_.drive_state = payload[1];
  state_.stop_reason = payload[2];
  state_.flags = payload[3];
  state_.online = payload[4];
  state_.enabled = payload[5];
  state_.feedback_valid = payload[6];
  state_.fault = payload[7];
  state_.last_velocity_sequence = read_u16(&payload[8]);
  for (std::size_t wheel = 0; wheel < wheel_count; ++wheel) {
    state_.speed_mrad_s[wheel] = read_i32(&payload[10 + 4 * wheel]);
    state_.feedback_age_ms[wheel] = read_u16(&payload[22 + 2 * wheel]);
  }
  state_.received_at = Clock::now();
  pending_ = false;
}

bool UartLink::wait_for_state(std::string & error)
{
  while (pending_ && Clock::now() < deadline_) {
    const auto wait = std::chrono::ceil<std::chrono::milliseconds>(deadline_ - Clock::now());
    pollfd input{fd_, POLLIN, 0};
    const int ready = ::poll(&input, 1, static_cast<int>(std::max(0L, wait.count())));
    if (ready < 0 && errno == EINTR) {
      continue;
    }
    if (ready < 0 || (input.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      error = "UART disconnected while waiting for STATE";
      return false;
    }
    if (ready > 0 && !read_available(error)) {
      return false;
    }
  }
  if (pending_) {
    error = "Timed out waiting for UART STATE";
    return false;
  }
  return true;
}

bool UartLink::accept_velocity_state(std::string & error)
{
  if (state_.result != 0 || state_.drive_state != armed ||
    !(state_.flags & last_command_valid) || state_.last_velocity_sequence != state_sequence_ ||
    state_.enabled != all_wheels || state_.online != all_wheels ||
    state_.feedback_valid != all_wheels || state_.fault != 0)
  {
    error = "MCU rejected velocity or a wheel is unavailable (stop reason " +
      std::to_string(state_.stop_reason) + ", drive fault mask " + std::to_string(state_.fault) + ")";
    return false;
  }
  for (std::size_t wheel = 0; wheel < wheel_count; ++wheel) {
    if (state_.feedback_age_ms[wheel] == unknown_feedback_age ||
      state_.feedback_age_ms[wheel] > options_.response_timeout.count())
    {
      error = "MCU reported stale wheel feedback";
      return false;
    }
    feedback_rad_s_[wheel] = state_.speed_mrad_s[wheel] * 0.001;
  }
  return true;
}

bool UartLink::update(const WheelSpeeds & wheel_command, std::string & error)
{
  if (fd_ == -1) {
    error = "UART is not open";
    return false;
  }
  const bool was_pending = pending_;
  if (!read_available(error) || (was_pending && !pending_ && !accept_velocity_state(error))) {
    stop_after_error();
    return false;
  }
  const auto now = Clock::now();
  if (pending_) {
    if (now >= deadline_) {
      error = "Timed out waiting for a velocity STATE";
      stop_after_error();
      return false;
    }
  } else if (now >= next_command_) {
    if (!begin(set_velocity, wheel_command, error)) {
      stop_after_error();
      return false;
    }
    next_command_ += options_.command_period;
    if (next_command_ <= now) {
      next_command_ = now + options_.command_period;
    }
  }



  if (options_.can_preview_period.count() > 0 && now >= next_preview_) {
    if (!begin(get_can_preview, {}, error)) {
      stop_after_error();
      return false;
    }
    next_preview_ = now + options_.can_preview_period;
  }
  return true;
}

bool UartLink::take_can_preview(CanPreview & preview)
{
  if (!preview_available_) {
    return false;
  }
  preview = preview_;
  preview_available_ = false;
  return true;
}

bool UartLink::update_disarmed(std::string & error)
{
  if (fd_ == -1) {
    error = "UART is not open";
    return false;
  }
  const auto now = Clock::now();
  if (is_rearming()) {
    rearm_phase_ = RearmPhase::idle;
    disarm_requested_ = false;
  }
  if (!disarm_requested_) {

    if (!begin(disarm, {}, error)) {
      stop_after_error();
      return false;
    }
    disarm_requested_ = true;
    next_command_ = now + options_.command_period;
    return true;
  }
  if (!read_available(error)) {
    stop_after_error();
    return false;
  }
  if (pending_) {
    if (now < deadline_) {
      return true;
    }
    error = "Timed out waiting for MCU state after DISARM";
    stop_after_error();
    return false;
  }
  if (state_.result != 0) {
    error = "MCU rejected request during DISARM (result " +
      std::to_string(state_.result) + ")";
    stop_after_error();
    return false;
  }
  if (now >= next_command_) {
    if (!begin(get_state, {}, error)) {
      stop_after_error();
      return false;
    }
    next_command_ = now + options_.command_period;
  }
  return true;
}

bool UartLink::update_rearm(std::string & error)
{
  if (fd_ == -1) {
    error = "UART is not open";
    return false;
  }
  if (!read_available(error)) {
    stop_after_error();
    return false;
  }
  const auto now = Clock::now();
  if (pending_) {
    if (now < deadline_) {
      return true;
    }
    error = "Timed out waiting for MCU state during ARM";
    stop_after_error();
    return false;
  }
  if (!is_rearming()) {
    if (!disarm_requested_ || state_.result != 0 || state_.drive_state != disarmed ||
      state_.stop_reason != 1 || state_.fault != 0)
    {
      error = "ARM requires a confirmed normal DISARM";
      stop_after_error();
      return false;
    }
    if (!begin(arm, {}, error)) {
      stop_after_error();
      return false;
    }
    rearm_phase_ = RearmPhase::waiting_for_arm;
    rearm_deadline_ = now + can_arm_timeout;
    next_command_ = now + options_.command_period;
    return true;
  }
  if (state_.result != 0 || state_.fault != 0 || now >= rearm_deadline_) {
    error = "MCU could not re-arm (result " + std::to_string(state_.result) +
      ", state " + std::to_string(state_.drive_state) +
      ", stop reason " + std::to_string(state_.stop_reason) + ")";
    stop_after_error();
    return false;
  }
  if (rearm_phase_ == RearmPhase::waiting_for_zero) {
    if (!accept_velocity_state(error)) {
      stop_after_error();
      return false;
    }
    rearm_phase_ = RearmPhase::idle;
    disarm_requested_ = false;
    next_command_ = now + options_.command_period;
    next_preview_ = now + options_.can_preview_period;
    return true;
  }
  if (state_.drive_state == armed) {
    if (!begin(set_velocity, {}, error)) {
      stop_after_error();
      return false;
    }
    rearm_phase_ = RearmPhase::waiting_for_zero;
    return true;
  }
  if (state_.drive_state != arming) {
    error = "Unexpected MCU state during ARM: " + std::to_string(state_.drive_state);
    stop_after_error();
    return false;
  }
  if (now >= next_command_) {
    if (!begin(get_state, {}, error)) {
      stop_after_error();
      return false;
    }
    next_command_ = now + options_.command_period;
  }
  return true;
}

void UartLink::reset_receiver()
{
  inside_frame_ = false;
  escape_pending_ = false;
  raw_size_ = 0;
}

void UartLink::close_port()
{
  rearm_phase_ = RearmPhase::idle;
  if (fd_ != -1) {
    (void)::ioctl(fd_, TIOCNXCL);
    (void)::close(fd_);
    fd_ = -1;
  }
  pending_ = false;
  preview_requested_ = false;
  preview_available_ = false;
  reset_receiver();
}

void UartLink::stop_after_error()
{

  std::string ignored;
  (void)begin(disarm, {}, ignored);
  close_port();
}

void UartLink::stop()
{
  if (fd_ == -1) {
    return;
  }
  std::string ignored;
  if (begin(disarm, {}, ignored)) {
    (void)wait_for_state(ignored);
  }
  close_port();
}

}
