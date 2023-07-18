#pragma once

namespace civ {
namespace drivers {
namespace common {
class Stream {
 public:
  // Stream status.
  enum class Status {
    DISCONNECTED,
    CONNECTED,
    ERROR,
  };
  static constexpr size_t NUM_STATUS =
      static_cast<int>(Stream::Status::ERROR) + 1;

  Status get_status() const { return status_; }
  // Returns whether it was successful to connect.
  virtual bool Connect() = 0;

  // Returns whether it was successful to disconnect.
  virtual bool Disconnect() = 0;

  // Reads up to max_length bytes. Returns actually number of bytes read.
  virtual size_t read(uint8_t *buffer, size_t max_length) = 0;

  // Returns how many bytes it was successful to write.
  virtual size_t write(const uint8_t *buffer, size_t length) = 0;

 protected:
  Status status_ = Status::DISCONNECTED;
};
}  // namespace common
}  // namespace drivers
}  // namespace civ
