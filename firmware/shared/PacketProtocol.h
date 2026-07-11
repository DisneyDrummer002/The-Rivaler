#pragma once

// Shared wire format for ESP-NOW and the wired UART links.
// This header deliberately has no Arduino dependency so all four boards can use it.

#include <stdint.h>
#include <string.h>

namespace rivaler {

constexpr uint8_t kPacketStartByte = 0xA5;
constexpr uint8_t kProtocolVersion = 1;
constexpr uint8_t kMaxPayloadBytes = 24;
constexpr uint8_t kPacketHeaderBytes = 5;
constexpr uint8_t kPacketChecksumBytes = 1;
constexpr uint8_t kMaxPacketBytes =
    kPacketHeaderBytes + kMaxPayloadBytes + kPacketChecksumBytes;

enum class MessageType : uint8_t {
  kRemoteControl = 0x10,
  kRobotStatus = 0x20,
  kCameraCommand = 0x30,
  kAcknowledgement = 0x40,
};

enum ControlButton : uint16_t {
  kArmSwitchHeld = 1 << 0,
  kFollowLineHeld = 1 << 1,
  kEmergencyStopHeld = 1 << 2,
};

enum ControlEvent : uint16_t {
  kQuickShootPressed = 1 << 0,
  kRapidFirePressed = 1 << 1,
  kTakePicturePressed = 1 << 2,
  kEjectSdPressed = 1 << 3,
  kFollowLinePressed = 1 << 4,
};

enum RobotStatusFlag : uint16_t {
  kLauncherArmed = 1 << 0,
  kLineDetected = 1 << 1,
  kLineFollowingActive = 1 << 2,
  kSafetyWarning = 1 << 3,
  kRemoteLinkTimedOut = 1 << 4,
  kCameraBusy = 1 << 5,
  kSdSafeToRemove = 1 << 6,
};

enum class SafetyFault : uint8_t {
  kNone = 0,
  kRemoteTimeout = 1,
  kWallTooClose = 2,
  kNoFloorDetected = 3,
  kEmergencyStop = 4,
  kInvalidCommand = 5,
};

enum class CameraOperation : uint8_t {
  kCaptureBurst = 1,
  kPrepareSdEject = 2,
};

#pragma pack(push, 1)

struct RemoteControlPayload {
  // -100 to 100 after remote-side dead-zone and scaling. Positive Y is forward.
  int8_t joystickX;
  int8_t joystickY;
  uint16_t heldButtons;
  uint16_t edgeEvents;
};

struct RobotStatusPayload {
  uint16_t statusFlags;
  uint16_t frontDistanceCm;
  uint16_t groundDistanceCm;
  SafetyFault safetyFault;
};

struct CameraCommandPayload {
  CameraOperation operation;
  uint8_t photoCount;
  uint16_t intervalMs;
};

struct AcknowledgementPayload {
  uint8_t acknowledgedSequence;
  uint8_t accepted;
  SafetyFault rejectionReason;
};

// Only the first packetWireSize(packet) bytes are sent. The remaining payload
// bytes are ignored and zeroed when a packet is initialized.
struct Packet {
  uint8_t startByte;
  uint8_t version;
  MessageType type;
  uint8_t sequence;
  uint8_t payloadLength;
  uint8_t payload[kMaxPayloadBytes];
  uint8_t checksum;
};

#pragma pack(pop)

static_assert(sizeof(RemoteControlPayload) == 6,
              "RemoteControlPayload layout changed");
static_assert(sizeof(RobotStatusPayload) == 7,
              "RobotStatusPayload layout changed");
static_assert(sizeof(CameraCommandPayload) == 4,
              "CameraCommandPayload layout changed");
static_assert(sizeof(AcknowledgementPayload) == 3,
              "AcknowledgementPayload layout changed");
static_assert(sizeof(Packet) == kMaxPacketBytes, "Packet layout changed");

inline uint8_t packetWireSize(const Packet& packet) {
  return kPacketHeaderBytes + packet.payloadLength + kPacketChecksumBytes;
}

inline uint8_t calculateChecksum(const Packet& packet) {
  uint8_t checksum = 0;
  checksum ^= packet.version;
  checksum ^= static_cast<uint8_t>(packet.type);
  checksum ^= packet.sequence;
  checksum ^= packet.payloadLength;

  for (uint8_t index = 0; index < packet.payloadLength; ++index) {
    checksum ^= packet.payload[index];
  }

  return checksum;
}

inline bool isPacketValid(const Packet& packet, uint8_t receivedBytes) {
  if (receivedBytes < kPacketHeaderBytes + kPacketChecksumBytes ||
      packet.startByte != kPacketStartByte ||
      packet.version != kProtocolVersion ||
      packet.payloadLength > kMaxPayloadBytes) {
    return false;
  }

  return receivedBytes == packetWireSize(packet) &&
         packet.checksum == calculateChecksum(packet);
}

inline void initializePacket(Packet& packet, MessageType type, uint8_t sequence) {
  memset(&packet, 0, sizeof(packet));
  packet.startByte = kPacketStartByte;
  packet.version = kProtocolVersion;
  packet.type = type;
  packet.sequence = sequence;
}

template <typename Payload>
inline bool setPayload(Packet& packet, const Payload& payload) {
  if (sizeof(Payload) > kMaxPayloadBytes) {
    return false;
  }

  memcpy(packet.payload, &payload, sizeof(Payload));
  packet.payloadLength = sizeof(Payload);
  packet.checksum = calculateChecksum(packet);
  return true;
}

template <typename Payload>
inline bool readPayload(const Packet& packet, Payload& payload) {
  if (packet.payloadLength != sizeof(Payload)) {
    return false;
  }

  memcpy(&payload, packet.payload, sizeof(Payload));
  return true;
}

class PacketParser {
 public:
  PacketParser() { reset(); }

  void reset() {
    receivedBytes_ = 0;
    expectedBytes_ = 0;
    memset(buffer_, 0, sizeof(buffer_));
  }

  // Feed bytes from a UART stream one at a time. Returns true only for a
  // complete valid packet, copied into completedPacket.
  bool pushByte(uint8_t incomingByte, Packet& completedPacket) {
    if (receivedBytes_ == 0) {
      if (incomingByte != kPacketStartByte) {
        return false;
      }

      buffer_[receivedBytes_++] = incomingByte;
      return false;
    }

    buffer_[receivedBytes_++] = incomingByte;

    if (receivedBytes_ == kPacketHeaderBytes) {
      const uint8_t payloadLength = buffer_[4];
      if (payloadLength > kMaxPayloadBytes) {
        reset();
        return false;
      }
      expectedBytes_ = kPacketHeaderBytes + payloadLength + kPacketChecksumBytes;
    }

    if (expectedBytes_ == 0 || receivedBytes_ < expectedBytes_) {
      return false;
    }

    Packet candidate{};
    memcpy(&candidate, buffer_, expectedBytes_);
    const bool isValid = isPacketValid(candidate, expectedBytes_);
    reset();

    if (!isValid) {
      return false;
    }

    completedPacket = candidate;
    return true;
  }

 private:
  uint8_t buffer_[kMaxPacketBytes];
  uint8_t receivedBytes_;
  uint8_t expectedBytes_;
};

}  // namespace rivaler
