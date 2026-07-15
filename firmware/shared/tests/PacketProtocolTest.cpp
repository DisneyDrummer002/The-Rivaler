#include <assert.h>

#include "../PacketProtocol.h"

int main() {
  for (uint16_t sequence = 0; sequence <= UINT8_MAX; ++sequence) {
    rivaler::RemoteControlPayload control{};
    control.joystickX = -25;
    control.joystickY = 75;
    control.heldButtons = rivaler::kArmSwitchHeld;
    control.edgeEvents = rivaler::kQuickShootPressed;

    rivaler::Packet packet{};
    rivaler::initializePacket(packet, rivaler::MessageType::kRemoteControl,
                              static_cast<uint8_t>(sequence));
    assert(rivaler::setPayload(packet, control));
    assert(rivaler::packetWireSize(packet) == 12);

    uint8_t wireBytes[rivaler::kMaxPacketBytes]{};
    assert(rivaler::encodePacket(packet, wireBytes, sizeof(wireBytes)));
    assert(wireBytes[11] == packet.checksum);

    rivaler::Packet decoded{};
    assert(rivaler::decodePacket(wireBytes, rivaler::packetWireSize(packet),
                                 decoded));
    rivaler::RemoteControlPayload decodedControl{};
    assert(rivaler::readPayload(decoded, decodedControl));
    assert(decodedControl.joystickX == control.joystickX);
    assert(decodedControl.joystickY == control.joystickY);
    assert(decodedControl.heldButtons == control.heldButtons);
    assert(decodedControl.edgeEvents == control.edgeEvents);

    rivaler::PacketParser parser;
    rivaler::Packet parsed{};
    bool completed = false;
    for (uint8_t index = 0; index < rivaler::packetWireSize(packet); ++index) {
      completed = parser.pushByte(wireBytes[index], parsed) || completed;
    }
    assert(completed);
    assert(parsed.sequence == packet.sequence);
  }

  return 0;
}
