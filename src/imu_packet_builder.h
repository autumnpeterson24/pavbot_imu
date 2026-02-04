// Don't forget to guard this

#include <vector>
#include <cstdint>

enum class ImuCommand : uint8_t {
    // Bonus
    StartByte = 0xF9,
    ResponseMask = 0x80,

    // Getters 
    GetEulerAngles = 0x01,

    // Setters
    SetCompassEnabled = 0x26,

};

class ImuPacketBuilder {
  public:
    static std::vector<uint8_t> build(ImuCommand c, const std::vector<uint8_t>& payload = {}) {
        std::vector<uint8_t> packet;

        packet.push_back(ImuCommand::StartByte);
        packet.push_back(static_cast<uint8_t>(c));

        packet.push_back(static_cast<uint8_t>(payload.size()));
        packet.insert(packet.end(), payload.begin(), payload.end());

        packet.push_back(checksum(packet));

        return packet;
    }

  private:
    static uint8_t checksum(const std::vector<uint8_t>& packet) {
        uint8_t sum = 0;

        for (size_t i = 1; i < packet.size(); ++i) {
            sum += packet[i];
        }

        return sum;
    }
};