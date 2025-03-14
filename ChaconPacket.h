class ChaconPacket {
  const uint32_t bits;

public:
  explicit ChaconPacket(uint32_t bits)
    : bits(bits) {}

  bool matches(uint32_t some_transmitter_and_button) const {
    if (multicast()) {
      return transmitter() == (some_transmitter_and_button >> 6);
    } else {
      return transmitter_and_button() == some_transmitter_and_button;
    }
  }

  uint32_t transmitter() const {
    return bits >> 6;
  }

  uint32_t transmitter_and_button() const {
    return bits & ~(1 << 5 | 1 << 4);
  }

  bool multicast() const {
    return (bits >> 5) & 1;
  }

  bool on_or_off() const {
    return (bits >> 4) & 1;
  }

  uint8_t page() const {
    return (bits >> 2) & 3;
  }

  uint8_t row() const {
    return bits & 3;
  }
};
