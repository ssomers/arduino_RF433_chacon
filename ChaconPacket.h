class ChaconButtonPairId {
  uint32_t bits;
  explicit ChaconButtonPairId(uint32_t bits)
    : bits(bits) {}

public:
  ChaconButtonPairId()
    : bits(~0u) {}
  ChaconButtonPairId(const ChaconButtonPairId&) = default;
  ChaconButtonPairId(ChaconButtonPairId&&) = default;
  ChaconButtonPairId& operator=(const ChaconButtonPairId&) = default;
  ChaconButtonPairId& operator=(ChaconButtonPairId&&) = delete;

  static ChaconButtonPairId extractFromPacket(uint32_t packet_bits) {
    return ChaconButtonPairId(packet_bits & ~0b110000);
  }

  bool valid() const {
    return (bits & 0b110000) == 0;
  }

  uint32_t transmitter() const {
    return bits & ~0b1111;
  }

  uint8_t page() const {
    return (bits >> 2) & 0b11;
  }

  uint8_t row() const {
    return bits & 0b11;
  }

  bool operator==(ChaconButtonPairId rhs) const {
    return bits == rhs.bits;
  }

  void invalidate() {
    bits = ~0u;
  }

  ChaconButtonPairId& load_bits(uint32_t bits) {
    this->bits = bits;
    return *this;
  }

  uint32_t extract_bits() const {
    return bits;
  }
};

class ChaconPacket {
  const uint32_t bits;

public:
  explicit ChaconPacket(uint32_t bits)
    : bits(bits) {}

  bool matches(ChaconButtonPairId some_button_pair) const {
    if (multicast()) {
      return transmitter() == some_button_pair.transmitter();
    } else {
      return button_pair() == some_button_pair;
    }
  }

  ChaconButtonPairId button_pair() const {
    return ChaconButtonPairId::extractFromPacket(bits);
  }

  uint32_t transmitter() const {
    return button_pair().transmitter();
  }

  bool multicast() const {
    return (bits >> 5) & 1;
  }

  bool on_or_off() const {
    return (bits >> 4) & 1;
  }
};
