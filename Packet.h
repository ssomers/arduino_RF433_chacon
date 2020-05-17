class Packet {
  const unsigned long bits;

public:
  explicit Packet(unsigned long bits) : bits(bits) {}

  bool matches(unsigned long some_transmitter_and_button) const {
    if (multicast()) {
      return transmitter() == (some_transmitter_and_button >> 6);
    } else {
      return transmitter_and_button() == some_transmitter_and_button;
    }
  }

  unsigned long transmitter() const {
    return bits >> 6;
  }

  unsigned long transmitter_and_button() const {
    return bits & ~(1 << 5 | 1 << 4);
  }

  bool multicast() const {
    return (bits >> 5) & 1;
  }

  bool on_or_off() const {
    return (bits >> 4) & 1;
  }

  unsigned page() const {
    return (bits >> 2) & 3;
  }

  unsigned row() const {
    return bits & 3;
  }

  void print_transmitter_and_button() const {
    Serial.print(" transmitter ");
    Serial.print(transmitter(), HEX);
    if (multicast()) {
      Serial.print(" all ");
    } else {
      Serial.print(" button ");
      Serial.write('A' + page());
      Serial.print(row() + 1);
    }
  }
};
