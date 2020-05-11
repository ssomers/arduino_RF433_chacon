static const int PIN_DIGITAL_IN = 2;
static const int MIN_PEAK_SPACING = 500;
static const int MAX_PEAK_SPACING = 600;
static const int MAX_VALE_SPACING = 1500;
static const int MIN_PACKET_SPACING = 10000;
static const int MIN_PACKET_PREAMBLE = 2800;
static const bool VERBOSE = false;

class ProtocolHandler {
  unsigned long last_time = micros();
  unsigned long packet_handled = 0;
  enum { IDLE = -2, DELIMITED = -1, OPENED = 0 };
  int reception_stage;
  int extra_peaks_received;
  unsigned long delimiter_micros;
  unsigned long preamble_micros;
  unsigned long bits_received;

  void abort_packet_train() {
    digitalWrite(LED_BUILTIN, LOW);
    packet_handled = 0;
    reception_stage = IDLE;
  }

  void cancel_packet() {
    reception_stage = IDLE;
  }

  void packet_delimited() {
    if (VERBOSE && reception_stage > OPENED) {
      if (reception_stage != 32) {
        Serial.print("Invalid vale count ");
        Serial.println(reception_stage);
      } else if (extra_peaks_received != int(bits_received & 1)) {
        Serial.print("Invalid final peak count ");
        Serial.println(1 + extra_peaks_received);
      } else if (packet_handled == 0) {
        Serial.println("Packet received but missed by main loop");
      } else if (packet_handled != bits_received) {
        Serial.print(packet_handled, BIN);
        Serial.println(" received, but then");
        Serial.print(bits_received, BIN);
        Serial.println(" received!");
      }
    }
  }

public:
  void handleRise() {
    const unsigned long time = micros();
    const unsigned long duration = time > last_time ? time - last_time : 0;
    last_time = time;

    if (duration >= MIN_PACKET_SPACING) {
      packet_delimited();
      delimiter_micros = duration;
      reception_stage = DELIMITED;
    } else if (reception_stage == DELIMITED) {
      if (duration < MIN_PACKET_PREAMBLE) {
        abort_packet_train(); // Not a repeat of the previous packet
        return;
      }
      preamble_micros = duration;
      reception_stage = OPENED;
      extra_peaks_received = 0;
      bits_received = 0;
    } else if (reception_stage >= OPENED) {
      if (duration < MIN_PEAK_SPACING) {
        if (VERBOSE) {
          Serial.print(duration);
          Serial.print(" µs peak seen in vale ");
          Serial.println(reception_stage);
        }
        cancel_packet();
        return;
      }
      if (duration <= MAX_PEAK_SPACING) {
        ++extra_peaks_received;
      } else {
        if (duration <= MAX_VALE_SPACING) {
          if (VERBOSE) {
            Serial.print(duration);
            Serial.print(" µs vale seen in vale ");
            Serial.println(reception_stage);
          }
          cancel_packet();
          return;
        }
        const int bit = 1 + int(bits_received & 1) - extra_peaks_received;
        if (bit < 0 || bit > 1) {
          if (VERBOSE) {
            Serial.print("Invalid peak count ");
            Serial.println(1 + extra_peaks_received);
          }
          cancel_packet();
          return;
        }
        bits_received = (bits_received << 1) | bit;
        extra_peaks_received = 0;
        if (reception_stage == OPENED) {
          digitalWrite(LED_BUILTIN, HIGH);
        }
        ++reception_stage;
      }
    }
  }

  unsigned long receive() {
    noInterrupts();
    const unsigned long bits = bits_received;
    const unsigned long bits_time = last_time;
    const bool has_new = packet_handled != bits
                        && reception_stage == 32
                        && extra_peaks_received == int(bits & 1);
    interrupts();
    if (has_new && micros() > bits_time + MAX_PEAK_SPACING) {
      packet_handled = bits;
      if (VERBOSE) {
        Serial.print("After ");
        Serial.print(delimiter_micros);
        Serial.print(" + ");
        Serial.print(preamble_micros);
        Serial.println(" µs:");
      }
      return bits;
    } else {
      return 0;
    }
  }
};

class Packet {
  const unsigned long bits;

public:
  explicit Packet(unsigned long bits): bits(bits) {}

  unsigned long transmitter() const {
    return bits >> 6;
  }

  bool multicast() const {
    return (bits >> 5) & 1;
  }

  unsigned page() const {
    return (bits >> 2) & 3;
  }

  unsigned row() const {
    return bits & 3;
  }

  bool on_or_off() const {
    return (bits >> 4) & 1;
  }
};

static ProtocolHandler handler;

static void handleRise() {
  handler.handleRise();
}

void setup() {
  pinMode(PIN_DIGITAL_IN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_IN), handleRise, RISING);
  Serial.begin(115200);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only
}

void loop() {
  if (auto bits = handler.receive()) {
    Serial.print("Received ");
    Serial.print(bits, BIN);
    Packet packet(bits);
    Serial.print(": transmitter ");
    Serial.print(packet.transmitter(), HEX);
    if (packet.multicast()) {
      Serial.print(" all ");
    } else {
      Serial.print(" button ");
      Serial.write('A' + packet.page());
      Serial.print(packet.row() + 1);
    }
    Serial.println(packet.on_or_off() ? "①" : "⓪");
  }
}
