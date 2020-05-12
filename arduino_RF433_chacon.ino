static const int PIN_DIGITAL_IN = 2;
static const bool VERBOSE = false;
#define DIAGNOSTICS 0

static const unsigned long MIN_PEAK_SPACING = 500;
static const unsigned long MAX_PEAK_SPACING = 600;
static const unsigned long MAX_VALE_SPACING = 1500;
static const unsigned long MIN_PACKET_SPACING = 10000;
static const unsigned long MAX_PACKET_SPACING = 11000;
static const unsigned long MIN_PACKET_PREAMBLE = 2800;
static const unsigned long MAX_PACKET_PREAMBLE = 3200;
static const unsigned long MAX_PAYLOAD_TIMEOUT = 32ul * (MAX_VALE_SPACING + 3 * MAX_PEAK_SPACING);

class ProtocolHandler {
  unsigned long last_rise_micros = micros();
  unsigned long packet_handled = 0;
  unsigned long packet_timeout_micros;
  enum { IDLE = -3, BOTCHED_PACKET = -2, DELIMITED = -1, OPENED = 0 };
  int reception_stage;
  int extra_peaks_received;
  unsigned long bits_received;
#if DIAGNOSTICS
  unsigned long delimiter_micros;
  unsigned long preamble_micros;
#endif

  void abort_packet_train() {
    digitalWrite(LED_BUILTIN, LOW);
    packet_handled = 0;
    reception_stage = IDLE;
  }

  void cancel_packet() {
    reception_stage = BOTCHED_PACKET;
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
    const unsigned long now = micros();
    const unsigned long duration = now > last_rise_micros ? now - last_rise_micros : 0;
    last_rise_micros = now;

    if (duration >= MIN_PACKET_SPACING) {
      packet_delimited();
#     if DIAGNOSTICS
        delimiter_micros = duration;
#     endif
      if (reception_stage != BOTCHED_PACKET) {
        packet_timeout_micros = now + MAX_PACKET_PREAMBLE + MAX_PAYLOAD_TIMEOUT + MAX_PACKET_SPACING;
      }
      reception_stage = DELIMITED;
    } else if (reception_stage == DELIMITED) {
      if (duration < MIN_PACKET_PREAMBLE || duration > MAX_PACKET_PREAMBLE) {
        if (VERBOSE) {
          Serial.print(duration);
          Serial.println(" µs rise after delimiter");
        }
        cancel_packet();
        return;
      }
#     if DIAGNOSTICS
        preamble_micros = duration;
#     endif
      reception_stage = OPENED;
      extra_peaks_received = 0;
      bits_received = 0;
      packet_timeout_micros = now + MAX_PAYLOAD_TIMEOUT + MAX_PACKET_SPACING;
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
    const unsigned long ss_bits_received = bits_received;
    const unsigned long ss_last_rise_micros = last_rise_micros;
    const bool has_new = reception_stage == 32
                      && extra_peaks_received == int(bits_received & 1)
                      && bits_received != packet_handled;
    interrupts();
    const unsigned long now = micros();
    if (packet_handled && now >= packet_timeout_micros) {
      if (VERBOSE) {
        Serial.println("Stop expecting packet repeats");
      }
      abort_packet_train();
    }
    if (has_new && now > ss_last_rise_micros + MAX_PEAK_SPACING) {
      packet_handled = ss_bits_received;
#     if DIAGNOSTICS
        Serial.print("After ");
        Serial.print(delimiter_micros);
        Serial.print(" + ");
        Serial.print(preamble_micros);
        Serial.println(" µs:");
#     endif
      return ss_bits_received;
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
