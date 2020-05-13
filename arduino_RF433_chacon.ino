static const int PIN_DIGITAL_IN = 2;
static const bool MAJORLOG = true;
static const bool MINORLOG = false;
#define MEASUREMENT 0

static const unsigned long MIN_PEAK_SPACING = 500;
static const unsigned long MAX_PEAK_SPACING = 600;
static const unsigned long MAX_VALE_SPACING = 1500;
static const unsigned long MIN_PACKET_SPACING = 10000;
static const unsigned long MAX_PACKET_SPACING = 11000;
static const unsigned long MIN_PACKET_PREAMBLE = 2800;
static const unsigned long MAX_PACKET_PREAMBLE = 3200;
static const unsigned long MAX_PAYLOAD_TIMEOUT = 32ul * (MAX_VALE_SPACING + 2 * MAX_PEAK_SPACING);

class ProtocolHandler {
  unsigned long train_handled = 0;
  unsigned long train_timeout_micros;
  int train_timeout_prolongements;
  unsigned long last_rise_micros = micros();
  enum { IDLE = -3, BOTCHED_PACKET = -2, DELIMITED = -1, OPENED = 0 };
  int reception_stage;
  int extra_peaks_received;
  unsigned long bits_received;
#if MEASUREMENT
  unsigned long delimiter_micros;
  unsigned long preamble_micros;
#endif

  void abort_packet_train() {
    digitalWrite(LED_BUILTIN, LOW);
    train_handled = 0;
    reception_stage = IDLE;
  }

  void cancel_packet() {
    if (train_handled) {
      reception_stage = BOTCHED_PACKET;
    } else {
      abort_packet_train();
    }
  }

  void packet_delimited() {
    if (MAJORLOG && reception_stage > OPENED) {
      if (reception_stage != 32) {
        Serial.print("Invalid vale count ");
        Serial.println(reception_stage);
      } else if (extra_peaks_received != int(bits_received & 1)) {
        Serial.print("Invalid final peak count ");
        Serial.println(1 + extra_peaks_received);
      } else if (train_handled == 0) {
        Serial.println("Packet received but missed by main loop");
      } else if (train_handled != bits_received) {
        Serial.print(train_handled, BIN);
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
#     if MEASUREMENT
        delimiter_micros = duration;
#     endif
      if (train_handled && reception_stage != BOTCHED_PACKET) {
        ++train_timeout_prolongements;
        train_timeout_micros = now + MAX_PACKET_PREAMBLE + MAX_PAYLOAD_TIMEOUT + MAX_PACKET_SPACING;
      }
      reception_stage = DELIMITED;
    } else if (reception_stage == DELIMITED) {
      if (duration < MIN_PACKET_PREAMBLE) {
        if (MINORLOG && duration > MIN_PACKET_PREAMBLE * 0.75) {
          Serial.print(duration);
          Serial.println("µs short preamble after delimiter");
        }
        cancel_packet();
        return;
      }
      if (duration > MAX_PACKET_PREAMBLE) {
        if (MINORLOG && duration < MAX_PACKET_PREAMBLE * 1.25) {
          Serial.print(duration);
          Serial.println("µs long preamble after delimiter");
        }
        cancel_packet();
        return;
      }
      if (!train_handled) {
        digitalWrite(LED_BUILTIN, HIGH);
#       if MEASUREMENT
          preamble_micros = duration;
#       endif
        train_timeout_prolongements = 0;
      }
      train_timeout_micros = now + MAX_PAYLOAD_TIMEOUT + MAX_PACKET_SPACING;
      reception_stage = OPENED;
      extra_peaks_received = 0;
      bits_received = 0;
    } else if (reception_stage >= OPENED) {
      if (duration < MIN_PEAK_SPACING) {
        if (MINORLOG) {
          Serial.print(duration);
          Serial.print("µs peak in vale #");
          Serial.println(reception_stage);
        }
        cancel_packet();
        return;
      }
      if (duration <= MAX_PEAK_SPACING) {
        ++extra_peaks_received;
      } else {
        if (duration <= MAX_VALE_SPACING) {
          if (MINORLOG) {
            Serial.print(duration);
            Serial.print("µs vale after vale #");
            Serial.println(reception_stage);
          }
          cancel_packet();
          return;
        }
        const int bit = 1 + int(bits_received & 1) - extra_peaks_received;
        if (bit < 0 || bit > 1) {
          if (MINORLOG) {
            Serial.print("Invalid peak count ");
            Serial.println(1 + extra_peaks_received);
          }
          cancel_packet();
          return;
        }
        bits_received = (bits_received << 1) | bit;
        extra_peaks_received = 0;
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
                      && bits_received != train_handled;
    interrupts();
    const unsigned long now = micros();
    if (train_handled && now >= train_timeout_micros) {
      if (MAJORLOG) {
        Serial.print("Stop expecting rest of packet train after ");
        Serial.print(train_timeout_prolongements);
        Serial.println(" prolongements");
      }
      abort_packet_train();
    }
    if (has_new && now > ss_last_rise_micros + MAX_PEAK_SPACING) {
      train_handled = ss_bits_received;
#     if MEASUREMENT
        Serial.print("After ");
        Serial.print(delimiter_micros);
        Serial.print("µs delimiter + ");
        Serial.print(preamble_micros);
        Serial.println("µs preamble:");
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
