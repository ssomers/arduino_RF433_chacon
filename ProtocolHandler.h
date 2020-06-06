enum ProtocolNotice : uint8_t { NO_NOTICE,
                                END_OF_TRAIN,
                                SPURIOUS_PEAKS,
                                EXCESS_TOTAL_PEAKS,
                                PREAMBLE_TOO_SOON = 4, PREAMBLE_TOO_LATE = 5,
                                MISSING_SOME_PEAKS = 6,
                                PEAK_TOO_SOON = 7, PEAK_TOO_LATE = 8,
                                MISSING_ADJACENT_PEAKS = 10, EXCESS_ADJACENT_PEAKS = 11,
                                MISSING_BITS = 12, EXCESS_BITS = 11,
                                WRONG_PARITY = 13,
                                MISSED_PACKET = 14,
                              };
static const uint32_t VOID_BITS = ~0ul;
static const uint32_t TRAIN_TIMEOUT = 0x60000;

inline uint32_t duration_from_to(uint32_t early, uint32_t later) {
  return later - early;
}

template <typename EventLogger, bool logTiming>
class ProtocolHandler {
    static const uint32_t MIN_ADJACENT_PEAK_SPACING = 0x100;
    static const uint32_t MAX_ADJACENT_PEAK_SPACING = 0x300;
    static const uint32_t MIN_SEPARATE_PEAK_SPACING = 0x500;
    static const uint32_t MIN_PACKET_SPACING = 0x2000;
    static const uint32_t MIN_PACKET_PREAMBLE = 0XA00;
    static const uint32_t MAX_PACKET_PREAMBLE = 0xD00;
    static const uint32_t PARITY_TIMEOUT = 3 * MAX_ADJACENT_PEAK_SPACING;

    enum { IDLE = 0, DELIMITED = 1, PREAMBLED = 2, FINISHED = 66 };
    uint32_t peak_micros[FINISHED + 1] { micros() };
    uint32_t last_probed_micros;
    uint32_t train_handled = VOID_BITS;
    uint32_t train_established_micros;
    uint8_t reception_stage = IDLE;

    void cancel_packet() {
      peak_micros[IDLE] = peak_micros[reception_stage];
      reception_stage = IDLE;
    }

    void packet_delimited() {
      if (reception_stage == FINISHED) {
        EventLogger::println(MISSED_PACKET, "Packet received but missed by main loop");
      } else if (reception_stage >= FINISHED - 22) {
        EventLogger::println(MISSING_SOME_PEAKS, "Missing some peaks");
      } else if (reception_stage > DELIMITED) {
        EventLogger::print(SPURIOUS_PEAKS, "Invalid peak count ");
        EventLogger::println(SPURIOUS_PEAKS, reception_stage);
      }
    }

    void dump_packet(uint32_t now) {
      if (logTiming) {
        Serial.println("Timing:");
        for (uint8_t i = 0; i <= FINISHED; ++i) {
          Serial.print("  ");
          Serial.print(peak_micros[i]);
          Serial.print(" peak ");
          Serial.println(i);
        }
        Serial.print("  ");
        Serial.print(now);
        Serial.println(" receiving");
        Serial.print("  ");
        Serial.print(micros());
        Serial.println(" finishing this debug output");
      }
    }

  public:
    void handleRise() {
      const uint32_t now = micros();
      const uint32_t spacing = duration_from_to(peak_micros[reception_stage], now);
      if (spacing >= MIN_PACKET_SPACING) {
        packet_delimited();
        cancel_packet();
        reception_stage = DELIMITED;
      } else switch (reception_stage) {
          case IDLE:
            break;
          case DELIMITED:
            if (is_valid_preamble(spacing)) {
              reception_stage = PREAMBLED;
            } else {
              reception_stage = IDLE;
            }
            break;
          case FINISHED:
            EventLogger::println(EXCESS_TOTAL_PEAKS, "Too many peaks in a packet");
            dump_packet(now);
            reception_stage = IDLE;
            break;
          default:
            ++reception_stage;
        };
      peak_micros[reception_stage] = now;
    }

    bool is_alive() {
      noInterrupts();
      const uint32_t last_peak_micros = peak_micros[reception_stage];
      interrupts();
      if (last_probed_micros != last_peak_micros) {
        last_probed_micros = last_peak_micros;
        return true;
      } else {
        return false;
      }
    }

  private:
    static bool is_valid_preamble(uint32_t preamble_duration) {
      if (preamble_duration < MIN_PACKET_PREAMBLE) {
        EventLogger::print(PREAMBLE_TOO_SOON, preamble_duration);
        EventLogger::println(PREAMBLE_TOO_SOON, "µs short preamble after delimiter");
        return false;
      }
      if (preamble_duration > MAX_PACKET_PREAMBLE) {
        EventLogger::print(PREAMBLE_TOO_LATE, preamble_duration);
        EventLogger::println(PREAMBLE_TOO_LATE, "µs long preamble after delimiter");
        return false;
      }
      return true;
    }

    uint32_t decode() const {
      uint32_t bits_received = 0;
      uint8_t extra_adjacent_peaks = 0;
      uint8_t bitcount = 0;
      for (uint8_t s = PREAMBLED; s < FINISHED; ++s) {
        const uint32_t spacing = duration_from_to(peak_micros[s], peak_micros[s + 1]);
        if (spacing < MIN_SEPARATE_PEAK_SPACING) {
          if (spacing < MIN_ADJACENT_PEAK_SPACING) {
            EventLogger::print(PEAK_TOO_SOON, spacing);
            EventLogger::print(PEAK_TOO_SOON, "µs peak #");
            EventLogger::println(PEAK_TOO_SOON, s);
            return VOID_BITS;
          }
          if (spacing > MAX_ADJACENT_PEAK_SPACING) {
            EventLogger::print(PEAK_TOO_LATE, spacing);
            EventLogger::print(PEAK_TOO_LATE, "µs peak #");
            EventLogger::println(PEAK_TOO_LATE, s);
            return VOID_BITS;
          }
          extra_adjacent_peaks += 1;
        } else {
          const uint8_t bit = 1 + (bits_received & 1) - extra_adjacent_peaks;
          if (bit > 1) {
            if (bit & 0x80) {
              EventLogger::print(EXCESS_ADJACENT_PEAKS, "Too many adjacent peaks at #");
              EventLogger::println(EXCESS_ADJACENT_PEAKS, s);
            } else {
              EventLogger::print(MISSING_ADJACENT_PEAKS, "Too few adjacent peaks at #");
              EventLogger::println(MISSING_ADJACENT_PEAKS, s);
            }
            return VOID_BITS;
          }
          bits_received = (bits_received << 1) | bit;
          bitcount += 1;
          extra_adjacent_peaks = 0;
        }
      }
      if (bitcount < 32) {
        EventLogger::print(MISSING_BITS, "#bits=");
        EventLogger::println(MISSING_BITS, bitcount);
        return VOID_BITS;
      }
      if (bitcount > 32) {
        EventLogger::print(EXCESS_BITS, "#bits=");
        EventLogger::println(EXCESS_BITS, bitcount);
        return VOID_BITS;
      }
      if (extra_adjacent_peaks != (bits_received & 1)) {
        EventLogger::print(WRONG_PARITY, "Incorrect #parity peaks ");
        EventLogger::println(WRONG_PARITY, 1 + extra_adjacent_peaks);
        return VOID_BITS;
      }
      return bits_received;
    }

  public:
    uint32_t receive() {
      noInterrupts();
      const uint32_t now = micros();
      const uint32_t last_peak = peak_micros[reception_stage];
      uint32_t bits_received = VOID_BITS;
      if (reception_stage == FINISHED && duration_from_to(last_peak, now) > PARITY_TIMEOUT) {
        // Keep blocking interrupts while we process the received packet - peaks right now are noise
        dump_packet(now);
        bits_received = decode();
        cancel_packet();
      }
      interrupts();
      if (bits_received != VOID_BITS) {
        if (bits_received != train_handled) { // not just a repeat in the same train
          train_handled = bits_received;
          train_established_micros = last_peak;
          return train_handled;
        }
      }
      if (train_handled != VOID_BITS) {
        if (duration_from_to(train_established_micros, now) > TRAIN_TIMEOUT) {
          train_handled = VOID_BITS;
          EventLogger::println(END_OF_TRAIN, "Stop expecting rest of packet train");
        }
      }
      return VOID_BITS;
    }
};
