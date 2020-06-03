enum Notice { MISSED_PACKET,
              WRONG_PARITY,
              EXCESS_BITS, MISSING_BITS,
              MISSING_1_PEAK, MISSING_2_PEAKS,
              EXCESS_ADJACENT_PEAKS, MISSING_ADJACENT_PEAKS,
              PEAK_TOO_SOON, PEAK_TOO_LATE,
              PREAMBLE_TOO_SOON, PREAMBLE_TOO_LATE,
              EXCESS_TOTAL_PEAKS,
              SPURIOUS_PEAKS,
              END_OF_TRAIN
            };
static const unsigned long VOID_BITS = ~0ul;
static const unsigned long TRAIN_TIMEOUT = 0x60000;

inline unsigned long duration_from_to(unsigned long early, unsigned long later) {
  return later - early;
}

template <typename EventLogger, bool logTiming>
class ProtocolHandler {
    static const unsigned long MIN_ADJACENT_PEAK_SPACING = 416;
    static const unsigned long MAX_ADJACENT_PEAK_SPACING = 608;
    static const unsigned long MIN_SEPARATE_PEAK_SPACING = 1488;
    static const unsigned long MIN_PACKET_SPACING = 9984;
    static const unsigned long MIN_PACKET_PREAMBLE = 2720;
    static const unsigned long MAX_PACKET_PREAMBLE = 3200;
    static const unsigned long PARITY_TIMEOUT = 3 * MAX_ADJACENT_PEAK_SPACING;

    enum { IDLE = 0, DELIMITED = 1, PREAMBLED = 2, FINISHED = 66 };
    unsigned long peak_micros[FINISHED + 1] { micros() };
    unsigned long last_probed_micros;
    unsigned long train_handled = VOID_BITS;
    unsigned long train_established_micros;
    byte reception_stage = IDLE;

    void cancel_packet() {
      peak_micros[IDLE] = peak_micros[reception_stage];
      reception_stage = IDLE;
    }

    void packet_delimited() {
      if (reception_stage == FINISHED) {
        EventLogger::println(MISSED_PACKET, "Packet received but missed by main loop");
      } else if (reception_stage == FINISHED - 1) {
        EventLogger::println(MISSING_1_PEAK, "Missing 1 peak");
      } else if (reception_stage == FINISHED - 2) {
        EventLogger::println(MISSING_2_PEAKS, "Missing 2 peaks");
      } else if (reception_stage > DELIMITED) {
        EventLogger::print(SPURIOUS_PEAKS, "Invalid peak count ");
        EventLogger::println(SPURIOUS_PEAKS, reception_stage);
      }
    }

    void dump_packet(unsigned long now) {
      if (logTiming) {
        Serial.println("Timing:");
        for (byte i = 0; i <= FINISHED; ++i) {
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
      const unsigned long now = micros();
      const unsigned long spacing = duration_from_to(peak_micros[reception_stage], now);
      if (spacing >= MIN_PACKET_SPACING) {
        packet_delimited();
        cancel_packet();
        reception_stage = DELIMITED;
      } else if (reception_stage > IDLE) {
        ++reception_stage;
        if (reception_stage > FINISHED) {
          EventLogger::println(EXCESS_TOTAL_PEAKS, "Too many peaks in a packet");
          dump_packet(now);
          cancel_packet();
        }
      }
      peak_micros[reception_stage] = now;
    }

    bool is_alive() {
      noInterrupts();
      const unsigned long last_peak_micros = peak_micros[reception_stage];
      interrupts();
      if (last_probed_micros != last_peak_micros) {
        last_probed_micros = last_peak_micros;
        return true;
      } else {
        return false;
      }
    }

  private:
    unsigned long decode() const {
      const unsigned long preamble_duration = peak_micros[PREAMBLED] - peak_micros[DELIMITED];
      if (preamble_duration < MIN_PACKET_PREAMBLE) {
        EventLogger::print(PREAMBLE_TOO_SOON, preamble_duration);
        EventLogger::println(PREAMBLE_TOO_SOON, "µs short preamble after delimiter");
        return VOID_BITS;
      }
      if (preamble_duration > MAX_PACKET_PREAMBLE) {
        EventLogger::print(PREAMBLE_TOO_LATE, preamble_duration);
        EventLogger::println(PREAMBLE_TOO_LATE, "µs long preamble after delimiter");
        return VOID_BITS;
      }

      unsigned long bits_received = 0;
      byte extra_adjacent_peaks = 0;
      byte bitcount = 0;
      for (byte s = PREAMBLED; s < FINISHED; ++s) {
        const unsigned long spacing = duration_from_to(peak_micros[s], peak_micros[s + 1]);
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
          const byte bit = 1 + (bits_received & 1) - extra_adjacent_peaks;
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
    unsigned long receive() {
      noInterrupts();
      const unsigned long now = micros();
      const unsigned long last_peak = peak_micros[reception_stage];
      unsigned long bits_received = VOID_BITS;
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
