enum Notice { MISSED_PACKET,
              WRONG_PARITY,
              EXCESS_BITS, MISSING_BITS,
              MISSING_PEAKS,
              EXCESS_PEAKS,
              EXCESS_NEARBY_PEAKS, MISSING_NEARBY_PEAKS,
              MISSED_EPILOGUE,
              PEAK_TOO_SOON, PEAK_TOO_LATE,
              PREAMBLE_TOO_SOON, PREAMBLE_TOO_LATE,
              SPURIOUS_PEAKS
            };
static const unsigned long VOID_BITS = ~0ul;

template <typename EventLogger, bool logTiming>
class ProtocolHandler {
    static const unsigned long MIN_PEAK_SPACING = 416;
    static const unsigned long MAX_PEAK_SPACING = 608;
    static const unsigned long MIN_VALE_SPACING = 1488;
    static const unsigned long MIN_PACKET_SPACING = 9984;
    static const unsigned long MIN_PACKET_PREAMBLE = 2720;
    static const unsigned long MAX_PACKET_PREAMBLE = 3200;
    static const unsigned long PARITY_TIMEOUT = 3ul * MAX_PEAK_SPACING;
    static const unsigned long TRAIN_TIMEOUT = 0x30000;

    unsigned long last_rise_micros = micros();
    enum { IDLE = 0, DELIMITED = 1, PREAMBLED = 2, FINISHED = 66, FINAL };
    unsigned long delimiter_micros;
    unsigned long peak_micros[FINAL];
    byte reception_stage;
    unsigned long train_handled = VOID_BITS;
    unsigned long train_established_micros;

    void abort_packet_train() {
      train_handled = VOID_BITS;
      reception_stage = IDLE;
    }

    void cancel_packet() {
      reception_stage = IDLE;
    }

    void packet_delimited() {
      if (reception_stage == FINISHED) {
        EventLogger::println(MISSED_PACKET, "Packet received but missed by main loop");
      } else if (reception_stage > FINISHED / 2) {
        EventLogger::print(MISSING_PEAKS, "Invalid peak count ");
        EventLogger::println(MISSING_PEAKS, reception_stage);
      } else if (reception_stage > IDLE) {
        EventLogger::print(SPURIOUS_PEAKS, "Invalid peak count ");
        EventLogger::println(SPURIOUS_PEAKS, reception_stage);
      }
    }

  public:
    void handleRise() {
      const unsigned long now = micros();
      const unsigned long duration = now - last_rise_micros;
      last_rise_micros = now;
      if (duration >= MIN_PACKET_SPACING) {
        packet_delimited();
        delimiter_micros = duration;
        peak_micros[0] = now;
        reception_stage = DELIMITED;
      } else if (reception_stage > IDLE) {
        if (reception_stage < FINAL) {
          peak_micros[reception_stage] = now;
        }
        ++reception_stage;
      }
    }

    bool is_alive() const {
      static unsigned long last_seen_rise_micros = last_rise_micros;
      if (last_seen_rise_micros != last_rise_micros) {
        last_seen_rise_micros = last_rise_micros;
        return true;
      } else {
        return false;
      }
    }

    unsigned long decode() const {
      const unsigned long preamble_duration = peak_micros[1] - peak_micros[0];
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

      if (reception_stage > FINISHED) {
        EventLogger::print(EXCESS_PEAKS, reception_stage);
        EventLogger::println(EXCESS_PEAKS, " peaks in a packet");
        return VOID_BITS;
      }

      unsigned long bits_received = 0;
      byte consecutive_nearby_rises = 0;
      byte bitcount = 0;
      for (byte s = PREAMBLED; s < FINISHED; ++s) {
        const unsigned long duration = peak_micros[s] - peak_micros[s - 1];
        if (duration < MIN_VALE_SPACING) {
          if (duration < MIN_PEAK_SPACING) {
            EventLogger::print(PEAK_TOO_SOON, duration);
            EventLogger::print(PEAK_TOO_SOON, "µs peak #");
            EventLogger::println(PEAK_TOO_SOON, reception_stage);
            return VOID_BITS;
          }
          if (duration > MAX_PEAK_SPACING) {
            EventLogger::print(PEAK_TOO_LATE, duration);
            EventLogger::print(PEAK_TOO_LATE, "µs peak #");
            EventLogger::println(PEAK_TOO_LATE, reception_stage);
            return VOID_BITS;
          }
          ++consecutive_nearby_rises;
        } else {
          const byte bit = 1 + (bits_received & 1) - consecutive_nearby_rises;
          if (bit > 1) {
            if (bit & 0x80) {
              EventLogger::print(EXCESS_NEARBY_PEAKS, "Too many nearby peaks at #");
              EventLogger::println(EXCESS_NEARBY_PEAKS, s);
            } else {
              EventLogger::print(MISSING_NEARBY_PEAKS, "Too few nearby peaks at #");
              EventLogger::println(MISSING_NEARBY_PEAKS, s);
            }
            return VOID_BITS;
          }
          bits_received = (bits_received << 1) | bit;
          bitcount += 1;
          consecutive_nearby_rises = 0;
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
      if (consecutive_nearby_rises != (bits_received & 1)) {
        EventLogger::print(WRONG_PARITY, "Incorrect #parity peaks ");
        EventLogger::println(WRONG_PARITY, 1 + consecutive_nearby_rises);
        return VOID_BITS;
      }
      return bits_received;
    }

    unsigned long receive() {
      noInterrupts();
      const bool has_old = train_handled != VOID_BITS;
      const bool might_have_new = reception_stage >= FINISHED;
      if (!has_old && !might_have_new) {
        interrupts();
        return VOID_BITS;
      }
      const unsigned long now = micros();
      const unsigned long last_peak = peak_micros[reception_stage - 1];
      if (might_have_new && now - last_peak > PARITY_TIMEOUT) {
        // Keep blocking interrupts while we process the received packet - rises right now are noise
        if (logTiming) {
          Serial.print("After ");
          Serial.print(delimiter_micros);
          Serial.println("µs delimiter");
          for (byte i = 0; i < reception_stage; ++i) {
            Serial.print("  ");
            Serial.print(peak_micros[i]);
            Serial.print(" peak ");
            Serial.println(i + 1);
          }
          Serial.print("  ");
          Serial.print(now);
          Serial.println(" posted");
          Serial.print("  ");
          Serial.print(micros());
          Serial.println(" finishing this debug output");
        }
        const unsigned long bits_received = decode();
        reception_stage = IDLE;
        if (bits_received != VOID_BITS && bits_received != train_handled) {
          train_handled = bits_received;
          interrupts();
          train_established_micros = last_peak;
          return train_handled;
        }
      }
      if (has_old && now - train_established_micros > TRAIN_TIMEOUT) {
        abort_packet_train();
        interrupts();
        EventLogger::println(MISSED_EPILOGUE, "Stop expecting rest of packet train");
        return VOID_BITS;
      }
      interrupts();
      return VOID_BITS;
    }
};
