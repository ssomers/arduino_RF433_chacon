enum ProtocolNotice : uint8_t { END_OF_TRAIN,
                                INVALID_PREAMBLE = 1,
                                MISSING_TOTAL_PEAKS = 2, EXCESS_TOTAL_PEAKS = 3,
                                WRONG_PEAK_SPACING = 4, WRONG_PEAK_COUNT = 5,
                                MISSING_BITS = 6, EXCESS_BITS = 7,
                                WRONG_PARITY = 8,
                                MISSED_PACKET = 9,
                              };
enum { PEAKS = 65, IGNORED_WHEN_INCOMPLETE = 60 };
enum PeakBufferStage : uint8_t { IDLE, DELIMITED, STARTED, FINISHED = DELIMITED + PEAKS };

inline uint32_t duration_from_to(uint32_t early, uint32_t later) {
  return later - early;
}

struct Reception {
  uint32_t bits_received;
  uint32_t time_received;
};

template <typename EventLogger>
class PeakBuffer {
    static const uint8_t SCALING = 32; // enough for spacing to max out on delimiters
    static const uint8_t MIN_ADJACENT_PEAK_SPACING = 10; // unit = SCALING µs
    static const uint8_t MAX_ADJACENT_PEAK_SPACING = 20; // unit = SCALING µs
    static const uint8_t MIN_SEPARATE_PEAK_SPACING = 40; // unit = SCALING µs
    static const uint8_t PACKET_FINAL_TIMEOUT      = 64; // unit = SCALING µs
    static const uint8_t MIN_PACKET_PREAMBLE       = 80; // unit = SCALING µs
    static const uint8_t MAX_PACKET_PREAMBLE      = 100; // unit = SCALING µs

    uint32_t last_rise_micros;
    uint8_t reception_stage;
    uint8_t peak_32micros[PEAKS]; // unit = SCALING µs

  public:
    void initialize_at_startup() {
      last_rise_micros = micros();
      reception_stage = IDLE;
    }

    void initialize_from(PeakBuffer const& other) {
      last_rise_micros = other.last_rise_micros;
      reception_stage = IDLE;
    }

    uint32_t probe_last_rise_micros() const {
      return last_rise_micros;
    }

    uint8_t spacing_32micros(uint32_t now) const {
      const uint32_t spacing = duration_from_to(last_rise_micros, now);
      return spacing / SCALING > 0xFF ? 0xFF : spacing / SCALING;
    }

    bool appears_final_at(uint32_t now) const {
      return reception_stage == FINISHED && duration_from_to(last_rise_micros, now) > PACKET_FINAL_TIMEOUT * SCALING;
    }

    void handle_rise(uint32_t now, uint8_t spacing_32micros) {
      if (spacing_32micros == 0xFF) {
        reception_stage = DELIMITED;
      } else if (reception_stage > IDLE) {
        ++reception_stage;
        // if we get so many consecutive peaks that reception_stage spills, just let it be
        if (reception_stage <= FINISHED) {
          peak_32micros[reception_stage - STARTED] = spacing_32micros;
        }
      }
      last_rise_micros = now;
    }

    uint8_t stage() const {
      return reception_stage;
    }

    bool decode(Reception& packet) const {
      if (reception_stage > FINISHED) {
        EventLogger::print(EXCESS_TOTAL_PEAKS, reception_stage - DELIMITED);
        EventLogger::println(EXCESS_TOTAL_PEAKS, " peaks in a packet");
        return false;
      }
      if (reception_stage < FINISHED) {
        EventLogger::print(MISSING_TOTAL_PEAKS, reception_stage - DELIMITED);
        EventLogger::println(MISSING_TOTAL_PEAKS, " peaks in a packet");
        return false;
      }

      if (peak_32micros[0] < MIN_PACKET_PREAMBLE || peak_32micros[0] > MAX_PACKET_PREAMBLE) {
        EventLogger::print(INVALID_PREAMBLE, peak_32micros[0] * SCALING);
        EventLogger::println(INVALID_PREAMBLE, "µs preamble after delimiter");
        return false;
      }

      uint8_t extra_adjacent_peaks = 0;
      uint8_t bitcount = 0;
      uint8_t spacing_errors = 0;
      uint8_t bit_errors = 0;
      packet.bits_received = 0;
      packet.time_received = last_rise_micros;
      for (uint8_t p = 1; p < PEAKS; ++p) {
        const uint8_t spacing_32micros = peak_32micros[p];
        if (spacing_32micros < MIN_SEPARATE_PEAK_SPACING) {
          spacing_errors += (spacing_32micros < MIN_ADJACENT_PEAK_SPACING);
          spacing_errors += (spacing_32micros > MAX_ADJACENT_PEAK_SPACING);
          extra_adjacent_peaks += 1;
        } else {
          const uint8_t bit = 1 + (packet.bits_received & 1) - extra_adjacent_peaks;
          bit_errors += (bit > 1);
          packet.bits_received = (packet.bits_received << 1) | (bit & 1);
          bitcount += 1;
          extra_adjacent_peaks = 0;
        }
      }
      if (spacing_errors) {
        EventLogger::println(WRONG_PEAK_SPACING, "Peak spacing wildly out of whack");
        return false;
      }
      if (bit_errors) {
        EventLogger::println(WRONG_PEAK_COUNT, "Wrong number of adjacent peaks");
        return false;
      }
      if (bitcount < 32) {
        EventLogger::print(MISSING_BITS, "#bits=");
        EventLogger::println(MISSING_BITS, bitcount);
        return false;
      }
      if (bitcount > 32) {
        EventLogger::print(EXCESS_BITS, "#bits=");
        EventLogger::println(EXCESS_BITS, bitcount);
        return false;
      }
      if (extra_adjacent_peaks != (packet.bits_received & 1)) {
        EventLogger::print(WRONG_PARITY, "Incorrect #parity peaks ");
        EventLogger::println(WRONG_PARITY, 1 + extra_adjacent_peaks);
        return false;
      }
      return true;
    }

    void dump(uint32_t now) const {
      Serial.println("Timing:");
      for (uint8_t p = 0; p < min(PEAKS, reception_stage - DELIMITED); ++p) {
        Serial.print("  -");
        Serial.print(peak_32micros[p] * SCALING);
        Serial.print(" peak ");
        Serial.println(p);
      }
      Serial.print("  ");
      Serial.print(last_rise_micros);
      Serial.println(" last peak");
      Serial.print("  ");
      Serial.print(now);
      Serial.println(" receiving");
      Serial.print("  ");
      Serial.print(micros());
      Serial.println(" finishing this debug output");
    }
};
