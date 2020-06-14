enum ProtocolNotice : uint8_t { END_OF_TRAIN,
                                INVALID_PREAMBLE = 1,
                                MISSING_TOTAL_PEAKS = 2, EXCESS_TOTAL_PEAKS = 3,
                                WRONG_PEAK_SPACING = 4, WRONG_PEAK_COUNT = 5,
                                MISSING_BITS = 6, EXCESS_BITS = 7,
                                WRONG_PARITY = 8,
                                MISSED_PACKET = 9,
                              };

static const uint8_t PEAKS = 65;

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
    static const uint8_t PACKET_FINAL_TIMEOUT      = 60; // unit = SCALING µs
    static const uint8_t MIN_PACKET_PREAMBLE       = 80; // unit = SCALING µs
    static const uint8_t MAX_PACKET_PREAMBLE      = 100; // unit = SCALING µs

    uint32_t last_peak_micros;
    uint8_t peak_count;
    uint8_t peak_preceding32micros[PEAKS - 1]; // unit = SCALING µs, can't record delimiter peak

  public:
    void initialize_at_startup() {
      last_peak_micros = micros();
      peak_count = 0;
    }

    void initialize_from(PeakBuffer const& other) {
      last_peak_micros = other.last_peak_micros;
      peak_count = 0;
    }

    void initialize_delimited(uint32_t now) {
      last_peak_micros = now;
      peak_count = 1;
    }

    void register_peak(uint32_t now, uint8_t preceding32micros) {
      last_peak_micros = now;
      if (peak_count > 0) {
        ++peak_count;
        // if we get so many consecutive peaks that peak_count spills, just let it be
        if (peak_count <= PEAKS) {
          peak_preceding32micros[peak_count - 2] = preceding32micros; // peak 2 is first recorded peak
        }
      }
    }

    uint32_t probe_last_peak_micros() const {
      return last_peak_micros;
    }

    uint8_t preceding32micros(uint32_t now) const {
      const uint32_t spacing = duration_from_to(last_peak_micros, now);
      return spacing / SCALING > 0xFF ? 0xFF : spacing / SCALING;
    }

    bool appears_final_at(uint32_t now) const {
      return peak_count == PEAKS && duration_from_to(last_peak_micros, now) > PACKET_FINAL_TIMEOUT * SCALING;
    }

    bool has_almost_seen_packet() const {
      return peak_count >= PEAKS * 9 / 10;
    }

    bool decode(Reception& packet) const {
      if (peak_count > PEAKS) {
        EventLogger::print(EXCESS_TOTAL_PEAKS, peak_count);
        EventLogger::println(EXCESS_TOTAL_PEAKS, " peaks in a packet");
        return false;
      }
      if (peak_count < PEAKS) {
        EventLogger::print(MISSING_TOTAL_PEAKS, peak_count);
        EventLogger::println(MISSING_TOTAL_PEAKS, " peaks in a packet");
        return false;
      }

      if (peak_preceding32micros[0] < MIN_PACKET_PREAMBLE || peak_preceding32micros[0] > MAX_PACKET_PREAMBLE) {
        EventLogger::print(INVALID_PREAMBLE, peak_preceding32micros[0] * SCALING);
        EventLogger::println(INVALID_PREAMBLE, "µs preamble after delimiter");
        return false;
      }

      uint8_t extra_adjacent_peaks = 0;
      uint8_t bitcount = 0;
      uint8_t spacing_errors = 0;
      uint8_t bit_errors = 0;
      packet.bits_received = 0;
      packet.time_received = last_peak_micros;
      for (uint8_t p = 1; p < PEAKS - 1; ++p) {
        const uint8_t preceding32micros = peak_preceding32micros[p];
        if (preceding32micros < MIN_SEPARATE_PEAK_SPACING) {
          spacing_errors += (preceding32micros < MIN_ADJACENT_PEAK_SPACING);
          spacing_errors += (preceding32micros > MAX_ADJACENT_PEAK_SPACING);
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
      for (uint8_t p = 0; p < min(PEAKS, peak_count) - 1; ++p) {
        Serial.print("  -");
        Serial.print(peak_preceding32micros[p] * SCALING);
        Serial.print(" peak ");
        Serial.println(p);
      }
      Serial.print("  ");
      Serial.print(last_peak_micros);
      Serial.println(" last peak");
      Serial.print("  ");
      Serial.print(now);
      Serial.println(" receiving");
      Serial.print("  ");
      Serial.print(micros());
      Serial.println(" finishing this debug output");
    }
};
