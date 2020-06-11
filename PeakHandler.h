enum ProtocolNotice : uint8_t { END_OF_TRAIN,
                                SPURIOUS_PEAKS = 1,
                                INVALID_PREAMBLE = 2,
                                MISSING_SOME_PEAKS = 3,
                                EXCESS_TOTAL_PEAKS = 4,
                                WRONG_PEAK_SPACING = 5, WRONG_PEAK_COUNT = 6,
                                MISSING_BITS = 7, EXCESS_BITS = 8,
                                WRONG_PARITY = 9,
                                MISSED_PACKET = 10, DIRTY_BUFFER = 11,
                              };
enum BufferStage : uint8_t { IDLE, DELIMITED, PREAMBLED, STARTED, FINISHED = PREAMBLED + 64 };

inline uint32_t duration_from_to(uint32_t early, uint32_t later) {
  return later - early;
}

struct Reception {
  uint32_t bits_received;
  uint32_t time_received;
};

template <typename EventLogger, bool logTiming>
class PeakHandler {
    static const uint8_t SCALING = 32; // enough to fit MIN_PACKET_SPACING / SCALING into a byte
    static const uint8_t MIN_ADJACENT_PEAK_SPACING = 10; // unit = SCALING µs
    static const uint8_t MAX_ADJACENT_PEAK_SPACING = 20; // unit = SCALING µs
    static const uint8_t MIN_SEPARATE_PEAK_SPACING = 40; // unit = SCALING µs
    static const uint8_t PACKET_FINAL_TIMEOUT      = 64; // unit = SCALING µs
    static const uint8_t MIN_PACKET_PREAMBLE       = 80; // unit = SCALING µs
    static const uint8_t MAX_PACKET_PREAMBLE      = 100; // unit = SCALING µs
    static const uint32_t MIN_PACKET_SPACING = 0x100 * SCALING; // unit = µs

    enum { IGNORED = 48 }; // completely ignore "packets" going no further than this

    static bool validate_preamble(uint16_t preamble_duration_32micros) {
      if (preamble_duration_32micros < MIN_PACKET_PREAMBLE || preamble_duration_32micros > MAX_PACKET_PREAMBLE) {
        EventLogger::print(INVALID_PREAMBLE, preamble_duration_32micros * SCALING);
        EventLogger::println(INVALID_PREAMBLE, "µs preamble after delimiter");
        return false;
      } else {
        return true;
      }
    }

  public:
    class Buffer {
        friend class PeakHandler;
        uint32_t last_rise_micros;
        BufferStage reception_stage;
        uint8_t peak_32micros[FINISHED - PREAMBLED + 1]; // unit = SCALING µs

      public:
        void initialize_at_startup() {
          last_rise_micros = micros();
          reception_stage = IDLE;
        }

        void initialize_from(Buffer const& other) {
          last_rise_micros = other.last_rise_micros;
          reception_stage = IDLE;
        }

        BufferStage stage() const {
          return reception_stage;
        }

        bool decode(Reception& packet) const {
          if (!validate_preamble(peak_32micros[0])) {
            return false;
          }

          uint8_t extra_adjacent_peaks = 0;
          uint8_t bitcount = 0;
          uint8_t spacing_errors = 0;
          uint8_t bit_errors = 0;
          packet.bits_received = 0;
          packet.time_received = last_rise_micros;
          for (uint8_t s = STARTED; s <= FINISHED; ++s) {
            const uint8_t spacing_32micros = peak_32micros[s - PREAMBLED];
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
          if (logTiming) {
            Serial.println("Timing:");
            for (uint8_t i = PREAMBLED; i < reception_stage; ++i) {
              Serial.print("  -");
              Serial.print(peak_32micros[i - PREAMBLED] * SCALING);
              Serial.print(" peak ");
              Serial.println(i - PREAMBLED);
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
        }

        bool appears_final_at(uint32_t micros) const {
          return reception_stage == FINISHED && duration_from_to(last_rise_micros, micros) > PACKET_FINAL_TIMEOUT * SCALING;
        }

        void mark_as_seen() {
          reception_stage = IDLE;
        }
    };

  private:
    static const uint8_t RECEPTION_BUFFERS = 4;
    Buffer buffers[RECEPTION_BUFFERS];
    volatile uint8_t current_buffer_incoming = 0;
    uint32_t last_probed_micros;

    void finish_packet_offline(uint8_t buffer_incoming) {
      const uint8_t next_buffer_incoming = next_buffer(buffer_incoming);
      if (buffers[next_buffer_incoming].stage() != IDLE) { // not marked as seen
        EventLogger::println(DIRTY_BUFFER, "Received packet not cleared");
      }
      buffers[next_buffer_incoming].initialize_from(buffers[buffer_incoming]);
      current_buffer_incoming = next_buffer_incoming;
    }

  public:
    static uint8_t next_buffer(uint8_t b) {
      return b + 1 < RECEPTION_BUFFERS ? b + 1 : 0;
    }

    Buffer& access_buffer(uint8_t b) {
      return buffers[b];
    }

    // Must be invoked with interrupts disabled
    bool finalize_offline(uint8_t buffer_index, uint32_t micros) {
      if (buffer_index != current_buffer_incoming) {
        return true;
      }
      if (buffers[buffer_index].appears_final_at(micros)) {
        finish_packet_offline(buffer_index);
        return true;
      }
      return false;
    }

  public:
    PeakHandler() {
      buffers[0].initialize_at_startup();
    }

    void handle_rise() {
      const uint32_t now = micros();
      const uint8_t buffer_incoming = current_buffer_incoming;
      const uint32_t spacing = duration_from_to(buffers[buffer_incoming].last_rise_micros, now);
      const BufferStage prev_reception_stage = buffers[buffer_incoming].reception_stage;
      uint8_t next_buffer_incoming = buffer_incoming;
      BufferStage next_reception_stage;
      if (spacing >= MIN_PACKET_SPACING) {
        if (prev_reception_stage > IGNORED) {
          next_buffer_incoming = next_buffer(buffer_incoming);
          if (buffers[next_buffer_incoming].reception_stage != IDLE) { // not marked as seen
            EventLogger::println(MISSED_PACKET, "Packet not timely processed by main loop");
          }
        }
        next_reception_stage = DELIMITED;
      } else if (prev_reception_stage == IDLE) {
        next_reception_stage = IDLE;
      } else if (prev_reception_stage < FINISHED) {
        next_reception_stage = BufferStage(prev_reception_stage + 1);
        buffers[next_buffer_incoming].peak_32micros[next_reception_stage - PREAMBLED] = uint8_t(uint16_t(spacing) / SCALING);
      } else {
        EventLogger::println(EXCESS_TOTAL_PEAKS, "Too many peaks in a packet");
        buffers[buffer_incoming].dump(now);
        next_reception_stage = IDLE;
      }

      current_buffer_incoming = next_buffer_incoming;
      buffers[next_buffer_incoming].reception_stage = next_reception_stage;
      buffers[next_buffer_incoming].last_rise_micros = now;
    }

    bool has_been_alive() {
      noInterrupts();
      const uint32_t last_rise_micros = buffers[current_buffer_incoming].last_rise_micros;
      interrupts();
      if (last_probed_micros != last_rise_micros) {
        last_probed_micros = last_rise_micros;
        return true;
      } else {
        return false;
      }
    }
};
