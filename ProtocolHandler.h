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
static const uint32_t VOID_BITS = ~0ul;
static const uint32_t TRAIN_TIMEOUT = 0x60000;

inline uint32_t duration_from_to(uint32_t early, uint32_t later) {
  return later - early;
}

template <typename T>
inline T increment_modulo(T old, T limit) {
  return old + 1 < limit ? old + 1 : 0;
}

template <typename EventLogger, bool logTiming>
class ProtocolHandler {
    static const uint8_t MIN_ADJACENT_PEAK_SPACING = 10; // unit = 32 µs
    static const uint8_t MAX_ADJACENT_PEAK_SPACING = 20; // unit = 32 µs
    static const uint8_t MIN_SEPARATE_PEAK_SPACING = 40; // unit = 32 µs
    static const uint8_t MIN_PACKET_PREAMBLE       = 80; // unit = 32 µs
    static const uint8_t MAX_PACKET_PREAMBLE      = 100; // unit = 32 µs
    static const uint32_t MIN_PACKET_SPACING = 0x100 * 32; // unit = µs
    static const uint32_t PARITY_TIMEOUT = 3 * MAX_ADJACENT_PEAK_SPACING; // unit = µs

    enum { IDLE, DELIMITED, PREAMBLED, STARTED, FINISHED = PREAMBLED + 64 };
    enum { RECORDING = PREAMBLED };
    enum { IGNORED = 48 }; // completely ignore "packets" going no further than this
    typedef uint8_t PacketPeakTiming [FINISHED - RECORDING + 1]; // unit = 8 µs
    struct Reception {
      uint32_t last_rise_micros;
      PacketPeakTiming peak_32micros;
      uint8_t reception_stage = IDLE;

      uint32_t decode() const {
        uint32_t bits_received = 0;
        uint8_t extra_adjacent_peaks = 0;
        uint8_t bitcount = 0;
        uint8_t spacing_errors = 0;
        uint8_t bit_errors = 0;

        const uint8_t spacing_32micros = peak_32micros[PREAMBLED - RECORDING];
        if (!is_valid_preamble(spacing_32micros)) {
          return VOID_BITS;
        }

        for (uint8_t s = STARTED; s <= FINISHED; ++s) {
          const uint8_t spacing_32micros = peak_32micros[s - RECORDING];
          if (spacing_32micros < MIN_SEPARATE_PEAK_SPACING) {
            spacing_errors += (spacing_32micros < MIN_ADJACENT_PEAK_SPACING);
            spacing_errors += (spacing_32micros > MAX_ADJACENT_PEAK_SPACING);
            extra_adjacent_peaks += 1;
          } else {
            const uint8_t bit = 1 + (bits_received & 1) - extra_adjacent_peaks;
            bit_errors += (bit > 1);
            bits_received = (bits_received << 1) | (bit & 1);
            bitcount += 1;
            extra_adjacent_peaks = 0;
          }
        }
        if (spacing_errors) {
          EventLogger::println(WRONG_PEAK_SPACING, "Peak spacing wildly out of whack");
          return VOID_BITS;
        }
        if (bit_errors) {
          EventLogger::println(WRONG_PEAK_COUNT, "Wrong number of adjacent peaks");
          return VOID_BITS;
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

      void dump(uint32_t now) const {
        if (logTiming) {
          Serial.println("Timing:");
          for (uint8_t i = RECORDING; i < reception_stage; ++i) {
            Serial.print("  -");
            Serial.print(peak_32micros[i - RECORDING] * 32);
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
    };
    static const uint8_t RECEPTION_BUFFERS = 4;
    Reception buffers[RECEPTION_BUFFERS];
    uint32_t last_probed_micros;
    uint32_t train_handled = VOID_BITS;
    uint32_t train_established_micros;
    volatile uint8_t current_buffer_incoming = 0;
    uint8_t buffer_receiving = 0;

    void finish_packet(uint8_t buffer_incoming) {
      const uint8_t next_buffer_incoming = increment_modulo(buffer_incoming, RECEPTION_BUFFERS);
      if (buffers[next_buffer_incoming].reception_stage != IDLE) { // not marked as seen
        EventLogger::println(DIRTY_BUFFER, "Received packet not cleared");
        buffers[next_buffer_incoming].reception_stage = IDLE;
      }
      buffers[next_buffer_incoming].last_rise_micros = buffers[buffer_incoming].last_rise_micros;
      current_buffer_incoming = next_buffer_incoming;
    }

    static bool is_valid_preamble(uint16_t preamble_duration_32micros) {
      if (preamble_duration_32micros < MIN_PACKET_PREAMBLE || preamble_duration_32micros > MAX_PACKET_PREAMBLE) {
        EventLogger::print(INVALID_PREAMBLE, preamble_duration_32micros * 32);
        EventLogger::println(INVALID_PREAMBLE, "µs preamble after delimiter");
        return false;
      } else {
        return true;
      }
    }

  public:
    ProtocolHandler() {
      buffers[0].last_rise_micros = micros();
    }

    void handleRise() {
      const uint32_t now = micros();
      const uint8_t buffer_incoming = current_buffer_incoming;
      const uint32_t spacing = duration_from_to(buffers[buffer_incoming].last_rise_micros, now);
      const uint8_t prev_reception_stage = buffers[buffer_incoming].reception_stage;
      uint8_t next_buffer_incoming = buffer_incoming;
      uint8_t next_reception_stage;
      if (spacing >= MIN_PACKET_SPACING) {
        if (prev_reception_stage > IGNORED) {
          next_buffer_incoming = increment_modulo(buffer_incoming, RECEPTION_BUFFERS);
          if (buffers[next_buffer_incoming].reception_stage != IDLE) { // not marked as seen
            EventLogger::println(MISSED_PACKET, "Packet not timely processed by main loop");
          }
        }
        next_reception_stage = DELIMITED;
      } else if (prev_reception_stage == IDLE) {
        next_reception_stage = IDLE;
      } else if (prev_reception_stage < FINISHED) {
        next_reception_stage = prev_reception_stage + 1;
        buffers[next_buffer_incoming].peak_32micros[next_reception_stage - RECORDING] = uint8_t(spacing / 32);
      } else {
        EventLogger::println(EXCESS_TOTAL_PEAKS, "Too many peaks in a packet");
        buffers[buffer_incoming].dump(now);
        next_reception_stage = IDLE;
      }

      current_buffer_incoming = next_buffer_incoming;
      buffers[next_buffer_incoming].reception_stage = next_reception_stage;
      buffers[next_buffer_incoming].last_rise_micros = now;
    }

    bool is_alive() {
      noInterrupts();
      const uint32_t last_peak_micros = buffers[current_buffer_incoming].last_rise_micros;
      interrupts();
      if (last_probed_micros != last_peak_micros) {
        last_probed_micros = last_peak_micros;
        return true;
      } else {
        return false;
      }
    }

  private:
    bool process_buffer(Reception const& buffer, uint32_t now) {
      buffer.dump(now);
      const uint32_t bits_received = buffer.decode();
      if (bits_received != VOID_BITS) {
        if (bits_received != train_handled) { // not just a repeat in the same train
          train_handled = bits_received;
          train_established_micros = buffer.last_rise_micros;
          return true;
        }
      }
      return false;
    }


  public:
    uint32_t receive() {
      const uint32_t now = micros();
      noInterrupts();
      while (buffer_receiving != current_buffer_incoming) {
        interrupts();
        Reception& buffer = buffers[buffer_receiving];
        bool new_train = false;
        if (buffer.reception_stage == FINISHED) {
          new_train = process_buffer(buffer, now);
        } else if (buffer.reception_stage >= FINISHED - 2) {
          EventLogger::println(MISSING_SOME_PEAKS, "Missing some peaks");
        } else {
          EventLogger::print(SPURIOUS_PEAKS, "Invalid peak count ");
          EventLogger::println(SPURIOUS_PEAKS, buffer.reception_stage);
        }
        buffer.reception_stage = IDLE; // mark as seen
        if (new_train) {
          return train_handled;
        }
        buffer_receiving = increment_modulo(buffer_receiving, RECEPTION_BUFFERS);
        noInterrupts();
      }

      Reception& buffer = buffers[buffer_receiving];
      const bool buffer_is_finished = buffer.reception_stage == FINISHED && duration_from_to(buffer.last_rise_micros, now) > PARITY_TIMEOUT;
      if (buffer_is_finished) {
        finish_packet(buffer_receiving);
      }
      interrupts();
      if (buffer_is_finished) {
        const bool new_train = process_buffer(buffer, now);
        buffer.reception_stage = IDLE; // mark as seen
        if (new_train) {
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
