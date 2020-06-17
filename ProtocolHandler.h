#include "PeakBufferPool.h"

enum ProtocolNotice : uint8_t { NONE,
                                MISSING_N_PEAKS = 1,
                                MISSING_2_PEAKS = 2,
                                MISSING_1_PEAK =  5,
                                EXCESS_PEAKS = 6,
                                INVALID_PREAMBLE = 7,
                                WRONG_PEAK_SPACING = 8, WRONG_ADJACENT_PEAK_COUNT = 9,
                                WRONG_BIT_COUNT = 10, WRONG_PARITY = 11
                              };

static const uint8_t BUFFERS = 4;
static const uint8_t PEAKS = 65; // internal peaks (or the gaps leading up to them), excluding the delimiter peak
static const uint8_t SCALING = 32; // granularity in µs of duration measurements between peaks
static const uint8_t MAX_SPACING = 0xFF; // anything higher implies a delimiter, like 0x100 × 32 µs = 8192 µs

static const uint32_t TRAIN_TIMEOUT = 360000; // in µs

class ProtocolHandler {
    static const uint8_t MIN_ADJACENT_PEAK_SPACING = 10; // in SCALING µs
    static const uint8_t MAX_ADJACENT_PEAK_SPACING = 20; // in SCALING µs
    static const uint8_t MIN_SEPARATE_PEAK_SPACING = 40; // in SCALING µs
    static const uint8_t PACKET_FINAL_TIMEOUT      = 60; // in SCALING µs
    static const uint8_t MIN_PREAMBLE              = 80; // in SCALING µs
    static const uint8_t MAX_PREAMBLE             = 100; // in SCALING µs

    using Peaks = PeakBufferPool<BUFFERS, PEAKS, SCALING, PACKET_FINAL_TIMEOUT, MAX_SPACING>;
    Peaks peaks;

    class StateHandler {
        bool has_last_bits_handled;
        uint32_t last_bits_handled; // only valid if has_last_bits_handled
        bool has_last_time;
        uint32_t last_time; // only usable if has_last_time

      public:
        void setup(uint32_t now) {
          has_last_bits_handled = false;
          has_last_time = true;
          last_time = now;
        }

        // Whether we should make a fuss about errors:
        // - not right after booting because we may very well tune in in the middle of broadcast,
        // - not right after receiving a packet because our reaction deteriorates the reception quality
        //   of the rest of the packet train.
        bool is_good_weather(uint32_t time_received) const {
          return !has_last_time || duration_from_to(last_time, time_received) >= TRAIN_TIMEOUT;
        }

        bool handle(uint32_t bits_received, uint32_t time_received) {
          if (has_last_bits_handled && has_last_time
              && last_bits_handled == bits_received
              && duration_from_to(last_time, time_received) < TRAIN_TIMEOUT) {
            return false; // looks like a repeat packet in the same train
          } else {
            has_last_bits_handled = true;
            last_bits_handled = bits_received;
            has_last_time = true;
            last_time = time_received;
            return true;
          }
        }

        void catch_up(uint32_t now) {
          if (duration_from_to(last_time, now) >= (1ul << 30)) {
            // Invalidate last_time because some future invocation of receive might happen when
            // micros() has come around to last_time again. If has_last_time was already false,
            // we compared last_time ourselves, but the outcome makes no difference then.
            has_last_time = false;
          }
        }
    } state;

    static void dump(Peaks::Buffer const& buffer, uint32_t time_received, uint32_t now) {
      const uint8_t peak_count = buffer.counted();
      Serial.print(peak_count);
      Serial.println(" peaks, timing:");
      for (uint8_t p = 0; p < min(PEAKS, peak_count); ++p) {
        Serial.print("  -");
        Serial.print(buffer.value(p));
        Serial.print("µs preceding peak ");
        Serial.println(p);
      }
      Serial.print("  ");
      Serial.print(time_received);
      Serial.println(" last peak");
      Serial.print("  ");
      Serial.print(now);
      Serial.println(" receiving");
      Serial.print("  ");
      Serial.print(micros());
      Serial.println(" finishing this debug output");
    }

    template <typename EventLogger>
    static bool decode(Peaks::Buffer const& buffer, bool with_conviction, uint32_t& bits_received) {
      const uint8_t peak_count = buffer.counted();
      if (peak_count != PEAKS) {
        const auto notice = peak_count > PEAKS ? EXCESS_PEAKS :
                            peak_count == PEAKS - 1 ? MISSING_1_PEAK :
                            peak_count == PEAKS - 2 ? MISSING_2_PEAKS :
                            MISSING_N_PEAKS;
        EventLogger::print(with_conviction * notice, peak_count);
        EventLogger::println(with_conviction * notice, " peaks in a packet");
        return false;
      }

      const uint8_t preamble = buffer.value(0);
      if (preamble < MIN_PREAMBLE || preamble > MAX_PREAMBLE) {
        EventLogger::print(with_conviction * INVALID_PREAMBLE, preamble);
        EventLogger::println(with_conviction * INVALID_PREAMBLE, "µs preamble after delimiter");
        return false;
      }

      uint8_t extra_adjacent_peaks = 0;
      uint8_t bitcount = 0;
      uint8_t spacing_errors = 0;
      uint8_t bit_errors = 0;
      bits_received = 0;
      for (uint8_t p = 1; p < PEAKS; ++p) {
        const uint8_t preceding_spacing = buffer.value(p);
        if (preceding_spacing < MIN_SEPARATE_PEAK_SPACING) {
          spacing_errors += (preceding_spacing < MIN_ADJACENT_PEAK_SPACING);
          spacing_errors += (preceding_spacing > MAX_ADJACENT_PEAK_SPACING);
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
        EventLogger::println(with_conviction * WRONG_PEAK_SPACING, "Peak spacing wildly out of whack");
        return false;
      }
      if (bit_errors) {
        EventLogger::println(with_conviction * WRONG_ADJACENT_PEAK_COUNT, "Wrong number of adjacent peaks");
        return false;
      }
      if (bitcount != 32) {
        EventLogger::print(with_conviction * WRONG_BIT_COUNT, "#bits=");
        EventLogger::println(with_conviction * WRONG_BIT_COUNT, bitcount);
        return false;
      }
      if (extra_adjacent_peaks != (bits_received & 1)) {
        EventLogger::print(with_conviction * WRONG_PARITY, "Incorrect #parity peaks ");
        EventLogger::println(with_conviction * WRONG_PARITY, 1 + extra_adjacent_peaks);
        return false;
      }
      return true;
    }

  public:
    void setup() {
      const uint32_t now = micros();
      peaks.setup(now);
      state.setup(now);
    }

    bool handle_rise() {
      return peaks.handle_rise();
    }

    bool has_been_alive() {
      return peaks.has_been_alive();
    }

    template <typename EventLogger, bool logTiming>
    bool receive(uint32_t& bits_received) {
      const uint32_t now = micros();
      bool new_was_decodable;
      uint32_t new_time_received;
      auto process = [&](Peaks::Buffer const& buffer, uint32_t time_received) {
        const bool with_conviction = state.is_good_weather(time_received);
        if (logTiming) {
          dump(buffer, time_received, now);
        }
        new_was_decodable = decode<EventLogger>(buffer, with_conviction, bits_received);
        new_time_received = time_received;
      };

      while (peaks.receive_buffer(now, process)) {
        if (new_was_decodable && state.handle(bits_received, new_time_received)) {
          return true;
        }
      }

      state.catch_up(now);
      return false;
    }
};
