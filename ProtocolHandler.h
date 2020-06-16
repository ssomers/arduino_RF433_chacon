#include "PeakBufferPool.h"

enum ProtocolNotice : uint8_t { END_OF_TRAIN,
                                MISSING_N_PEAKS = 1, MISSING_2_PEAKS = 2, MISSING_1_PEAK = 3, EXCESS_PEAKS = 4,
                                INVALID_PREAMBLE = 5,
                                WRONG_PEAK_SPACING = 6, WRONG_ADJACENT_PEAK_COUNT = 7,
                                MISSING_BITS = 8, EXCESS_BITS = 9,
                                WRONG_PARITY = 10
                              };

static const uint8_t BUFFERS = 4;
static const uint8_t PEAKS = 65; // internal peaks (or the gaps leading up to them), excluding the delimiter peak
static const uint8_t SCALING = 32; // enough for spacing to max out on delimiters

static const uint32_t TRAIN_TIMEOUT = 0x50000;

static const uint32_t VOID_BITS = ~0ul;

class ProtocolHandler {
    static const uint8_t MIN_ADJACENT_PEAK_SPACING = 10;
    static const uint8_t MAX_ADJACENT_PEAK_SPACING = 20;
    static const uint8_t MIN_SEPARATE_PEAK_SPACING = 40;
    static const uint8_t PACKET_FINAL_TIMEOUT      = 60;
    static const uint8_t MIN_PACKET_PREAMBLE       = 80;
    static const uint8_t MAX_PACKET_PREAMBLE      = 100;

    struct Train {
      uint32_t bits_received;
      uint32_t time_received; // in µs
    };

    PeakBufferPool<BUFFERS, PEAKS, SCALING> peak_buffer_pool;
    Train train_handled;

    static void dump(PeakArray<PEAKS, uint8_t> const& buffer, uint32_t time_received, uint32_t now) {
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
    static uint32_t decode(PeakArray<PEAKS, uint8_t> const& buffer) {
      const uint8_t peak_count = buffer.counted();
      if (peak_count > PEAKS) {
        EventLogger::print(EXCESS_PEAKS, peak_count);
        EventLogger::println(EXCESS_PEAKS, " peaks in a packet");
        return VOID_BITS;
      }
      if (peak_count < PEAKS) {
        const ProtocolNotice notice = peak_count == PEAKS - 1 ? MISSING_1_PEAK :
                                      peak_count == PEAKS - 2 ? MISSING_2_PEAKS :
                                      MISSING_N_PEAKS;
        EventLogger::print(notice, peak_count);
        EventLogger::println(notice, " peaks in a packet");
        return VOID_BITS;
      }

      const uint8_t preamble = buffer.value(0);
      if (preamble < MIN_PACKET_PREAMBLE || preamble > MAX_PACKET_PREAMBLE) {
        EventLogger::print(INVALID_PREAMBLE, preamble);
        EventLogger::println(INVALID_PREAMBLE, "µs preamble after delimiter");
        return VOID_BITS;
      }

      uint8_t extra_adjacent_peaks = 0;
      uint8_t bitcount = 0;
      uint8_t spacing_errors = 0;
      uint8_t bit_errors = 0;
      uint32_t bits_received = 0;
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
        EventLogger::println(WRONG_PEAK_SPACING, "Peak spacing wildly out of whack");
        return VOID_BITS;
      }
      if (bit_errors) {
        EventLogger::println(WRONG_ADJACENT_PEAK_COUNT, "Wrong number of adjacent peaks");
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

  public:
    void setup() {
      peak_buffer_pool.setup();
      train_handled.bits_received = VOID_BITS;
    }

    bool handle_rise() {
      return peak_buffer_pool.handle_rise();
    }

    bool has_been_alive() {
      return peak_buffer_pool.has_been_alive();
    }

    template <typename EventLogger, bool logTiming>
    uint32_t receive() {
      const uint32_t now = micros();
      Train train_received;
      auto receive = [&train_received, now] (PeakArray<PEAKS, uint8_t> const& buffer, uint32_t time_received) {
        if (logTiming) {
          dump(buffer, time_received, now);
        }
        train_received.bits_received = decode<EventLogger>(buffer);
        train_received.time_received = time_received;
      };

      while (peak_buffer_pool.receive_buffer(now, PACKET_FINAL_TIMEOUT, receive)) {
        if (train_received.bits_received != VOID_BITS) {
          if (train_received.bits_received != train_handled.bits_received) { // not just a repeat in the same train
            train_handled = train_received;
            return train_handled.bits_received;
          }
        }
      }
      if (train_handled.bits_received != VOID_BITS) {
        if (duration_from_to(train_handled.time_received, now) > TRAIN_TIMEOUT) {
          train_handled.bits_received = VOID_BITS;
          EventLogger::println(END_OF_TRAIN, "Stop expecting rest of packet train");
        }
      }
      return VOID_BITS;
    }
};
