#include "TruncatingVector.h"

// Positive even if microseconds have rolled around.
inline uint32_t duration_from_to(uint32_t early, uint32_t later) {
  return later - early;
}

template <uint8_t BUFFERS, uint8_t MIN_GAPS, uint8_t REQUIRED_GAPS, uint8_t TIME_SCALING, uint32_t PACKET_GAP_TIMEOUT, uint32_t PACKET_FINAL_TIMEOUT>
class GapTracker {
  public:
    using Width = uint8_t;
    using Buffer = TruncatingVector<uint8_t, REQUIRED_GAPS, Width>;

  private:
    Buffer buffers[BUFFERS];
    uint8_t buffer_incoming;
    uint8_t buffer_outgoing;
    uint32_t last_rise_micros[BUFFERS];
    uint32_t last_probed_micros;

    static uint8_t next_buffer(uint8_t b) {
      return ++b < BUFFERS ? b : 0;
    }

    bool finalize_buffer_offline(uint32_t now) {
      if (buffer_outgoing != buffer_incoming) {
        return true;
      }
      if (buffers[buffer_outgoing].size() == REQUIRED_GAPS) {
        const int32_t last = last_rise_micros[buffer_outgoing];
        if (duration_from_to(last, now) >= PACKET_FINAL_TIMEOUT) {
          buffer_incoming = next_buffer(buffer_incoming);
          buffers[buffer_incoming].reset();
          last_rise_micros[buffer_incoming] = last;
          return true;
        }
      }
      return false;
    }

  public:
    void setup(uint32_t now) {
      buffer_incoming = 0;
      buffer_outgoing = 0;
      buffers[buffer_incoming].reset();
      last_rise_micros[buffer_incoming] = now;
    }

    bool handle_rise() {
      bool keeping_up = true;
      const uint32_t now = micros();
      const uint32_t preceding_gap_duration = duration_from_to(last_rise_micros[buffer_incoming], now);
      if (preceding_gap_duration >= PACKET_GAP_TIMEOUT) {
        if (buffers[buffer_incoming].size() >= MIN_GAPS) {
          buffer_incoming = next_buffer(buffer_incoming);
          keeping_up = (buffer_incoming != buffer_outgoing);
        }
        buffers[buffer_incoming].reset();
      } else {
        static_assert(((PACKET_GAP_TIMEOUT - 1) >> TIME_SCALING) < 0x100);
        const uint8_t scaled_gap_width = preceding_gap_duration >> TIME_SCALING;
        buffers[buffer_incoming].push_back(scaled_gap_width);
      }
      last_rise_micros[buffer_incoming] = now;
      return keeping_up;
    }

    template <typename Receive>
    bool receive_buffer(uint32_t now, Receive receive) {
      noInterrupts();
      const bool ready = finalize_buffer_offline(now);
      interrupts();
      if (ready) {
        receive(buffers[buffer_outgoing], last_rise_micros[buffer_outgoing]);
        buffer_outgoing = next_buffer(buffer_outgoing);
      }
      return ready;
    }

    bool has_been_alive() {
      noInterrupts();
      const uint32_t last = last_rise_micros[buffer_incoming];
      interrupts();
      if (last_probed_micros != last) {
        last_probed_micros = last;
        return true;
      } else {
        return false;
      }
    }
};
