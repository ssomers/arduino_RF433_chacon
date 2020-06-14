#include "PeakArray.h"

inline uint32_t duration_from_to(uint32_t early, uint32_t later) {
  return later - early;
}

template <uint8_t BUFFERS, uint8_t PEAKS, uint8_t SCALING, uint8_t MAX_SPACING = 0xFF>
class PeakBufferPool {
    uint8_t buffer_incoming = 0;
    uint8_t buffer_outgoing = 0;
    PeakArray<PEAKS, uint8_t> buffers[BUFFERS];
    uint32_t last_peak_micros[BUFFERS];
    uint32_t last_probed_micros;

    static uint8_t next_buffer(uint8_t b) {
      return b + 1 < BUFFERS ? b + 1 : 0;
    }

    bool finalize_buffer_offline(uint32_t now, uint8_t timeout) {
      PeakArray<PEAKS, uint8_t>& buffer = buffers[buffer_outgoing];
      if (buffer_outgoing != buffer_incoming) {
        return true;
      }
      if (buffer.counted() == PEAKS) {
        if (duration_from_to(last_peak_micros[buffer_outgoing], now) / SCALING >= timeout) {
          buffer_incoming = next_buffer(buffer_incoming);
          buffers[buffer_incoming].initialize_idle();
          last_peak_micros[buffer_incoming] = last_peak_micros[buffer_outgoing];
          return true;
        }
      }
      return false;
    }

  public:
    PeakBufferPool() {
      buffers[0].initialize_idle();
      last_peak_micros[0] = micros();
    }

    bool handle_rise() {
      bool keeping_up = true;
      const uint32_t now = micros();
      const uint32_t preceding_spacing = duration_from_to(last_peak_micros[buffer_incoming], now) / SCALING;
      if (preceding_spacing > MAX_SPACING) {
        if (buffers[buffer_incoming].is_active() && buffers[buffer_incoming].counted() > PEAKS / 2) {
          buffer_incoming = next_buffer(buffer_incoming);
          keeping_up = (buffer_incoming != buffer_outgoing);
        }
        buffers[buffer_incoming].initialize_active();
      } else {
        buffers[buffer_incoming].append(preceding_spacing);
      }
      last_peak_micros[buffer_incoming] = now;
      return keeping_up;
    }

    template <typename Receive>
    bool finalize_buffer(uint32_t now, uint8_t timeout, Receive receive) {
      noInterrupts();
      const bool ready = finalize_buffer_offline(now, timeout);
      interrupts();
      if (ready) {
        receive(buffers[buffer_outgoing], last_peak_micros[buffer_outgoing]);
        buffer_outgoing = next_buffer(buffer_outgoing);
      }
      return ready;
    }

    bool has_been_alive() {
      noInterrupts();
      const uint32_t last = last_peak_micros[buffer_incoming];
      interrupts();
      if (last_probed_micros != last) {
        last_probed_micros = last;
        return true;
      } else {
        return false;
      }
    }
};
