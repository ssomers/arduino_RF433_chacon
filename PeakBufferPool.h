#include "PeakArray.h"

inline uint32_t duration_from_to(uint32_t early, uint32_t later) {
  return later - early;
}

template <uint8_t BUFFERS, uint8_t PEAKS, uint8_t SCALING, uint8_t PACKET_FINAL_TIMEOUT, uint8_t MAX_SPACING>
class PeakBufferPool {
  public:
    using Spacing = uint8_t;
    using Buffer = PeakArray<PEAKS, Spacing>;

  private:
    uint8_t buffer_incoming;
    uint8_t buffer_outgoing;
    Buffer buffers[BUFFERS];
    uint32_t last_peak_micros[BUFFERS];
    uint32_t last_probed_micros;

    static uint8_t next_buffer(uint8_t b) {
      return ++b < BUFFERS ? b : 0;
    }

    bool finalize_buffer_offline(uint32_t now) {
      Buffer& buffer = buffers[buffer_outgoing];
      if (buffer_outgoing != buffer_incoming) {
        return true;
      }
      if (buffer.counted() == PEAKS) {
        const int32_t last = last_peak_micros[buffer_outgoing];
        if (duration_from_to(last, now) / SCALING >= PACKET_FINAL_TIMEOUT) {
          buffer_incoming = next_buffer(buffer_incoming);
          buffers[buffer_incoming].initialize();
          last_peak_micros[buffer_incoming] = last;
          return true;
        }
      }
      return false;
    }

  public:
    void setup(uint32_t now) {
      buffer_incoming = 0;
      buffer_outgoing = 0;
      buffers[buffer_incoming].initialize();
      last_peak_micros[buffer_incoming] = now;
    }

    bool handle_rise() {
      bool keeping_up = true;
      const uint32_t now = micros();
      const uint32_t preceding_spacing = duration_from_to(last_peak_micros[buffer_incoming], now) / SCALING;
      if (preceding_spacing > MAX_SPACING) {
        if (buffers[buffer_incoming].counted() > PEAKS / 2) {
          buffer_incoming = next_buffer(buffer_incoming);
          keeping_up = (buffer_incoming != buffer_outgoing);
        }
        buffers[buffer_incoming].initialize();
      } else {
        buffers[buffer_incoming].append(preceding_spacing);
      }
      last_peak_micros[buffer_incoming] = now;
      return keeping_up;
    }

    template <typename Receive>
    bool receive_buffer(uint32_t now, Receive receive) {
      noInterrupts();
      const bool ready = finalize_buffer_offline(now);
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
