#include "PeakBuffer.h"

template <typename EventLogger>
class PeakBufferPool {
    static const uint8_t RECEPTION_BUFFERS = 4;
    uint8_t buffer_incoming = 0;
    uint8_t buffer_outgoing = 0;
    PeakBuffer<EventLogger> buffers[RECEPTION_BUFFERS];

    static uint8_t next_buffer(uint8_t b) {
      return b + 1 < RECEPTION_BUFFERS ? b + 1 : 0;
    }

  public:
    PeakBufferPool() {
      buffers[0].initialize_at_startup();
    }

    void handle_rise() {
      const uint32_t now = micros();
      const uint8_t preceding32micros = buffers[buffer_incoming].preceding32micros(now);
      if (preceding32micros == 0xFF) {
        if (buffers[buffer_incoming].has_almost_seen_packet()) {
          buffer_incoming = next_buffer(buffer_incoming);
          if (buffer_incoming == buffer_outgoing) {
            EventLogger::println(MISSED_PACKET, "Packet not timely processed by main loop");
          }
        }
        buffers[buffer_incoming].initialize_delimited(now);
      } else {
        buffers[buffer_incoming].register_peak(now, preceding32micros);
      }
    }

    PeakBuffer<EventLogger>* finalize_buffer(uint32_t micros) {
      noInterrupts();
      if (buffer_outgoing != buffer_incoming) {
        interrupts();
        return &buffers[buffer_outgoing];
      } else if (buffers[buffer_incoming].appears_final_at(micros)) {
        const uint8_t next_buffer_incoming = next_buffer(buffer_incoming);
        buffers[next_buffer_incoming].initialize_from(buffers[buffer_incoming]);
        buffer_incoming = next_buffer_incoming;
        interrupts();
        return &buffers[buffer_outgoing];
      } else {
        interrupts();
        return NULL;
      }
    }

    void mark_as_received() {
      buffer_outgoing = next_buffer(buffer_outgoing);
    }

    uint32_t probe_last_peak_micros() const {
      noInterrupts();
      const uint32_t result = buffers[buffer_incoming].probe_last_peak_micros();
      interrupts();
      return result;
    }
};
