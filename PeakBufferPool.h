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
      const uint8_t spacing_32micros = buffers[buffer_incoming].spacing_32micros(now);
      if (spacing_32micros == 0xFF) {
        if (buffers[buffer_incoming].stage() > DELIMITED + IGNORED_WHEN_INCOMPLETE) {
          buffer_incoming = next_buffer(buffer_incoming);
          if (buffer_incoming == buffer_outgoing) {
            EventLogger::println(MISSED_PACKET, "Packet not timely processed by main loop");
          }
        }
      }
      buffers[buffer_incoming].handle_rise(now, spacing_32micros);
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

    uint32_t probe_last_rise_micros() const {
      noInterrupts();
      const uint32_t result = buffers[buffer_incoming].probe_last_rise_micros();
      interrupts();
      return result;
    }
};
