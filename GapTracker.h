#include "Optional.h"

// Positive even if microseconds have rolled around.
inline uint32_t duration_from_to(uint32_t early, uint32_t later) {
  return later - early;
}

template<uint8_t BUFFERS, uint8_t MIN_VIABLE_GAPS, uint8_t REQUIRED_GAPS, uint8_t TIME_SCALING, uint16_t PACKET_GAP_TIMEOUT, uint32_t PACKET_FINAL_TIMEOUT>
class GapTracker {
public:
  using Width = uint8_t;
  struct Buffer {
    uint32_t last_interrupt_micros;
    Width gap_widths[REQUIRED_GAPS];
    uint8_t gaps_seen;
  };

private:
  Buffer buffers[BUFFERS];
  uint8_t buffer_incoming = 0;
  uint8_t buffer_outgoing = 0;
  struct Flags {
    bool first_interrupt_seen : 1;
    bool alive : 1;
    Flags()
      : first_interrupt_seen(false), alive(false) {}
  } flags;

  static uint8_t next_buffer(uint8_t b) {
    // less opcode than any single expression I could think of
    ++b;
    return b < BUFFERS ? b : 0;
  }

  bool finalize_buffer_offline(uint32_t now) {
    // offline = interrupts are disabled
    if (buffer_outgoing != buffer_incoming) {
      return true;
    }
    Buffer const& buffer = buffers[buffer_incoming];
    if (buffer.gaps_seen == REQUIRED_GAPS) {
      const int32_t last = buffer.last_interrupt_micros;
      if (duration_from_to(last, now) >= PACKET_FINAL_TIMEOUT) {
        // Revert to initial state in a new buffer.
        buffer_incoming = next_buffer(buffer_incoming);
        flags.first_interrupt_seen = false;
        return true;
      }
    }
    return false;
  }

public:
  bool handle_rise() {
    // Assume we're in an interrupt handler ourselves.
    const uint32_t now = micros();
    Optional<uint8_t> preceding_gap_width;  // in ųs shifted by TIME_SCALING
    bool keeping_up = true;

    Buffer& buffer = buffers[buffer_incoming];
    if (flags.first_interrupt_seen) {
      const uint32_t gap_duration = duration_from_to(buffer.last_interrupt_micros, now);
      if (gap_duration < PACKET_GAP_TIMEOUT) {
        static_assert(((PACKET_GAP_TIMEOUT - 1) >> TIME_SCALING) < 0x100);
        preceding_gap_width = uint8_t(uint16_t(gap_duration) >> TIME_SCALING);
      } else if (buffer.gaps_seen >= MIN_VIABLE_GAPS) {
        buffer_incoming = next_buffer(buffer_incoming);
        keeping_up = (buffer_incoming != buffer_outgoing);
      }
    }
    flags.first_interrupt_seen = true;
    flags.alive = true;

    if (preceding_gap_width.has_value()) {
      if (buffer.gaps_seen < REQUIRED_GAPS) {
        buffer.gap_widths[buffer.gaps_seen] = preceding_gap_width.value();
      }
      buffer.gaps_seen += 1;
    } else {
      buffer.gaps_seen = 0;
    }
    buffer.last_interrupt_micros = now;
    return keeping_up;
  }

  template<typename Receive>
  bool receive_buffer(uint32_t now, Receive receive) {
    noInterrupts();
    const bool ready = finalize_buffer_offline(now);
    interrupts();
    if (ready) {
      receive(buffers[buffer_outgoing]);
      buffer_outgoing = next_buffer(buffer_outgoing);
    }
    return ready;
  }

  bool has_been_alive() {
    noInterrupts();
    const bool alive = flags.alive;
    flags.alive = false;
    interrupts();
    return alive;
  }
};
