#include "Optional.h"

// Positive even if microseconds have rolled around, assuming that the duration is less than 35 minutes.
inline uint32_t duration_from_to(uint32_t early, uint32_t later) {
  return later - early;
}

enum class HandlingError {
  None,
  RanOutOfBuffers,
};

template<uint8_t BUFFERS, uint8_t MIN_VIABLE_GAPS, uint8_t REQUIRED_GAPS, uint8_t TIME_SCALING, uint16_t PACKET_GAP_TIMEOUT, uint32_t PACKET_FINAL_TIMEOUT>
class GapTracker {
public:
  class GapWidth {
  public:
    GapWidth() = default;  // not initialized!
    ~GapWidth() = default;
    GapWidth(GapWidth const&) = default;
    GapWidth(GapWidth&&) = default;
    GapWidth& operator=(GapWidth const&) = default;
    GapWidth& operator=(GapWidth&&) = default;

    // Establishes the number of slots spanning the duration in ųs,
    // provided the duration is less than PACKET_GAP_TIMEOUT.
    bool try_assign(uint32_t gap_duration) {
      if (gap_duration < PACKET_GAP_TIMEOUT) {
        static_assert(scale(PACKET_GAP_TIMEOUT - 1) <= 0xFF);
        slots = scale(gap_duration);
        return true;
      } else {
        return false;
      }
    }

    uint8_t raw() const {
      return slots;
    }

  private:
    static constexpr uint16_t scale(uint16_t micros) {
      return micros >> TIME_SCALING;
    }

    uint8_t slots;
  };

  struct Buffer {
    uint32_t last_interrupt_micros;
    GapWidth gap_widths[REQUIRED_GAPS];
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
    // Generates less opcode than any single expression I could think of.
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
      if (duration_from_to(buffer.last_interrupt_micros, now) >= PACKET_FINAL_TIMEOUT) {
        // Revert to initial state in a new buffer.
        buffer_incoming = next_buffer(buffer_incoming);
        flags.first_interrupt_seen = false;
        return true;
      }
    }
    return false;
  }

public:
  // To be called from an interrupt handler, so assuming no other interrupts can happen.
  // And obviously returns quickly.
  // Returns whether timing looks all right, i.e.:
  // - that that we didn't fill all the buffers before receive_buffer got a chance to handle them.
  HandlingError handle_rise() {
    const uint32_t now = micros();
    HandlingError error = HandlingError::None;
    uint8_t gaps_seen = 0;
    Buffer& buffer = buffers[buffer_incoming];

    if (flags.first_interrupt_seen) {
      gaps_seen = buffer.gaps_seen;
      const uint32_t gap_duration = duration_from_to(buffer.last_interrupt_micros, now);
      GapWidth preceding_gap_width;
      if (preceding_gap_width.try_assign(gap_duration)) {
        if (gaps_seen < REQUIRED_GAPS) {
          buffer.gap_widths[gaps_seen] = preceding_gap_width;
        }
        gaps_seen += 1;
      } else {
        if (gaps_seen >= MIN_VIABLE_GAPS) {
          buffer_incoming = next_buffer(buffer_incoming);
          if (buffer_incoming == buffer_outgoing) {
            error = HandlingError::RanOutOfBuffers;
          }
        }
        gaps_seen = 0;
      }
    }
    flags.first_interrupt_seen = true;
    flags.alive = true;
    buffer.gaps_seen = gaps_seen;
    buffer.last_interrupt_micros = now;
    return error;
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
