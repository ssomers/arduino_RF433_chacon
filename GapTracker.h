#include "TruncatingVector.h"
#include "Optional.h"

// Positive even if microseconds have rolled around.
inline uint32_t duration_from_to(uint32_t early, uint32_t later) {
  return later - early;
}

template<uint8_t BUFFERS, uint8_t MIN_GAPS, uint8_t REQUIRED_GAPS, uint8_t TIME_SCALING, uint16_t PACKET_GAP_TIMEOUT, uint32_t PACKET_FINAL_TIMEOUT>
class GapTracker {
public:
  using Width = uint8_t;
  using Buffer = TruncatingVector<uint8_t, REQUIRED_GAPS, Width>;

private:
  Buffer buffers[BUFFERS];
  uint32_t last_rise_micros[BUFFERS];
  uint8_t buffer_incoming = 0;
  uint8_t buffer_outgoing = 0;
  struct Flags {
    bool first_rise_seen : 1;
    bool alive : 1;
    Flags() : first_rise_seen(false), alive(false) {}
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
    if (buffers[buffer_incoming].size() == REQUIRED_GAPS) {
      const int32_t last = last_rise_micros[buffer_incoming];
      if (duration_from_to(last, now) >= PACKET_FINAL_TIMEOUT) {
        // revert to initial state
        buffer_incoming = next_buffer(buffer_incoming);
        flags.first_rise_seen = false;
        return true;
      }
    }
    return false;
  }

public:
  bool handle_rise() {
    // assume we're in an interrupt handler ourselves
    const uint32_t now = micros();
    Optional<uint8_t> preceding_gap_width;  // in ųs shifted by TIME_SCALING
    bool keeping_up = true;

    if (flags.first_rise_seen) {
      const uint32_t duration = duration_from_to(last_rise_micros[buffer_incoming], now);
      if (duration < PACKET_GAP_TIMEOUT) {
        static_assert(((PACKET_GAP_TIMEOUT - 1) >> TIME_SCALING) < 0x100);
        preceding_gap_width = uint8_t(uint16_t(duration) >> TIME_SCALING);
      } else {
        if (buffers[buffer_incoming].size() >= MIN_GAPS) {
          buffer_incoming = next_buffer(buffer_incoming);
          keeping_up = (buffer_incoming != buffer_outgoing);
        }
      }
    }
    flags.first_rise_seen = true;
    flags.alive = true;

    if (preceding_gap_width.has_value()) {
      buffers[buffer_incoming].push_back(preceding_gap_width.value());
    } else {
      buffers[buffer_incoming].reset();
    }
    last_rise_micros[buffer_incoming] = now;
    return keeping_up;
  }

  template<typename Receive>
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
    const bool alive = flags.alive;
    flags.alive = false;
    interrupts();
    return alive;
  }
};
