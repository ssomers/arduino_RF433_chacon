#include "GapTracker.h"
#include "Optional.h"

enum class ProtocolNotice : uint8_t {
  MISSING_N_GAPS = 1,
  MISSING_2_GAPS = 2,
  MISSING_1_GAP = 5,
  EXCESS_GAPS = 6,
  INVALID_PREAMBLE = 7,
  WRONG_PEAK_SPACING = 8,
  WRONG_ADJACENT_PEAK_COUNT = 9,
  WRONG_BIT_COUNT = 10,
  WRONG_PARITY = 11
};

class BitsReceiver {
public:
  static const uint32_t TRAIN_TIMEOUT = 0x50000;  // in µs, that makes 328 ms

private:
  static const uint8_t BUFFERS = 4;
  static const uint8_t REQUIRED_GAPS = 65;             // number of gaps between peaks forming a packet
  static const uint8_t MIN_GAPS = 60;                  // number of gaps we want the tracker to consider viable
  static const uint8_t TIME_SCALING = 5;               // how many bits to right-shift measured times in µs, for recording gap widths
  static const uint32_t PACKET_GAP_TIMEOUT = 0x2000;   // in µs, wider gap implies a delimiter
  static const uint32_t PACKET_FINAL_TIMEOUT = 0x800;  // in µs, more implies there's something following an otherwise legit packet

  static const uint8_t MIN_NARROW_GAP_WIDTH = 12;  // in scaled gap width
  static const uint8_t MAX_NARROW_GAP_WIDTH = 24;  // in scaled gap width
  static const uint8_t MIN_WIDE_GAP_WIDTH = 40;    // in scaled gap width
  static const uint8_t MIN_PREAMBLE = 60;          // in scaled gap width
  static const uint8_t MAX_PREAMBLE = 120;         // in scaled gap width

  using MyGapTracker = GapTracker<BUFFERS, MIN_GAPS, REQUIRED_GAPS, TIME_SCALING, PACKET_GAP_TIMEOUT, PACKET_FINAL_TIMEOUT>;
  using GapBuffer = MyGapTracker::Buffer;
  MyGapTracker gap_tracker;

  class PacketTrainState {
    Optional<uint32_t> last_bits_handled;
    Optional<uint32_t> last_event_time;

  public:
    void setup(uint32_t now) {
      last_bits_handled.reset();
      last_event_time = now;
    }

    // Whether we are:
    // - right after booting, when we may very well be tuning in at the middle of a broadcast;
    // - right after successfully receiving a packet in a packet train, when our response
    //   greatly deteriorates the reception quality of the rest of the packet train.
    bool is_settling_down(uint32_t time_received) const {
      return last_event_time.has_value()
             && duration_from_to(last_event_time.value(), time_received) < TRAIN_TIMEOUT;
    }

    bool handle(uint32_t bits_received, uint32_t time_received) {
      if (last_bits_handled.has_value()
          && is_settling_down(time_received) && last_bits_handled.value() == bits_received) {
        return false;  // looks like a repeat packet in the same train
      } else {
        last_bits_handled = bits_received;
        last_event_time = time_received;
        return true;
      }
    }

    void catch_up(uint32_t now) {
      // Every ~72 minutes, the time in ųs rolls over.
      // Forget any last_event_time recorded in a previous era.
      if (last_event_time.has_value() && duration_from_to(last_event_time.value(), now) & (uint32_t(2) << 31)) {
        last_event_time.reset();
      }
    }
  } state;

  static void dump(GapBuffer const& buffer, uint32_t time_received, uint32_t now) {
    const uint8_t gap_count = min(REQUIRED_GAPS, buffer.size());
    Serial.print("gap widths:");
    for (uint8_t p = 0; p < gap_count; ++p) {
      if (p % 16 == 1) {
        Serial.println();
        Serial.print("  ");
      }
      Serial.print(" ");
      Serial.print(buffer[p]);
    }
    Serial.println();
    Serial.print("  ");
    Serial.print(time_received);
    Serial.println("µs last rise");
    Serial.print("  ");
    Serial.print(now);
    Serial.println("µs started receiving");
    Serial.print("  ");
    Serial.print(micros());
    Serial.println("µs finishing this debug output");
  }

  template<typename EventLogger>
  static bool decode(GapBuffer const& buffer, bool seems_legit, uint32_t& bits_received) {
    const uint8_t gap_count = buffer.size();
    if (gap_count != REQUIRED_GAPS) {
      if (seems_legit) {
        const auto notice = gap_count > REQUIRED_GAPS        ? ProtocolNotice::EXCESS_GAPS
                            : gap_count == REQUIRED_GAPS - 1 ? ProtocolNotice::MISSING_1_GAP
                            : gap_count == REQUIRED_GAPS - 2 ? ProtocolNotice::MISSING_2_GAPS
                                                             : ProtocolNotice::MISSING_N_GAPS;
        EventLogger::print(gap_count);
        EventLogger::println(notice, " gaps in a packet");
      }
      return false;
    }

    const uint8_t preamble = buffer[0];
    if (preamble < MIN_PREAMBLE || preamble > MAX_PREAMBLE) {
      if (seems_legit) {
        EventLogger::print(preamble);
        EventLogger::println(ProtocolNotice::INVALID_PREAMBLE, "µs preamble after delimiter");
      }
      return false;
    }

    uint8_t adjacent_narrow_gaps = 0;
    uint8_t bitcount = 0;
    uint8_t spacing_errors = 0;
    uint8_t bit_errors = 0;
    bits_received = 0;
    for (uint8_t p = 1; p < REQUIRED_GAPS; ++p) {
      const uint8_t gap_width = buffer[p];
      if (gap_width < MIN_WIDE_GAP_WIDTH) {
        spacing_errors += (gap_width < MIN_NARROW_GAP_WIDTH);
        spacing_errors += (gap_width > MAX_NARROW_GAP_WIDTH);
        adjacent_narrow_gaps += 1;
      } else {
        const uint8_t bit = 1 + (bits_received & 1) - adjacent_narrow_gaps;
        bit_errors += (bit > 1);
        bits_received = (bits_received << 1) | (bit & 1);
        bitcount += 1;
        adjacent_narrow_gaps = 0;
      }
    }
    if (spacing_errors) {
      if (seems_legit) {
        EventLogger::println(ProtocolNotice::WRONG_PEAK_SPACING, "Peak spacing wildly out of whack");
      }
      return false;
    }
    if (bit_errors) {
      if (seems_legit) {
        EventLogger::println(ProtocolNotice::WRONG_ADJACENT_PEAK_COUNT, "Wrong number of adjacent peaks");
      }
      return false;
    }
    if (bitcount != 32) {
      if (seems_legit) {
        EventLogger::print("#bits=");
        EventLogger::println(ProtocolNotice::WRONG_BIT_COUNT, bitcount);
      }
      return false;
    }
    if (adjacent_narrow_gaps != (bits_received & 1)) {
      if (seems_legit) {
        EventLogger::print("Incorrect #parity gaps ");
        EventLogger::println(ProtocolNotice::WRONG_PARITY, adjacent_narrow_gaps);
      }
      return false;
    }
    return true;
  }

public:
  void setup() {
    const uint32_t now = micros();
    state.setup(now);
  }

  bool handle_rise() {
    return gap_tracker.handle_rise();
  }

  bool has_been_alive() {
    return gap_tracker.has_been_alive();
  }

  template<typename EventLogger, bool logTiming>
  bool receive(uint32_t& bits_received) {
    const uint32_t now = micros();
    bool new_was_decodable;
    uint32_t new_time_received;
    auto process = [&](GapBuffer const& buffer, uint32_t time_received) {
      const bool seems_legit = !state.is_settling_down(time_received);
      new_was_decodable = decode<EventLogger>(buffer, seems_legit, bits_received);
      new_time_received = time_received;
      if (logTiming && seems_legit) {
        dump(buffer, time_received, now);
      }
    };

    while (gap_tracker.receive_buffer(now, process)) {
      if (new_was_decodable && state.handle(bits_received, new_time_received)) {
        return true;
      }
    }

    state.catch_up(now);
    return false;
  }
};
