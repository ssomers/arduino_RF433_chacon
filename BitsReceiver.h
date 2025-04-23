#include "GapTracker.h"
#include "PacketTrainTracker.h"

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
  static const uint8_t MIN_VIABLE_GAPS = 60;           // number of gaps we want the tracker to consider diagnosing
  static const uint8_t TIME_SCALING = 5;               // how many bits to right-shift measured times in µs, for recording gap widths
  static const uint32_t PACKET_GAP_TIMEOUT = 0x2000;   // in µs, wider gap implies a delimiter
  static const uint32_t PACKET_FINAL_TIMEOUT = 0x800;  // in µs, earlier gap implies packet isn't yet finished

  static const uint8_t MIN_NARROW_GAP_WIDTH = 12;  // in scaled gap width
  static const uint8_t MAX_NARROW_GAP_WIDTH = 24;  // in scaled gap width
  static const uint8_t MIN_WIDE_GAP_WIDTH = 40;    // in scaled gap width
  static const uint8_t MIN_PREAMBLE = 60;          // in scaled gap width
  static const uint8_t MAX_PREAMBLE = 120;         // in scaled gap width

  using MyGapTracker = GapTracker<BUFFERS, MIN_VIABLE_GAPS, REQUIRED_GAPS, TIME_SCALING, PACKET_GAP_TIMEOUT, PACKET_FINAL_TIMEOUT>;
  using GapBuffer = MyGapTracker::Buffer;
  MyGapTracker gap_tracker;
  PacketTrainTracker<TRAIN_TIMEOUT> packet_train_tracker;

  static void dump(GapBuffer const& buffer, uint32_t time_received, uint32_t now) {
    const uint8_t gap_count = min(REQUIRED_GAPS, buffer.gaps_seen);
    Serial.print("gap widths:");
    for (uint8_t p = 0; p < gap_count; ++p) {
      if (p % 16 == 1) {
        Serial.println();
        Serial.print("  ");
      }
      Serial.print(" ");
      Serial.print(buffer.gap_widths[p].raw());
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
    const uint8_t gap_count = buffer.gaps_seen;
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

    const uint8_t preamble = buffer.gap_widths[0].raw();
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
      const uint8_t gap_width = buffer.gap_widths[p].raw();
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
  void setup(uint32_t now) {
    packet_train_tracker.setup(now);
  }

  bool handle_rise() {
    return gap_tracker.handle_rise();
  }

  bool has_been_alive() {
    return gap_tracker.has_been_alive();
  }

  template<typename EventLogger, bool logTiming>
  bool receive(uint32_t now, uint32_t& bits_received) {
    bool new_was_decodable;
    uint32_t new_time_received;
    auto process = [&](GapBuffer const& buffer) {
      const uint32_t time_received = buffer.last_interrupt_micros;
      const bool seems_legit = !packet_train_tracker.is_settling_down(time_received);
      new_was_decodable = decode<EventLogger>(buffer, seems_legit, bits_received);
      new_time_received = time_received;
      if (logTiming && seems_legit) {
        dump(buffer, time_received, now);
      }
    };

    while (gap_tracker.receive_buffer(now, process)) {
      if (new_was_decodable && packet_train_tracker.handle(bits_received, new_time_received)) {
        return true;
      }
    }

    packet_train_tracker.catch_up(now);
    return false;
  }
};
