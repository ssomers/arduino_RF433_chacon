#include "Optional.h"

template<uint32_t TRAIN_TIMEOUT>
class PacketTrainTracker {
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
};
