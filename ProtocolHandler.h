#include "PeakHandler.h"

static const uint32_t TRAIN_TIMEOUT = 0x50000;
static const uint32_t VOID_BITS = ~0ul;

template <typename EventLogger, bool logTiming>
class ProtocolHandler {
    typedef PeakHandler<EventLogger> Core;
    Core peak_handler;
    uint32_t train_handled = VOID_BITS;
    uint32_t train_established_micros;
    uint8_t buffer_receiving = 0;

  private:
    enum ReceptionSummary { NOTHING, NOISE, NEWS };
    ReceptionSummary catch_up_one(const uint32_t now, Reception& news) {
      noInterrupts();
      const bool final = peak_handler.finalize_offline(buffer_receiving, now);
      interrupts();
      if (final) {
        Buffer<EventLogger>& buffer = peak_handler.access_buffer(buffer_receiving);
        if (logTiming) {
          buffer.dump(now);
        }
        const bool success = buffer.decode(news);
        buffer.mark_as_seen();
        buffer_receiving = peak_handler.next_buffer(buffer_receiving);
        return success ? NEWS : NOISE;
      } else {
        return NOTHING;
      }
    }

  public:
    void handle_rise() {
      peak_handler.handle_rise();
    }

    bool has_been_alive() {
      return peak_handler.has_been_alive();
    }

    uint32_t receive() {
      for (;;) {
        const uint32_t now = micros();
        Reception news;
        switch (catch_up_one(now, news)) {
          case NOTHING:
            if (duration_from_to(train_established_micros, now) > TRAIN_TIMEOUT) {
              train_handled = VOID_BITS;
              EventLogger::println(END_OF_TRAIN, "Stop expecting rest of packet train");
            }
            return VOID_BITS;
          case NOISE:
            continue;
          case NEWS:
            if (news.bits_received != train_handled) { // not just a repeat in the same train
              train_handled = news.bits_received;
              train_established_micros = news.time_received;
              return train_handled;
            }
            continue;
        }
      }
    }
};
