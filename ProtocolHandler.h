#include "PeakHandler.h"

static const uint32_t TRAIN_TIMEOUT = 0x50000;

template <typename EventLogger, bool logTiming>
class ProtocolHandler : public PeakHandler<EventLogger, logTiming> {
    typedef PeakHandler<EventLogger, logTiming> Super;
    uint32_t train_handled = VOID_BITS;
    uint32_t train_established_micros;
    uint8_t buffer_receiving = 0;

  private:
    enum ReceptionSummary { NOTHING, NOISE, NEWS };
    ReceptionSummary catch_up_one(const uint32_t now, Reception& news) {
      noInterrupts();
      const bool final = Super::finalize_offline(buffer_receiving, now);
      interrupts();
      if (final) {
        typename Super::Buffer& buffer = Super::access_buffer(buffer_receiving);
        buffer.dump(now);
        bool success = false;
        if (buffer.stage() == FINISHED) {
          success = buffer.decode(news);
        } else if (buffer.stage() >= FINISHED - 2) {
          EventLogger::println(MISSING_SOME_PEAKS, "Missing some peaks");
        } else {
          EventLogger::print(SPURIOUS_PEAKS, "Invalid peak count ");
          EventLogger::println(SPURIOUS_PEAKS, buffer.stage());
        }
        buffer.mark_as_seen();
        buffer_receiving = Super::next_buffer(buffer_receiving);
        return success ? NEWS : NOISE;
      } else {
        return NOTHING;
      }
    }

  public:
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
