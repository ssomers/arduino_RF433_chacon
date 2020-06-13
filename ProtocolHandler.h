#include "PeakBufferPool.h"

static const uint32_t TRAIN_TIMEOUT = 0x50000;
static const uint32_t VOID_BITS = ~0ul;

template <typename EventLogger, bool logTiming>
class ProtocolHandler {
    typedef PeakBufferPool<EventLogger> Core;
    Core peak_buffer_pool;
    uint32_t train_handled = VOID_BITS;
    uint32_t train_established_micros;
    uint32_t last_probed_micros;

  private:
    enum ReceptionSummary { NOTHING, NOISE, NEWS };
    ReceptionSummary catch_up_one(const uint32_t now, Reception& news) {
      if (PeakBuffer<EventLogger>* const buffer = peak_buffer_pool.finalize_buffer(now)) {
        if (logTiming) {
          buffer->dump(now);
        }
        const bool success = buffer->decode(news);
        peak_buffer_pool.mark_as_received();
        return success ? NEWS : NOISE;
      } else {
        return NOTHING;
      }
    }

  public:
    void handle_rise() {
      peak_buffer_pool.handle_rise();
    }

    bool has_been_alive() {
      const uint32_t last_rise_micros = peak_buffer_pool.probe_last_rise_micros();
      if (last_probed_micros != last_rise_micros) {
        last_probed_micros = last_rise_micros;
        return true;
      } else {
        return false;
      }
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
