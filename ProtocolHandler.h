#define MEASUREMENT 0

static const unsigned long VOID_BITS = ~0ul;

template <typename MajorEventLogger, typename MinorEventLogger>
class ProtocolHandler {
    static const unsigned long MIN_PEAK_SPACING = 416;
    static const unsigned long MAX_PEAK_SPACING = 608;
    static const unsigned long MIN_VALE_SPACING = 1488;
    static const unsigned long MIN_PACKET_SPACING = 9984;
    static const unsigned long MIN_PACKET_PREAMBLE = 2720;
    static const unsigned long MAX_PACKET_PREAMBLE = 3200;
    static const unsigned long PARITY_TIMEOUT = 3ul * MAX_PEAK_SPACING;
    static const unsigned long TRAIN_TIMEOUT = 0x30000;

    unsigned long last_rise_micros = micros();
    enum { IDLE = 0xFF, DELIMITED = 0XFE, OPENED = 0, FINISHED = 32 };
    byte reception_stage;
    byte extra_peaks_received;
    unsigned long bits_received;
    unsigned long packet_complete_micros;
    unsigned long train_handled = VOID_BITS;
#if MEASUREMENT
    unsigned long delimiter_micros;
    unsigned long times[FINISHED + 2];
#endif

    void abort_packet_train() {
      train_handled = VOID_BITS;
      reception_stage = IDLE;
    }

    void cancel_packet() {
      reception_stage = IDLE;
    }

    void packet_delimited() {
      if (reception_stage > OPENED && reception_stage < DELIMITED) {
        if (reception_stage != FINISHED) {
          if (reception_stage > 1) {
            MinorEventLogger::print("Invalid vale count ");
            MinorEventLogger::println(reception_stage);
          }
        } else if (extra_peaks_received != (bits_received & 1)) {
          MajorEventLogger::print("Invalid final peak count ");
          MajorEventLogger::println(1 + extra_peaks_received);
        } else if (train_handled == VOID_BITS) {
          MajorEventLogger::println("Packet received but missed by main loop");
        } else if (train_handled != bits_received) {
          MajorEventLogger::print(train_handled, BIN);
          MajorEventLogger::println(" received, but then");
          MajorEventLogger::print(bits_received, BIN);
          MajorEventLogger::println(" received!");
        }
      }
    }

  public:
    void handleRise() {
      const unsigned long now = micros();
      const unsigned long duration = now - last_rise_micros;
      last_rise_micros = now;

      if (duration >= MIN_PACKET_SPACING) {
        packet_delimited();
#      if MEASUREMENT
        delimiter_micros = duration;
        times[0] = now;
#      endif
        reception_stage = DELIMITED;
      } else if (reception_stage == IDLE) {
        // wait for delimiter
      } else if (reception_stage == DELIMITED) {
        if (duration < MIN_PACKET_PREAMBLE) {
          MinorEventLogger::print(duration);
          MinorEventLogger::println("µs short preamble after delimiter");
          cancel_packet();
          return;
        }
        if (duration > MAX_PACKET_PREAMBLE) {
          MinorEventLogger::print(duration);
          MinorEventLogger::println("µs long preamble after delimiter");
          cancel_packet();
          return;
        }
#      if MEASUREMENT
        if (train_handled == VOID_BITS) {
          times[1] = now;
        }
#      endif
        reception_stage = OPENED;
        extra_peaks_received = 0;
        bits_received = 0;
      } else {
        if (duration < MIN_PEAK_SPACING) {
          MinorEventLogger::print(duration);
          MinorEventLogger::print("µs peak in vale #");
          MinorEventLogger::println(reception_stage);
          cancel_packet();
          return;
        }
        if (duration <= MAX_PEAK_SPACING) {
          ++extra_peaks_received;
        } else {
          if (duration < MIN_VALE_SPACING) {
            MinorEventLogger::print(duration);
            MinorEventLogger::print("µs vale after vale #");
            MinorEventLogger::println(reception_stage);
            cancel_packet();
            return;
          }
          const byte bit = 1 + (bits_received & 1) - extra_peaks_received;
          if (bit > 1) {
            MinorEventLogger::print("Invalid peak count ");
            MinorEventLogger::println(1 + extra_peaks_received);
            cancel_packet();
            return;
          }
          bits_received = (bits_received << 1) | bit;
          extra_peaks_received = 0;
          ++reception_stage;
#        if MEASUREMENT
          if (reception_stage <= FINISHED) {
            times[1 + reception_stage] = now;
          }
#        endif
          if (reception_stage == FINISHED && train_handled == VOID_BITS) {
            packet_complete_micros = now + PARITY_TIMEOUT;
          }
        }
      }
    }

    unsigned long receive() {
      noInterrupts();
      const bool has_old = train_handled != VOID_BITS;
      const bool has_new = reception_stage == FINISHED
                           && extra_peaks_received == byte(bits_received & 1)
                           && bits_received != train_handled;
      if (has_old || has_new) {
        const unsigned long packet_complete = micros() - packet_complete_micros;
        if ((packet_complete & 0x8000) == 0) {
          // Otherwise packet_complete_micros is in the future (regardless of overflow)
          if (has_new) {
            train_handled = bits_received;
            reception_stage = IDLE;
#          if MEASUREMENT
            const unsigned long now = micros();
            Serial.print("After ");
            Serial.print(delimiter_micros);
            Serial.println("µs delimiter");
            for (byte i = 1; i < FINISHED + 2; ++i) {
              Serial.print("  ");
              Serial.print(times[i]);
              Serial.print(" start of bit ");
              Serial.println(i);
            }
            Serial.print("  ");
            Serial.print(now);
            Serial.println(" posted");
            Serial.print("  ");
            Serial.print(micros());
            Serial.println(" finishing this debug output");
#          endif
            interrupts();
            return train_handled;
          } else if (packet_complete > TRAIN_TIMEOUT) {
            abort_packet_train();
            interrupts();
            MinorEventLogger::println("Stop expecting rest of packet train");
            return VOID_BITS;
          }
        }
      }
      interrupts();
      return VOID_BITS;
    }
};
