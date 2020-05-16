#include <EEPROM.h>

#define MEASUREMENT 0

enum class LogLevel { NONE, MAJOR, MINOR };

template <LogLevel logLevel>
class ProtocolHandler {
  static const unsigned long MIN_PEAK_SPACING = 500;
  static const unsigned long MAX_PEAK_SPACING = 600;
  static const unsigned long MAX_VALE_SPACING = 1500;
  static const unsigned long MIN_PACKET_SPACING = 10000;
  static const unsigned long MAX_PACKET_SPACING = 11000;
  static const unsigned long MIN_PACKET_PREAMBLE = 2800;
  static const unsigned long MAX_PACKET_PREAMBLE = 3200;
  static const unsigned long PARITY_TIMEOUT = 3ul * MAX_PEAK_SPACING;
  static const unsigned long MAX_PAYLOAD = 32ul * (MAX_VALE_SPACING + MAX_PEAK_SPACING);
  static const unsigned long TRAIN_TIMEOUT = 3 * MAX_PACKET_SPACING + 4 * (MAX_PACKET_PREAMBLE + MAX_PAYLOAD);

  unsigned long last_rise_micros = micros();
  enum { IDLE = -3, BOTCHED_PACKET = -2, DELIMITED = -1, OPENED = 0, FINISHED = 32 };
  int reception_stage;
  int extra_peaks_received;
  unsigned long bits_received;
  unsigned long packet_complete_micros;
  unsigned long train_handled = 0;
#if MEASUREMENT
  unsigned long delimiter_micros;
  unsigned long preamble_micros;
#endif

  void abort_packet_train() {
    digitalWrite(LED_BUILTIN, LOW);
    train_handled = 0;
    reception_stage = IDLE;
  }

  void cancel_packet() {
    if (train_handled) {
      reception_stage = BOTCHED_PACKET;
    } else {
      abort_packet_train();
    }
  }

  void packet_delimited() {
    if (logLevel >= LogLevel::MAJOR && reception_stage > 0) {
      if (reception_stage != 32) {
        if (reception_stage > 1 || logLevel >= LogLevel::MINOR) {
          Serial.print("Invalid vale count ");
          Serial.println(reception_stage);
        }
      } else if (extra_peaks_received != int(bits_received & 1)) {
        Serial.print("Invalid final peak count ");
        Serial.println(1 + extra_peaks_received);
      } else if (train_handled == 0) {
        Serial.println("Packet received but missed by main loop");
      } else if (train_handled != bits_received) {
        Serial.print(train_handled, BIN);
        Serial.println(" received, but then");
        Serial.print(bits_received, BIN);
        Serial.println(" received!");
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
#     if MEASUREMENT
        delimiter_micros = duration;
#     endif
      reception_stage = DELIMITED;
    } else if (reception_stage == DELIMITED) {
      if (duration < MIN_PACKET_PREAMBLE) {
        if (logLevel >= LogLevel::MINOR) {
          Serial.print(duration);
          Serial.println("µs short preamble after delimiter");
        }
        cancel_packet();
        return;
      }
      if (duration > MAX_PACKET_PREAMBLE) {
        if (logLevel >= LogLevel::MINOR) {
          Serial.print(duration);
          Serial.println("µs long preamble after delimiter");
        }
        cancel_packet();
        return;
      }
      if (!train_handled) {
        digitalWrite(LED_BUILTIN, HIGH);
#       if MEASUREMENT
          preamble_micros = duration;
#       endif
      }
      reception_stage = OPENED;
      extra_peaks_received = 0;
      bits_received = 0;
    } else if (reception_stage >= OPENED) {
      if (duration < MIN_PEAK_SPACING) {
        if (logLevel >= LogLevel::MINOR) {
          Serial.print(duration);
          Serial.print("µs peak in vale #");
          Serial.println(reception_stage);
        }
        cancel_packet();
        return;
      }
      if (duration <= MAX_PEAK_SPACING) {
        ++extra_peaks_received;
      } else {
        if (duration <= MAX_VALE_SPACING) {
          if (logLevel >= LogLevel::MINOR) {
            Serial.print(duration);
            Serial.print("µs vale after vale #");
            Serial.println(reception_stage);
          }
          cancel_packet();
          return;
        }
        const int bit = 1 + int(bits_received & 1) - extra_peaks_received;
        if (bit < 0 || bit > 1) {
          if (logLevel >= LogLevel::MINOR) {
            Serial.print("Invalid peak count ");
            Serial.println(1 + extra_peaks_received);
          }
          cancel_packet();
          return;
        }
        bits_received = (bits_received << 1) | bit;
        extra_peaks_received = 0;
        ++reception_stage;
        if (reception_stage == FINISHED) {
          if (!train_handled) {
            packet_complete_micros = now + PARITY_TIMEOUT;
          }
        }
      }
    }
  }

  unsigned long receive() {
    const unsigned long now = micros();
    noInterrupts();
    const unsigned long packet_complete = now - packet_complete_micros;
    const bool has_new = reception_stage == FINISHED
                      && extra_peaks_received == int(bits_received & 1);
    if (packet_complete >> 31) {
      // packet_complete_micros is in the future (regardless of overflow),
      // or it is zero and meaningless at the moment.
      interrupts();
      return 0;
    } else if (has_new && bits_received != train_handled) {
      train_handled = bits_received;
      interrupts();
#     if MEASUREMENT
        Serial.print("After ");
        Serial.print(delimiter_micros);
        Serial.print("µs delimiter + ");
        Serial.print(preamble_micros);
        Serial.println("µs preamble:");
#     endif
      Serial.print("At ");
      Serial.print(now);
      Serial.print(" received ");
      return train_handled;
    } else if (train_handled && packet_complete > TRAIN_TIMEOUT) {
      abort_packet_train();
      interrupts();
      if (logLevel >= LogLevel::MAJOR) {
        Serial.println("Stop expecting rest of packet train");
      }
      return 0;
    } else {
      interrupts();
      return 0;
    }
  }
};
