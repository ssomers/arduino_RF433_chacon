#include <ATtinySerialOut.h>

#include "pitches.h"
#include "ProtocolHandler.h"
#include "TransmitterButtonStorage.h"

static const ProtocolNotice MIN_CONSIDERED_NOTICE = INVALID_PREAMBLE;

static const uint8_t PIN_IN_ASK = 2;
static const uint8_t PIN_OUT_BUZZER = 3;
static const unsigned long LOOP_MILLIS = 50;
static const uint8_t LOOPS_LEARNING = 128;
static const uint8_t LOOPS_CHATTY = 0; // meaning 256
static const uint8_t LOOPS_SPEED_UP = 20;
static const uint8_t LOOPS_SLOW_DOWN = 2;
static const uint8_t LOOPS_PER_HEARTBEAT = 25;

#ifdef LED_BUILTIN
// Aduino on modern library
static const bool logEvents = true;
static const bool logTiming = false;
static const int INT_IN = digitalPinToInterrupt(PIN_IN_ASK);
static const uint8_t PIN_OUT_LED = LED_BUILTIN;
static const uint8_t PIN_OUT_SLOW = 11;
static const uint8_t PIN_OUT_FAST = 12;
#else
// ATTiny85 on ancient Digispark library
static const bool logEvents = false;
static const bool logTiming = false;
static const int INT_IN = 0;
static const uint8_t PIN_OUT_LED = 4;
static const uint8_t PIN_OUT_SLOW = 0;
static const uint8_t PIN_OUT_FAST = 1;
#endif

enum Speed : uint8_t { OFF, SLOW, FAST };

static void write_speed(Speed speed) {
  // Make sure that at no point both output pins are high
  if (speed != SLOW) digitalWrite(PIN_OUT_SLOW, LOW);
  if (speed != FAST) digitalWrite(PIN_OUT_FAST, LOW);
  if (speed == SLOW) digitalWrite(PIN_OUT_SLOW, HIGH);
  if (speed == FAST) digitalWrite(PIN_OUT_FAST, HIGH);
}

static Speed read_speed() {
  if (digitalRead(PIN_OUT_FAST)) {
    return FAST;
  } else if (digitalRead(PIN_OUT_SLOW)) {
    return SLOW;
  } else {
    return OFF;
  }
}


enum LocalNotice : uint8_t { LOCALS = 128, REGISTER_ACTUAL = LOCALS, REGISTER_AGAIN, DEREGISTER_ACTUAL, DEREGISTER_AGAIN, DEREGISTER_ALL,
                             SHUTTING_UP, GOING_NOWHERE, GOING_UP, GOING_DOWN
                           };

static uint16_t note1(uint8_t beeps_buzzing) {
  switch (beeps_buzzing) {
    case REGISTER_ACTUAL:
    case REGISTER_AGAIN:
    case DEREGISTER_ACTUAL:
    case DEREGISTER_AGAIN:
    case DEREGISTER_ALL:
    case GOING_NOWHERE:
    case GOING_UP:
    case GOING_DOWN:        return NOTE_E5;
    case SHUTTING_UP:
    default:                return NOTE_A2;
  }
}

static uint16_t note2(uint8_t beeps_buzzing) {
  switch (beeps_buzzing) {
    case REGISTER_ACTUAL:
    case REGISTER_AGAIN:    return NOTE_E6;
    case DEREGISTER_ACTUAL:
    case DEREGISTER_AGAIN:  return NOTE_A4;
    case DEREGISTER_ALL:    return NOTE_A3;
    case GOING_NOWHERE:     return NOTE_E5;
    case GOING_UP:          return NOTE_A5;
    case GOING_DOWN:        return NOTE_A4;
    case SHUTTING_UP:       return NOTE_E2;
    default:                return NOTE_E3;
  }
}

static uint16_t note3(uint8_t beeps_buzzing) {
  switch (beeps_buzzing) {
    case REGISTER_ACTUAL:   return NOTE_A6;
    case REGISTER_AGAIN:    return NOTE_E6;
    case DEREGISTER_ACTUAL: return NOTE_E4;
    case DEREGISTER_AGAIN:  return NOTE_A4;
    case DEREGISTER_ALL:    return NOTE_E3;
    case SHUTTING_UP:       return NOTE_A1;
    default :               return 0;
  }
}

static uint8_t primary_notice = 0;
static uint32_t primary_notice_time;

template <bool logEvents> struct EventLogger;
template <> struct EventLogger<false> {
  template <typename T> static void print(ProtocolNotice, T) {}
  template <typename T, typename F> static void print(ProtocolNotice, T, F) {}
  template <typename T> static void println(ProtocolNotice n, T) {
    if (n >= MIN_CONSIDERED_NOTICE && n > primary_notice) {
      primary_notice = n;
      primary_notice_time = micros();
    }
  }
};
template <> struct EventLogger<true> {
  template <typename T> static void print(ProtocolNotice n, T t) {
    if (n >= MIN_CONSIDERED_NOTICE) Serial.print(t);
  }
  template <typename T, typename F> static void print(ProtocolNotice n, T t, F f) {
    if (n >= MIN_CONSIDERED_NOTICE) Serial.print(t, f);
  }
  template <typename T> static void println(ProtocolNotice n, T t) {
    if (n >= MIN_CONSIDERED_NOTICE) Serial.println(t);
    EventLogger<false>::println(n, t);
  }
};

static ProtocolHandler<EventLogger<logEvents>, logTiming> handler;
static TransmitterButtonStorage transmitterButtonStorage;

static void print_transmitter_and_button(const char* prefix, Packet packet) {
  Serial.print(prefix);
  Serial.print(" transmitter ");
  Serial.print(packet.transmitter(), HEX);
  if (packet.multicast()) {
    Serial.print(" all ");
  } else {
    Serial.print(" button ");
    Serial.write('A' + packet.page());
    Serial.write('1' + packet.row());
  }
}

static void dump_transmitters_and_buttons(const char* prefix) {
  for (int i = 0; i < transmitterButtonStorage.count(); ++i) {
    print_transmitter_and_button(prefix, Packet(transmitterButtonStorage.get(i)));
    Serial.println();
  }
}

void setup() {
  if (logEvents) {
    Serial.begin(115200);
  }

  pinMode(PIN_IN_ASK, INPUT);
  pinMode(PIN_OUT_LED, OUTPUT);
  pinMode(PIN_OUT_BUZZER, OUTPUT);
  pinMode(PIN_OUT_SLOW, OUTPUT);
  pinMode(PIN_OUT_FAST, OUTPUT);
  attachInterrupt(INT_IN, []() {
    handler.handle_rise();
  }, RISING);

  transmitterButtonStorage.load();
  if (logEvents) {
    dump_transmitters_and_buttons("Initial");
    Serial.println("Start learning");
  }

  tone(PIN_OUT_BUZZER, NOTE_A6, 75);
  delay(150);
}

static bool initial_learning() {
  static uint8_t iterations = 0;

  ++iterations;
  if (iterations == LOOPS_LEARNING) {
    transmitterButtonStorage.store();
    if (transmitterButtonStorage.count() > 0) {
      return true;
    }
    iterations = 0;
  }
  digitalWrite(PIN_OUT_LED, iterations & 8 ? HIGH : LOW);

  const uint32_t bits = handler.receive();
  if (bits != VOID_BITS) {
    const Packet packet(bits);
    if (packet.multicast()) {
      if (!packet.on_or_off()) {
        if (logEvents) {
          Serial.println("Received wipe");
        }
        transmitterButtonStorage.forget_all();
        primary_notice = DEREGISTER_ALL;
        iterations = 0;
      }
    } else {
      if (logEvents) {
        print_transmitter_and_button("Received", packet);
        Serial.println(packet.on_or_off() ? "①" : "⓪");
      }
      if (packet.on_or_off()) {
        if (transmitterButtonStorage.remember(packet.transmitter_and_button())) {
          primary_notice = REGISTER_ACTUAL;
        } else {
          primary_notice = REGISTER_AGAIN;
        }
      } else {
        if (transmitterButtonStorage.forget(packet.transmitter_and_button())) {
          primary_notice = DEREGISTER_ACTUAL;
        } else {
          primary_notice = DEREGISTER_AGAIN;
        }
      }
      iterations = 0;
    }
  }
  return false;
}

static void heartbeat() {
  static uint8_t iterations = 0;
  switch (++iterations) {
    case LOOPS_PER_HEARTBEAT - 5:
      if (handler.has_been_alive()) {
        digitalWrite(PIN_OUT_LED, HIGH);
      }
      break;
    case LOOPS_PER_HEARTBEAT - 4:
      digitalWrite(PIN_OUT_LED, LOW);
      break;
    case LOOPS_PER_HEARTBEAT - 1:
      digitalWrite(PIN_OUT_LED, HIGH);
      break;
    case LOOPS_PER_HEARTBEAT - 0:
      digitalWrite(PIN_OUT_LED, LOW);
      iterations = 0;
  }
}

static void buzz_primary_notice() {
  static uint8_t beeps_buzzing = 0;
  static uint8_t iterations;

  if (primary_notice >= LOCALS) {
    beeps_buzzing = primary_notice;
    primary_notice = 0;
    iterations = 0;
  } else if (beeps_buzzing == 0 && primary_notice > 0) {
    // postpone notice until we're sure it isn't going to be cancelled by a properly received packet
    if (duration_from_to(primary_notice_time, micros()) > TRAIN_TIMEOUT) {
      beeps_buzzing = primary_notice;
      primary_notice = 0;
      iterations = 0;
    }
  }

  if (beeps_buzzing > 0) {
    switch (++iterations) {
      case 1:
        tone(PIN_OUT_BUZZER, note1(beeps_buzzing), 80);
        break;
      case 3:
        tone(PIN_OUT_BUZZER, note2(beeps_buzzing), beeps_buzzing < 5 ? 25 : 100);
        break;
      case 5:
        if (beeps_buzzing >= LOCALS && !note3(beeps_buzzing)) {
          beeps_buzzing = 0;
        } else if (beeps_buzzing < 5) {
          beeps_buzzing -= 1;
          iterations = 2;
        }
        break;
      case 6:
        if (beeps_buzzing >= LOCALS) {
          tone(PIN_OUT_BUZZER, note3(beeps_buzzing), 60);
        } else {
          beeps_buzzing -= 5;
          iterations = 2;
        }
        break;
      case 8:
        beeps_buzzing = 0;
        break;
    }
  }
}

static void respond() {
  static uint8_t transition_iterations = 0;
  if (transition_iterations > 0) {
    if (--transition_iterations == 0) {
      write_speed(read_speed() == SLOW ? FAST : SLOW);
    }
  }

  const uint32_t bits = handler.receive();
  if (bits != VOID_BITS) {

    const Packet packet(bits);
    const bool recognized = transmitterButtonStorage.recognizes(packet);
    if (logEvents) {
      print_transmitter_and_button("Received", packet);
      Serial.print(packet.on_or_off() ? "①" : "⓪");
      Serial.println(recognized ? ", hallelujah!" : ", never mind");
    }
    if (recognized) {
      if (packet.on_or_off()) {
        if (transition_iterations > 0) {
          transition_iterations = 0;
        } else if (read_speed() == OFF) {
          transition_iterations = LOOPS_SPEED_UP;
        }
        write_speed(FAST);
      } else {
        if (transition_iterations > 0) {
          transition_iterations = 0;
        } else if (read_speed() == FAST) {
          transition_iterations = LOOPS_SLOW_DOWN;
        }
        write_speed(OFF);
      }
      primary_notice = packet.on_or_off() ? GOING_UP : GOING_DOWN;
    } else {
      primary_notice = GOING_NOWHERE; // also wipe errors from initial packet(s) in packet train
    }
  }
}

void loop() {
  delay(LOOP_MILLIS); // save energy & provide good enough intervals

  static bool learned = false;
  if (!learned) {
    if (initial_learning()) {
      learned = true;
      if (logEvents) {
        Serial.println("Ended learning");
      }
      for (uint8_t i = transmitterButtonStorage.count(); i > 0; --i) {
        digitalWrite(PIN_OUT_LED, HIGH);
        tone(PIN_OUT_BUZZER, NOTE_A5);
        delay(50);
        noTone(PIN_OUT_BUZZER);
        digitalWrite(PIN_OUT_LED, LOW);
        delay(25);
      }
    }
  } else {
    heartbeat();
    respond();
  }
  buzz_primary_notice();
}
