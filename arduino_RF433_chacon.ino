#include "pitches.h"
#include "BitsReceiver.h"
#include "SerialOrNot.h"
#include "TransmitterButtonStorage.h"

static const unsigned long LOOP_DELAY_MILLIS = 50;  // save energy, but allow processing buffers
static const uint8_t LOOPS_LEARNING = 80;
static const uint8_t LOOPS_SPEED_UP = 36;
static const uint8_t LOOPS_SLOW_DOWN = 4;
static const uint8_t LOOPS_PER_HEARTBEAT = 25;

#ifdef ARDUINO_AVR_NANO
static const bool LOG_EVENTS = true;
static const bool LOG_TIMING = true;
enum { PIN_IN_ASK = 2,
       PIN_OUT_BUZZER = 10,
       PIN_OUT_SLOW,
       PIN_OUT_FAST,
       PIN_OUT_LED = LED_BUILTIN };
static const int INT_ASK = digitalPinToInterrupt(PIN_IN_ASK);
#else  // ATTiny85
static const bool LOG_EVENTS = false;
static const bool LOG_TIMING = false;
// Stay away from pins 3 and 4 for relay output, because they're flipped during boot.
enum { PIN_OUT_SLOW = 0,
       PIN_OUT_FAST = 1,
       PIN_IN_ASK = 2,
       PIN_OUT_BUZZER = 3,
       PIN_OUT_LED = 4 };
static const int INT_ASK = 0;
#endif

class SpeedRegulator {
  uint8_t transition_iterations = 0;

  enum Speed : uint8_t { OFF,
                         SLOW,
                         FAST };
  static void write_speed(Speed speed) {
    // Make sure that at no point both output pins are high
    if (speed != SLOW) digitalWrite(PIN_OUT_SLOW, LOW);
    if (speed != FAST) digitalWrite(PIN_OUT_FAST, LOW);
    if (speed == SLOW) digitalWrite(PIN_OUT_SLOW, HIGH);
    if (speed == FAST) digitalWrite(PIN_OUT_FAST, HIGH);
  }

  static bool is_fast() {
    return digitalRead(PIN_OUT_FAST);
  }

  static bool is_slow() {
    return digitalRead(PIN_OUT_SLOW);
  }

public:
  void step() {
    if (transition_iterations > 0) {
      if (--transition_iterations == 0) {
        if (is_slow()) {
          write_speed(FAST);
        } else {
          write_speed(SLOW);
        }
      }
    }
  }

  void speed_up() {
    if (transition_iterations > 0) {
      transition_iterations = 0;
    } else if (!is_slow() && !is_fast()) {
      transition_iterations = LOOPS_SPEED_UP;
    }
    write_speed(FAST);  // ramp up with high power, we'll tone it down for slow speed after the transition
  }

  void slow_down() {
    if (transition_iterations > 0) {
      transition_iterations = 0;
    } else if (is_fast()) {
      transition_iterations = LOOPS_SLOW_DOWN;
    }
    write_speed(OFF);  // ramp down without power, we'll power on slow speed after the transition
  }
};

static const uint8_t PRIORITY_NOTICES = 128;
enum class LocalNotice : uint8_t {
  MISSED_PACKET = PRIORITY_NOTICES,
  REGISTER_ACTUAL,
  REGISTER_AGAIN,
  DEREGISTER_ACTUAL,
  DEREGISTER_AGAIN,
  DEREGISTER_ALL,
  GOING_NOWHERE,
  GOING_UP,
  GOING_DOWN
};

static uint16_t note1(uint8_t beeps_buzzing) {
  switch (beeps_buzzing) {
    case uint8_t(LocalNotice::REGISTER_ACTUAL):
    case uint8_t(LocalNotice::REGISTER_AGAIN):
    case uint8_t(LocalNotice::DEREGISTER_ACTUAL):
    case uint8_t(LocalNotice::DEREGISTER_AGAIN):
    case uint8_t(LocalNotice::DEREGISTER_ALL):
    case uint8_t(LocalNotice::GOING_NOWHERE):
    case uint8_t(LocalNotice::GOING_UP):
    case uint8_t(LocalNotice::GOING_DOWN): return NOTE_E5;
    case uint8_t(LocalNotice::MISSED_PACKET):
    default: return NOTE_A2;
  }
}

static uint16_t note2(uint8_t beeps_buzzing) {
  switch (beeps_buzzing) {
    case uint8_t(LocalNotice::REGISTER_ACTUAL):
    case uint8_t(LocalNotice::REGISTER_AGAIN): return NOTE_E6;
    case uint8_t(LocalNotice::DEREGISTER_ACTUAL):
    case uint8_t(LocalNotice::DEREGISTER_AGAIN): return NOTE_A4;
    case uint8_t(LocalNotice::DEREGISTER_ALL): return NOTE_A3;
    case uint8_t(LocalNotice::GOING_NOWHERE): return NOTE_E5;
    case uint8_t(LocalNotice::GOING_UP): return NOTE_A5;
    case uint8_t(LocalNotice::GOING_DOWN): return NOTE_A4;
    case uint8_t(LocalNotice::MISSED_PACKET): return NOTE_A2;
    default: return NOTE_E3;
  }
}

static uint16_t note3(uint8_t beeps_buzzing) {
  switch (beeps_buzzing) {
    case uint8_t(LocalNotice::REGISTER_ACTUAL): return NOTE_A6;
    case uint8_t(LocalNotice::REGISTER_AGAIN): return NOTE_E6;
    case uint8_t(LocalNotice::DEREGISTER_ACTUAL): return NOTE_E4;
    case uint8_t(LocalNotice::DEREGISTER_AGAIN): return NOTE_A4;
    case uint8_t(LocalNotice::DEREGISTER_ALL): return NOTE_E3;
    default: return 0;
  }
}

static volatile uint8_t primary_notice = 0;
static uint32_t primary_notice_time;

// The first packet in a train is practically always broken. Don't publish any error
// until we're sure it's not superseded by a properly received packet in the same train:
static bool is_time_to_publish() {
  return duration_from_to(primary_notice_time, micros()) >= BitsReceiver::TRAIN_TIMEOUT;
}

static SerialOrNot_t<LOG_EVENTS> SerialOrNot;

struct EventLogger {
  template<typename T> static void print(T t) {
    SerialOrNot.print(t);
  }
  template<typename T, typename F> static void print(T t, F f) {
    SerialOrNot.print(t, f);
  }
  template<typename N, typename T> static void println(N notice, T t) {
    SerialOrNot.println(t);
    if (primary_notice < uint8_t(notice)) {
      primary_notice = uint8_t(notice);
      primary_notice_time = micros();
    }
  }
};

static BitsReceiver receiver;
static TransmitterButtonStorage transmitterButtonStorage;

static void print_button_pair(ChaconButtonPairId button_pair) {
  SerialOrNot.print(" button pair ");
  SerialOrNot.write('A' + button_pair.page());
  SerialOrNot.write('1' + button_pair.row());
}

static void dump_packet(const char* prefix, ChaconPacket packet) {
  SerialOrNot.print(prefix);
  SerialOrNot.print(" transmitter ");
  SerialOrNot.print(packet.transmitter(), HEX);
  if (packet.multicast()) {
    SerialOrNot.print(" all ");
  } else {
    print_button_pair(packet.button_pair());
  }
}

static void dump_button_pairs(const char* prefix) {
  for (int i = 0; i < transmitterButtonStorage.count(); ++i) {
    auto button_pair = transmitterButtonStorage.get(i);
    SerialOrNot.print(prefix);
    SerialOrNot.print(" transmitter ");
    SerialOrNot.print(button_pair.transmitter(), HEX);
    print_button_pair(button_pair);
    SerialOrNot.println();
  }
}

void setup() {
  SerialOrNot.begin(115200);

  pinMode(PIN_IN_ASK, INPUT);
  pinMode(PIN_OUT_LED, OUTPUT);
  pinMode(PIN_OUT_BUZZER, OUTPUT);
  pinMode(PIN_OUT_SLOW, OUTPUT);
  pinMode(PIN_OUT_FAST, OUTPUT);
  receiver.setup();
  attachInterrupt(
    INT_ASK, []() {
      if (!receiver.handle_rise()) {
        primary_notice = uint8_t(LocalNotice::MISSED_PACKET);
        SerialOrNot.println("Buffer not timely processed by main loop");
      }
    },
    RISING);

  transmitterButtonStorage.load();
  dump_button_pairs("Initial");
  SerialOrNot.println("Start learning");

  tone(PIN_OUT_BUZZER, NOTE_A6, 75);
  delay(150);
}

static bool learn(ChaconPacket packet) {
  if (packet.multicast()) {
    if (!packet.on_or_off()) {
      SerialOrNot.println("Received wipe");
      transmitterButtonStorage.forget_all();
      primary_notice = uint8_t(LocalNotice::DEREGISTER_ALL);
      return true;
    }
  } else {
    dump_packet("Received", packet);
    SerialOrNot.println(packet.on_or_off() ? "①" : "⓪");
    if (packet.on_or_off()) {
      if (transmitterButtonStorage.remember(packet.button_pair())) {
        primary_notice = uint8_t(LocalNotice::REGISTER_ACTUAL);
      } else {
        primary_notice = uint8_t(LocalNotice::REGISTER_AGAIN);
      }
    } else {
      if (transmitterButtonStorage.forget(packet.button_pair())) {
        primary_notice = uint8_t(LocalNotice::DEREGISTER_ACTUAL);
      } else {
        primary_notice = uint8_t(LocalNotice::DEREGISTER_AGAIN);
      }
    }
    return true;
  }
  return false;
}

static bool end_learning() {
  transmitterButtonStorage.store();
  if (transmitterButtonStorage.count() == 0) {
    return false;
  } else {
    for (uint8_t i = transmitterButtonStorage.count(); i > 0; --i) {
      digitalWrite(PIN_OUT_LED, HIGH);
      tone(PIN_OUT_BUZZER, NOTE_A5);
      delay(50);
      noTone(PIN_OUT_BUZZER);
      digitalWrite(PIN_OUT_LED, LOW);
      delay(25);
    }
    return true;
  }
}

static void heartbeat() {
  static uint8_t iterations = 0;
  switch (++iterations) {
    case LOOPS_PER_HEARTBEAT - 5:
      if (receiver.has_been_alive()) {
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
  static uint8_t beeps_buzzing = 0;  // one of the PRIORITY_NOTICES, or number of beeps remaining to sound
  static uint8_t iterations;         // active when beeps_buzzing > 0

  if (primary_notice > 0) {
    if (primary_notice >= PRIORITY_NOTICES || (!beeps_buzzing && is_time_to_publish())) {
      beeps_buzzing = uint8_t(primary_notice);
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
      case 6:
        if (beeps_buzzing >= PRIORITY_NOTICES) {
          if (note3(beeps_buzzing)) {
            tone(PIN_OUT_BUZZER, note3(beeps_buzzing), 60);
          } else {
            beeps_buzzing = 0;
          }
        } else if (beeps_buzzing < 5) {
          beeps_buzzing -= 1;
          iterations = 2;
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

void loop() {
  static uint8_t iterations_learning = LOOPS_LEARNING;
  static SpeedRegulator speed_regulator;

  delay(LOOP_DELAY_MILLIS);

  if (iterations_learning > 0) {
    if (--iterations_learning == 0) {
      if (end_learning()) {
        SerialOrNot.println("Ended learning");
      } else {
        iterations_learning = 1;
      }
    }
  }

  if (iterations_learning > 0) {
    digitalWrite(PIN_OUT_LED, iterations_learning & 8 ? HIGH : LOW);
  } else {
    heartbeat();
  }

  uint32_t bits;
  while (receiver.receive<EventLogger, LOG_TIMING>(bits)) {
    const ChaconPacket packet{ bits };
    const bool recognized = transmitterButtonStorage.recognizes(packet);
    dump_packet("Received", packet);
    SerialOrNot.println(packet.on_or_off() ? "①" : "⓪");
    if (iterations_learning > 0) {
      if (learn(packet)) {
        iterations_learning = LOOPS_LEARNING;
      }
    } else if (!recognized) {
      primary_notice = uint8_t(LocalNotice::GOING_NOWHERE);  // also supersede errors from initial packet(s) in the packet train
    } else if (packet.on_or_off()) {
      primary_notice = uint8_t(LocalNotice::GOING_UP);
      speed_regulator.speed_up();
    } else {
      primary_notice = uint8_t(LocalNotice::GOING_DOWN);
      speed_regulator.slow_down();
    }
  }

  speed_regulator.step();
  buzz_primary_notice();
}
