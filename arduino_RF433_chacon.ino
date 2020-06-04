#include "pitches.h"
#include "ProtocolHandler.h"
#include "TransmitterButtonStorage.h"

static const bool logEvents = false;
static const bool logTiming = false;
static const Notice IGNORED_NOTICE = PREAMBLE_TOO_SOON;

static const byte PIN_DIGITAL_IN = 2;
static const byte PIN_DIGITAL_OUT_POWER = 3;
static const byte PIN_DIGITAL_OUT_SPEED = 4;
static const unsigned long INITIAL_LEARNING_MILLIS = 4000;
static const unsigned long LOOP_MILLIS = 5;
static const byte LOOPS_PER_BOOST = 250;
static const byte LOOPS_PER_HEARTBEAT = 190;

#ifdef LED_BUILTIN
/*
  #define Q(V) #V
  #define QV(V) Q(V)
  #pragma message "\nLED_BUILTIN = " QV(LED_BUILTIN) ";"
*/
static const byte PIN_BUZZER = 6;
static const int INT_IN = digitalPinToInterrupt(PIN_DIGITAL_IN);
#else
static const byte PIN_BUZZER = 0;
static const byte LED_BUILTIN = 1;
static const int INT_IN = 0;
#endif


static Notice worst_notice = IGNORED_NOTICE;
static unsigned long worst_notice_time;
static unsigned long last_good_time;

struct BuzzingEventLogger {
  template <typename T> static void print(Notice, T) {}
  template <typename T, typename F> static void print(Notice, T, F) {}
  template <typename T> static void println(Notice n, T) {
    const unsigned long now = micros();
    if (worst_notice > n && duration_from_to(last_good_time, now) > TRAIN_TIMEOUT) {
      worst_notice = n;
      worst_notice_time = now;
    }
  }
};

struct SerialEventLogger {
  template <typename T> static void print(Notice n, T t) {
    if (n < IGNORED_NOTICE) Serial.print(t);
  }
  template <typename T, typename F> static void print(Notice n, T t, F f) {
    if (n < IGNORED_NOTICE) Serial.print(t, f);
  }
  template <typename T> static void println(Notice n, T t) {
    if (n < IGNORED_NOTICE) Serial.println(t);
  }
};

static ProtocolHandler<BuzzingEventLogger, logTiming> handler;
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

static void initial_learning() {
  unsigned long restart_millis = millis();

  transmitterButtonStorage.load();
  if (logEvents) {
    dump_transmitters_and_buttons("Initial");
    Serial.println("Start learning");
  }
  unsigned long buzz_millis = restart_millis;
  for (;;) {
    const unsigned long passed_millis = millis();
    if (passed_millis >= restart_millis + INITIAL_LEARNING_MILLIS) {
      break;
    }
    if (passed_millis >= buzz_millis) {
      if (digitalRead(LED_BUILTIN) == LOW) {
        digitalWrite(LED_BUILTIN, HIGH);
        tone(PIN_BUZZER, NOTE_E4, 20);
      } else {
        digitalWrite(LED_BUILTIN, LOW);
      }
      buzz_millis += 500;
    }

    const unsigned long bits = handler.receive();
    if (bits != VOID_BITS) {
      const Packet packet(bits);
      bool change;
      unsigned int freq1;
      unsigned int freq2;
      if (packet.multicast()) {
        change = !packet.on_or_off();
        freq1 = NOTE_A3;
        freq2 = NOTE_E3;
        if (change) {
          if (logEvents) {
            Serial.println("Received wipe");
          }
          transmitterButtonStorage.forget_all();
        }
      } else {
        if (logEvents) {
          print_transmitter_and_button("Received", packet);
          Serial.println(packet.on_or_off() ? "①" : "⓪");
        }
        if (packet.on_or_off()) {
          change = transmitterButtonStorage.remember(packet.transmitter_and_button());
          freq1 = NOTE_E6;
          freq2 = NOTE_A6;
        } else {
          change = transmitterButtonStorage.forget(packet.transmitter_and_button());
          freq1 = NOTE_A4;
          freq2 = NOTE_E4;
        }
      }
      restart_millis = buzz_millis;
      if (change) {
        tone(PIN_BUZZER, NOTE_E5);
        delay(75);
        tone(PIN_BUZZER, freq1);
        delay(75);
        tone(PIN_BUZZER, freq2, 50);
      } else {
        tone(PIN_BUZZER, freq1, 50);
        delay(75);
        tone(PIN_BUZZER, freq1, 50);
      }
      delay(200);
    }
  }
  transmitterButtonStorage.store();
  for (byte i = transmitterButtonStorage.count(); i > 0; --i) {
    digitalWrite(LED_BUILTIN, HIGH);
    tone(PIN_BUZZER, NOTE_A5);
    delay(50);
    noTone(PIN_BUZZER);
    digitalWrite(LED_BUILTIN, LOW);
    delay(25);
  }
  if (logEvents) {
    Serial.println("Ended learning");
  }
  last_good_time = micros();
}

void setup() {
  if (logEvents) {
    Serial.begin(115200);
  }

  pinMode(PIN_DIGITAL_IN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_DIGITAL_OUT_POWER, OUTPUT);
  pinMode(PIN_DIGITAL_OUT_SPEED, OUTPUT);
  attachInterrupt(INT_IN, []() {
    handler.handleRise();
  }, RISING);
#ifdef LED_BUILTIN
  delay(100); // avoid spurious beep
#endif
  do {
    initial_learning();
  } while (transmitterButtonStorage.count() == 0);
}

static void heartbeat() {
  static byte iterations = 0;
  switch (++iterations) {
    case LOOPS_PER_HEARTBEAT - 44:
      if (handler.is_alive()) {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      break;
    case LOOPS_PER_HEARTBEAT - 40:
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case LOOPS_PER_HEARTBEAT - 4:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case LOOPS_PER_HEARTBEAT - 0:
      digitalWrite(LED_BUILTIN, LOW);
      iterations = 0;
  }
}

static void report_worst_notice() {
  static byte beeps_buzzing = 0;
  static byte iterations;

  if (beeps_buzzing == 0 && worst_notice < IGNORED_NOTICE
      && duration_from_to(worst_notice_time, micros()) > TRAIN_TIMEOUT) {
    beeps_buzzing = worst_notice;
    worst_notice = IGNORED_NOTICE;
    iterations = 0;
  }
  if (beeps_buzzing > 0) {
    switch (++iterations) {
      case 1:
        tone(PIN_BUZZER, NOTE_A2);
        break;
      case 21:
        noTone(PIN_BUZZER);
        break;
      case 31:
        tone(PIN_BUZZER, NOTE_E3);
        break;
      case 37:
        if (beeps_buzzing < 5) {
          noTone(PIN_BUZZER);
        }
        break;
      case 54:
        if (beeps_buzzing < 5) {
          beeps_buzzing -= 1;
          iterations = 30;
        }
        noTone(PIN_BUZZER);
        break;
      case 64:
        beeps_buzzing -= 5;
        iterations = 30;
        break;
    }
  }
}

void loop() {
  delay(LOOP_MILLIS); // save energy & provide timer
  heartbeat();
  report_worst_notice();

  static byte boost_iterations = 0;
  if (boost_iterations > 0) {
    if (--boost_iterations == 0) {
      digitalWrite(PIN_DIGITAL_OUT_SPEED, LOW);
    }
  }

  const unsigned long bits = handler.receive();
  if (bits != VOID_BITS) {
    last_good_time = micros();
    worst_notice = IGNORED_NOTICE; // wipe errors from initial packet(s) in packet train

    const Packet packet(bits);
    const bool recognized = transmitterButtonStorage.recognizes(packet);
    if (logEvents) {
      print_transmitter_and_button("Received", packet);
      Serial.print(packet.on_or_off() ? "①" : "⓪");
      Serial.println(recognized ? ", hallelujah!" : ", never mind");
    }
    tone(PIN_BUZZER, NOTE_E5, 20);
    if (recognized) {
      if (packet.on_or_off()) {
        if (boost_iterations > 0) {
          boost_iterations = 0;
        } else if (digitalRead(PIN_DIGITAL_OUT_POWER) == LOW) {
          boost_iterations = LOOPS_PER_BOOST;
        }
        digitalWrite(PIN_DIGITAL_OUT_SPEED, HIGH);
        digitalWrite(PIN_DIGITAL_OUT_POWER, HIGH);
      } else {
        if (boost_iterations > 0 || digitalRead(PIN_DIGITAL_OUT_SPEED) == LOW) {
          digitalWrite(PIN_DIGITAL_OUT_POWER, LOW);
        }
        digitalWrite(PIN_DIGITAL_OUT_SPEED, LOW);
        boost_iterations = 0;
      }
      delay(40);
      tone(PIN_BUZZER, packet.on_or_off() ? NOTE_A6 : NOTE_E4, 60);
    }
  }
}
