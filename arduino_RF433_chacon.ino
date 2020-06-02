#include "pitches.h"
#include "ProtocolHandler.h"
#include "TransmitterButtonStorage.h"

static const bool logEvents = false;
static const bool logTiming = false;
static const Notice MAX_NOTICE = EXCESS_TOTAL_PEAKS;

static const byte PIN_DIGITAL_IN = 2;
static const byte PIN_DIGITAL_OUT_POWER = 3;
static const byte PIN_DIGITAL_OUT_SPEED = 4;
static const unsigned long INITIAL_LEARNING_MILLIS = 4000;
static const unsigned long LOOP_MILLIS = 5;
static const byte LOOPS_PER_BOOST = 200;

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


static bool error = false;

struct QuietEventLogger {
  template <typename T>  static void print(Notice, T) {}
  template <typename T, typename F> static void print(Notice, T, F) {}
  template <typename T>  static void println(Notice, T) {}
};

struct BuzzingEventLogger {
  template <typename T> static void print(Notice, T) {}
  template <typename T, typename F> static void print(Notice, T, F) {}
  template <typename T> static void println(Notice n, T) {
    error |= n <= MAX_NOTICE;
  }
};

struct SerialEventLogger {
  template <typename T> static void print(Notice n, T t) {
    if (n <= MAX_NOTICE) Serial.print(t);
  }
  template <typename T, typename F> static void print(Notice n, T t, F f) {
    if (n <= MAX_NOTICE) Serial.print(t, f);
  }
  template <typename T> static void println(Notice n, T t) {
    if (n <= MAX_NOTICE) Serial.println(t);
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
      if (transmitterButtonStorage.count() != 0) {
        break;
      }
    }
    if (passed_millis >= buzz_millis) {
      buzz_millis += 1000;
      tone(PIN_BUZZER, NOTE_E4, 20);
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
  for (int i = 0; i < transmitterButtonStorage.count(); ++i) {
    tone(PIN_BUZZER, NOTE_A5, 40);
    delay(80);
  }
  if (logEvents) {
    Serial.println("Ended learning");
  }
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
  initial_learning();
}

void loop() {
  if (error) {
    error = false;
    tone(PIN_BUZZER, NOTE_E2);
    delay(20);
    noTone(PIN_BUZZER);
  }
  delay(LOOP_MILLIS); // save energy & provide timer

  static byte alive_iterations = 0;
  switch (++alive_iterations) {
    case 200:
      if (handler.is_alive()) digitalWrite(LED_BUILTIN, HIGH);
      break;
    case 204:
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case 240:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case 244:
      digitalWrite(LED_BUILTIN, LOW);
      alive_iterations = 0;
  }

  static byte boost_iterations = 0;
  if (boost_iterations > 0) {
    if (--boost_iterations == 0) {
      digitalWrite(PIN_DIGITAL_OUT_SPEED, LOW);
    }
  }

  const unsigned long bits = handler.receive();
  if (bits != VOID_BITS) {
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
