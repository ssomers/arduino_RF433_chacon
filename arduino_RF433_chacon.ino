#include "pitches.h"
#include "ProtocolHandler.h"
#include "TransmitterButtonStorage.h"

static const byte PIN_DIGITAL_IN = 2;
static const byte PIN_BUZZER = 12;
static const byte LEVELS = 2;
static const byte PIN_DIGITAL_OUT[LEVELS] = { 3, 4 };
static const unsigned long INITIAL_LEARNING_MILLIS = 4000;
static const LogEvents logEvents = LogEvents::NONE;

static ProtocolHandler<logEvents> handler;
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
  transmitterButtonStorage.for_each([prefix](unsigned long transmitter_button) {
    print_transmitter_and_button(prefix, Packet(transmitter_button));
    Serial.println();
  });
}

static void initial_learning() {
  unsigned long restart_millis = millis();

  transmitterButtonStorage.load();
  if (logEvents > LogEvents::NONE) {
    dump_transmitters_and_buttons("Initial");
    Serial.println("Start learning");
  }
  unsigned long buzz_millis = restart_millis;
  for (;;) {
    const auto passed_millis = millis();
    if (passed_millis >= restart_millis + INITIAL_LEARNING_MILLIS) {
      if (transmitterButtonStorage.count() != 0) {
        break;
      }
    }
    if (passed_millis >= buzz_millis) {
      buzz_millis += 1000;
      tone(PIN_BUZZER, NOTE_E4, 20);
    }

    const auto bits = handler.receive();
    if (bits != Bits::NO_PACKET) {
      const auto packet = Packet(bits);
      bool change;
      unsigned int freq1;
      unsigned int freq2;
      if (packet.multicast()) {
        change = !packet.on_or_off();
        freq1 = NOTE_A3;
        freq2 = NOTE_E3;
        if (change) {
          if (logEvents > LogEvents::NONE) {
            Serial.println("Received wipe");
          }
          transmitterButtonStorage.forget_all();
        }
      } else {
        if (logEvents > LogEvents::NONE) {
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
  transmitterButtonStorage.for_each([](unsigned long) {
    tone(PIN_BUZZER, NOTE_A5, 40);
    delay(80);
  });
  if (logEvents > LogEvents::NONE) {
    Serial.println("Ended learning");
  }
}

void setup() {
  if (logEvents > LogEvents::NONE) {
    Serial.begin(115200);
    while (!Serial); // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(PIN_DIGITAL_IN, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_DIGITAL_OUT[0], OUTPUT);
  pinMode(PIN_DIGITAL_OUT[1], OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_IN), []() {
    handler.handleRise();
  }, RISING);
  delay(100); // avoid spurious beep
  initial_learning();
}

void loop() {
  delay(10); // save energy?
  const auto bits = handler.receive();
  if (bits != Bits::NO_PACKET) {
    const auto packet = Packet(bits);
    const bool recognized = transmitterButtonStorage.recognizes(packet);
    if (logEvents > LogEvents::NONE) {
      print_transmitter_and_button("Received", packet);
      Serial.print(packet.on_or_off() ? "①" : "⓪");
      Serial.println(recognized ? ", hallelujah!" : ", never mind");
    }
    tone(PIN_BUZZER, NOTE_E5, 20);
    if (recognized) {
      delay(40);
      if (packet.on_or_off()) {
        tone(PIN_BUZZER, NOTE_A6, 60);
        for (byte i = 0; i < LEVELS; ++i) {
          if (digitalRead(PIN_DIGITAL_OUT[i]) == LOW) {
            digitalWrite(PIN_DIGITAL_OUT[i], HIGH);
            break;
          }
        }
      } else {
        tone(PIN_BUZZER, NOTE_E4, 60);
        for (byte i = LEVELS; i > 0; --i) {
          if (digitalRead(PIN_DIGITAL_OUT[i - 1]) == HIGH) {
            digitalWrite(PIN_DIGITAL_OUT[i - 1], LOW);
            break;
          }
        }
      }
    }
  }
}
