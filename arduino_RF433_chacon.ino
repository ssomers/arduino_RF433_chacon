#include "pitches.h"
#include "ProtocolHandler.h"
#include "TransmitterButtonStorage.h"

static const int PIN_DIGITAL_IN = 2;
static const int PIN_BUZZER = 12;
static const int LEVELS = 2;
static const int PIN_DIGITAL_OUT[LEVELS] = { 3, 4 };
static const unsigned INITIAL_LEARNING_MILLIS = 4000;

static const LogEvents logEvents = LogEvents::NONE;
static ProtocolHandler<logEvents> handler;
static TransmitterButtonStorage transmitterButtonStorage;

static void initial_learning() {
  const unsigned long boot_millis = millis();

  transmitterButtonStorage.load();
  if (logEvents > LogEvents::NONE) {
    transmitterButtonStorage.dump("Initial");
    Serial.println("Start learning");
  }
  unsigned long passed_millis;
  unsigned long buzz_millis = boot_millis;
  while ((passed_millis = millis()) < boot_millis + INITIAL_LEARNING_MILLIS) {
    if (passed_millis >= buzz_millis) {
      buzz_millis += 1000;
      tone(PIN_BUZZER, NOTE_E4, 25);
    }
    auto bits = handler.receive();
    if (bits != Bits::NO_PACKET) {
      auto p = Packet(bits);
      if (!p.multicast()) {
        if (logEvents > LogEvents::NONE) {
          Serial.print("Received");
          p.print_transmitter_and_button();
          Serial.println(p.on_or_off() ? "①" : "⓪");
        }
        if (p.on_or_off()) {
          if (transmitterButtonStorage.remember(p.transmitter_and_button())) {
            tone(PIN_BUZZER, NOTE_E5, 50);
            delay(75);
            tone(PIN_BUZZER, NOTE_E6, 50);
            delay(75);
            tone(PIN_BUZZER, NOTE_A6, 80);
          } else {
            tone(PIN_BUZZER, NOTE_E6, 50);
            delay(75);
            tone(PIN_BUZZER, NOTE_E6, 50);
          }
        } else {
          if (transmitterButtonStorage.forget(p.transmitter_and_button())) {
            tone(PIN_BUZZER, NOTE_E5, 50);
            delay(75);
            tone(PIN_BUZZER, NOTE_A6, 50);
            delay(75);
            tone(PIN_BUZZER, NOTE_E6, 75);
          } else {
            tone(PIN_BUZZER, NOTE_A4, 50);
            delay(75);
            tone(PIN_BUZZER, NOTE_A4, 50);
          }
        }
      }
    }
  }
  if (transmitterButtonStorage.store()) {
    transmitterButtonStorage.dump("Updated");
  }
  tone(PIN_BUZZER, NOTE_A5, 75);
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
      Serial.print("Received");
      packet.print_transmitter_and_button();
      Serial.print(packet.on_or_off() ? "①" : "⓪");
      Serial.println(recognized ? ", hallelujah!" : ", never mind");
    }
    tone(PIN_BUZZER, NOTE_E5);
    delay(50);
    tone(PIN_BUZZER, recognized ? NOTE_A6 : NOTE_A4, 50);
    if (recognized) {
      if (packet.on_or_off()) {
        for (int i = 0; i < LEVELS; ++i) {
          if (digitalRead(PIN_DIGITAL_OUT[i]) == LOW) {
            digitalWrite(PIN_DIGITAL_OUT[i], HIGH);
            break;
          }
        }
      } else {
        for (int i = LEVELS - 1; i >= 0; --i) {
          if (digitalRead(PIN_DIGITAL_OUT[i]) == HIGH) {
            digitalWrite(PIN_DIGITAL_OUT[i], LOW);
            break;
          }
        }
      }
    }
  }
}
