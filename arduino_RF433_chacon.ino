#include "pitches.h"
#include "ProtocolHandler.h"
#include "TransmitterButtonStorage.h"

static const int PIN_DIGITAL_IN = 2;
static const int PIN_BUZZER = 12;
static const int LEVELS = 2;
static const int PIN_DIGITAL_OUT[LEVELS] = { 3, 4 };
static const unsigned INITIAL_LEARNING_MILLIS = 4000;

static ProtocolHandler<LogEvents::SOME> handler;
static TransmitterButtonStorage transmitterButtonStorage;

static void initial_learning() {
  const unsigned long boot_millis = millis();

  transmitterButtonStorage.load();
  transmitterButtonStorage.dump("Initial");
  Serial.println("Start learning");
  unsigned long passed_millis;
  unsigned long buzz_millis = boot_millis;
  while ((passed_millis = millis()) < boot_millis + INITIAL_LEARNING_MILLIS) {
    if (passed_millis >= buzz_millis) {
      buzz_millis += 1000;
      tone(PIN_BUZZER, NOTE_E4, 30);
    }
    auto bits = handler.receive();
    if (bits != Bits::NO_PACKET) {
      auto p = Packet(bits);
      if (!p.multicast()) {
        Serial.print("Received");
        p.print_transmitter_and_button();
        Serial.println(p.on_or_off() ? "①" : "⓪");
        if (p.on_or_off()) {
          if (transmitterButtonStorage.remember(p.transmitter_and_button())) {
            tone(PIN_BUZZER, NOTE_E5, 50);
            delay(80);
            tone(PIN_BUZZER, NOTE_E6, 50);
            delay(80);
            tone(PIN_BUZZER, NOTE_A6, 80);
          } else {
            tone(PIN_BUZZER, NOTE_E6, 50);
            delay(80);
            tone(PIN_BUZZER, NOTE_E6, 50);
          }
        } else {
          if (transmitterButtonStorage.forget(p.transmitter_and_button())) {
            tone(PIN_BUZZER, NOTE_E5, 50);
            delay(80);
            tone(PIN_BUZZER, NOTE_A6, 50);
            delay(80);
            tone(PIN_BUZZER, NOTE_E6, 80);
          } else {
            tone(PIN_BUZZER, NOTE_A4, 50);
            delay(80);
            tone(PIN_BUZZER, NOTE_A4, 50);
          }
        }
      }
    }
  }
  if (transmitterButtonStorage.store()) {
    transmitterButtonStorage.dump("Updated");
  }
  tone(PIN_BUZZER, NOTE_A4, 120);
  delay(120);
  tone(PIN_BUZZER, NOTE_A5, 60);
  Serial.println("Ended learning");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_DIGITAL_IN, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_DIGITAL_OUT[0], OUTPUT);
  pinMode(PIN_DIGITAL_OUT[1], OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_IN), []() {
    handler.handleRise();
  }, RISING);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only
  delay(100); // avoid spurious beep
  initial_learning();
}

void loop() {
  delay(10);
  auto bits = handler.receive();
  if (bits != Bits::NO_PACKET) {
    auto p = Packet(bits);
    Serial.print("Received");
    p.print_transmitter_and_button();
    Serial.print(p.on_or_off() ? "①" : "⓪");
    if (transmitterButtonStorage.recognizes(p)) {
      Serial.println(", hallelujah!");
      if (p.on_or_off()) {
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
    } else {
      Serial.println(", never mind");
    }
  }
}
