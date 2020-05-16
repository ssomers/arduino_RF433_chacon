#include "ProtocolHandler.h"
#include "TransmitterButtonStorage.h"

static const int PIN_DIGITAL_IN = 2;
static const int PIN_DIGITAL_OUT_1 = 11;
static const int PIN_DIGITAL_OUT_2 = 12;
static const unsigned INITIAL_LEARNING_MILLIS = 500;

static ProtocolHandler<LogLevel::MAJOR> handler;
static TransmitterButtonStorage transmitterButtonStorage;
static int speed = 0;

static void initial_learning() {
  const unsigned long boot_millis = millis();

  transmitterButtonStorage.load();
  transmitterButtonStorage.dump("Initial");
  Serial.println("Start learning");
  unsigned long passed_millis;
  while ((passed_millis = millis() - boot_millis) < INITIAL_LEARNING_MILLIS) {
    digitalWrite(LED_BUILTIN, (passed_millis >> 9) & 1);
    if (auto bits = handler.receive()) {
      transmitterButtonStorage.learn(Packet(bits));
    }
  }
  if (transmitterButtonStorage.store()) {
    transmitterButtonStorage.dump("Updated");
  }
  Serial.println("Ended learning");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_DIGITAL_IN, INPUT);
  pinMode(PIN_DIGITAL_OUT_1, OUTPUT);
  pinMode(PIN_DIGITAL_OUT_2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_IN), [](){handler.handleRise();}, RISING);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only
  initial_learning();
}

void loop() {
  delay(3);
  if (auto bits = handler.receive()) {
    auto p = Packet(bits);
    p.print_transmitter_and_button();
    Serial.print(p.on_or_off() ? "①" : "⓪");
    if (transmitterButtonStorage.recognizes(p)) {
      Serial.println(", hallelujah!");
      if (p.on_or_off()) {
        switch (speed) {
          case 0: {
            digitalWrite(PIN_DIGITAL_OUT_1, HIGH);
            ++speed;
            break;
          }
          case 1: {
            digitalWrite(PIN_DIGITAL_OUT_2, HIGH);
            ++speed;
            break;
          }
        }
      } else {
        switch (speed) {
          case 2: {
            digitalWrite(PIN_DIGITAL_OUT_2, LOW);
            --speed;
            break;
          }
          case 1: {
            digitalWrite(PIN_DIGITAL_OUT_1, LOW);
            --speed;
            break;
          }
        }
      }
    } else {
      Serial.println(", never mind");
    }
  }
}
