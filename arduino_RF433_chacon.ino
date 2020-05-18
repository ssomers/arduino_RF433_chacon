#include "ProtocolHandler.h"
#include "TransmitterButtonStorage.h"

static const int PIN_DIGITAL_IN = 2;
static const int LEVELS = 2;
static const int PIN_DIGITAL_OUT[LEVELS] = { 11, 12 };
static const unsigned INITIAL_LEARNING_MILLIS = 500;

static ProtocolHandler<LogEvents::SOME> handler;
static TransmitterButtonStorage transmitterButtonStorage;

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
  pinMode(PIN_DIGITAL_OUT[0], OUTPUT);
  pinMode(PIN_DIGITAL_OUT[1], OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_IN), []() {
    handler.handleRise();
  }, RISING);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only
  initial_learning();
}

void loop() {
  delay(3);
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
