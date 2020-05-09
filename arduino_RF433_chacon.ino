static const int PIN_DIGITAL_IN = 2;
static const int DATA_LEN = 33;

class ProtocolHandler {
  unsigned long last_time = 0;
  enum { IDLE, READY, RECEIVING } state = IDLE;
  int vales_received;
  char data[DATA_LEN];
  int prev_vales_received;
  char prev_data[DATA_LEN];

  void bail_out() {
    digitalWrite(LED_BUILTIN, LOW);
    state = IDLE;
    prev_vales_received = 0;
  }

public:
  void handleRise() {
    const unsigned long time = micros();
    const unsigned long duration = time > last_time ? time - last_time : 0;
    last_time = time;
  
    if (duration >= 9900) {
      if (state == RECEIVING) {
        if (prev_vales_received) {
          if (vales_received != prev_vales_received || memcmp(prev_data, data, vales_received)) {
            Serial.write(prev_data, prev_vales_received + 1);
            Serial.println(" received, but then");
            Serial.write(data, vales_received + 1);
            Serial.println(" received!");
          } else {
            Serial.write(data, vales_received + 1);
            Serial.println(" received loud and clear");
          }
          bail_out();
        } else {
          prev_vales_received = vales_received;
          memcpy(prev_data, data, vales_received + 1);
          state = IDLE;
        }
      } else {
        state = READY;
      }
    } else switch (state) {
      case IDLE: return;
      case READY: {
        if (duration < 2900) {
          bail_out(); // Not a genuine packet
          return;
        }
        digitalWrite(LED_BUILTIN, HIGH);
        state = RECEIVING;
        vales_received = 0;
        data[0] = '0';
        return;
      }
      case RECEIVING: {
        if (duration < 500) {
          Serial.print(duration);
          Serial.println(" µs peak spotted!");
          bail_out();
          return;
        }
        if (duration < 600) {
          ++data[vales_received];
        } else {
          if (duration < 1500 || duration > 1600) {
            Serial.print(duration);
            Serial.println(" µs vale spotted!");
            bail_out();
            return;
          }
          ++vales_received;
          data[vales_received] = '0';
          if (vales_received >= DATA_LEN) {
            Serial.println("Bailing out!");
            bail_out();
          }
        }
        return;
      }
    }
  }
};

static ProtocolHandler handler;

static volatile unsigned noise_spotted = 0;

void handleRise() {
  handler.handleRise();
}

void setup() {
  pinMode(PIN_DIGITAL_IN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_IN), handleRise, RISING);
  Serial.begin(115200);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only
}

void loop() {
}
