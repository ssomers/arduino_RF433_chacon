#include <EEPROM.h>
#include "Packet.h"

class TransmitterButtonStorage {
  static const int TRANSMITTER_BUTTONS_STORED = 4;

  unsigned long transmitter_buttons[TRANSMITTER_BUTTONS_STORED];
  int transmitter_button_count = 0;
  bool dirty = false;

  void remember(unsigned long some_transmitter_button) {
    if (!recognizes(Packet(some_transmitter_button))) {
      if (transmitter_button_count == TRANSMITTER_BUTTONS_STORED) {
        for (int i = 1; i < TRANSMITTER_BUTTONS_STORED; ++i) {
          transmitter_buttons[i - 1] = transmitter_buttons[i];
        }
        transmitter_button_count -= 1;
      }
      transmitter_buttons[transmitter_button_count] = some_transmitter_button;
      transmitter_button_count += 1;
      dirty = true;
    }
  }

  void forget(unsigned long some_transmitter_button) {
    int old_index = 0;
    int new_index = 0;
    while (old_index < transmitter_button_count) {
      if (transmitter_buttons[old_index] == some_transmitter_button) {
        dirty = true;
      } else {
        transmitter_buttons[new_index] = transmitter_buttons[old_index];
        ++new_index;
      }
      ++old_index;
    }
    transmitter_button_count = new_index;
    while (new_index < TRANSMITTER_BUTTONS_STORED) {
      transmitter_buttons[new_index] = ~0ul;
      ++new_index;
    }
  }

public:
  void load() {
    EEPROM.get(0, transmitter_buttons);
    while (transmitter_button_count < TRANSMITTER_BUTTONS_STORED &&
           transmitter_buttons[transmitter_button_count] != ~0ul) {
      ++transmitter_button_count;
    }
  }

  void dump(const char* prefix) {
    for (int i = 0; i < transmitter_button_count; ++i) {
      Serial.print(prefix);
      Packet(transmitter_buttons[i]).print_transmitter_and_button();
      Serial.println();
    }
  }

  bool recognizes(Packet packet) const {
    for (int i = 0; i < transmitter_button_count; ++i) {
      if (packet.matches(transmitter_buttons[i])) {
        return true;
      }
    }
    return false;
  }

  void learn(Packet packet) {
    if (!packet.multicast()) {
      if (packet.on_or_off()) {
        remember(packet.transmitter_and_button());
      } else {
        forget(packet.transmitter_and_button());
      }
    }
  }

  bool store() {
    if (dirty) {
      EEPROM.put(0, transmitter_buttons);
      dirty = false;
      return true;
    } else {
      return false;
    }
  }
};
