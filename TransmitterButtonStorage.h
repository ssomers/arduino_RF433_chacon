#include <avr/eeprom.h>
#include "Packet.h"

class TransmitterButtonStorage {
  static const uint8_t TRANSMITTER_BUTTONS_STORED = 4;
  static const uint32_t NONE = ~0ul;

  uint32_t transmitter_buttons[TRANSMITTER_BUTTONS_STORED];
  uint8_t transmitter_button_count = 0;

public:
  void load() {
    uint32_t* const address = 0;
    for (transmitter_button_count = 0;
         transmitter_button_count < TRANSMITTER_BUTTONS_STORED;
         transmitter_button_count++) {
      uint32_t tb = transmitter_buttons[transmitter_button_count] = eeprom_read_dword(&address[transmitter_button_count]);
      if (tb == NONE) {
        break;
      }
    }
    for (uint8_t i = transmitter_button_count; i < TRANSMITTER_BUTTONS_STORED; ++i) {
      transmitter_buttons[i] = NONE;
    }
  }

  uint8_t count() const {
    return transmitter_button_count;
  }

  uint32_t get(uint8_t index) const {
    return transmitter_buttons[index];
  }

  bool recognizes(Packet packet) const {
    for (uint8_t i = 0; i < transmitter_button_count; ++i) {
      if (packet.matches(transmitter_buttons[i])) {
        return true;
      }
    }
    return false;
  }

  bool remember(uint32_t some_transmitter_button) {
    if (!recognizes(Packet(some_transmitter_button))) {
      if (transmitter_button_count == TRANSMITTER_BUTTONS_STORED) {
        for (uint8_t i = 1; i < TRANSMITTER_BUTTONS_STORED; ++i) {
          transmitter_buttons[i - 1] = transmitter_buttons[i];
        }
        transmitter_button_count -= 1;
      }
      transmitter_buttons[transmitter_button_count] = some_transmitter_button;
      transmitter_button_count += 1;
      return true;
    } else {
      return false;
    }
  }

  bool forget(uint32_t some_transmitter_button) {
    bool found = false;
    uint8_t old_index = 0;
    uint8_t new_index = 0;
    while (old_index < transmitter_button_count) {
      if (transmitter_buttons[old_index] == some_transmitter_button) {
        found = true;
      } else {
        transmitter_buttons[new_index] = transmitter_buttons[old_index];
        ++new_index;
      }
      ++old_index;
    }
    transmitter_button_count = new_index;
    while (new_index < TRANSMITTER_BUTTONS_STORED) {
      transmitter_buttons[new_index] = NONE;
      ++new_index;
    }
    return found;
  }

  void forget_all() {
    while (transmitter_button_count > 0) {
      transmitter_button_count -= 1;
      transmitter_buttons[transmitter_button_count] = NONE;
    }
  }

  void store() {
    uint32_t* const address = 0;
    for (uint8_t i = 0; i < TRANSMITTER_BUTTONS_STORED; ++i) {
      eeprom_update_dword(&address[i], transmitter_buttons[i]);
    }
  }
};
