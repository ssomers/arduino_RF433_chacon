#include <avr/eeprom.h>
#include "ChaconPacket.h"

class TransmitterButtonStorage {
  static const uint8_t BUTTON_PAIRS_STORED = 4;
  ChaconButtonPairId button_pairs[BUTTON_PAIRS_STORED];
  uint8_t button_pair_count = 0;
  static uint32_t* const eeprom_address;

public:
  void load() {
    for (button_pair_count = 0; button_pair_count < BUTTON_PAIRS_STORED; ++button_pair_count) {
      uint32_t bits = eeprom_read_dword(&eeprom_address[button_pair_count]);
      auto valid = button_pairs[button_pair_count].load_bits(bits).valid();
      if (!valid) {
        break;
      }
    }
    for (uint8_t i = button_pair_count; i < BUTTON_PAIRS_STORED; ++i) {
      button_pairs[i].invalidate();
    }
  }

  uint8_t count() const {
    return button_pair_count;
  }

  ChaconButtonPairId get(uint8_t index) const {
    return button_pairs[index];
  }

  bool contains(ChaconButtonPairId some_button_pair) const {
    for (uint8_t i = 0; i < button_pair_count; ++i) {
      if (some_button_pair == button_pairs[i]) {
        return true;
      }
    }
    return false;
  }

  bool recognizes(ChaconPacket packet) const {
    for (uint8_t i = 0; i < button_pair_count; ++i) {
      if (packet.matches(button_pairs[i])) {
        return true;
      }
    }
    return false;
  }

  bool remember(ChaconButtonPairId some_button_pair) {
    if (!contains(some_button_pair)) {
      if (button_pair_count == BUTTON_PAIRS_STORED) {
        for (uint8_t i = 1; i < BUTTON_PAIRS_STORED; ++i) {
          button_pairs[i - 1] = button_pairs[i];
        }
        button_pair_count -= 1;
      }
      button_pairs[button_pair_count] = some_button_pair;
      button_pair_count += 1;
      return true;
    } else {
      return false;
    }
  }

  bool forget(ChaconButtonPairId some_button_pair) {
    bool found = false;
    uint8_t old_index = 0;
    uint8_t new_index = 0;
    while (old_index < button_pair_count) {
      if (button_pairs[old_index] == some_button_pair) {
        found = true;
      } else {
        button_pairs[new_index] = button_pairs[old_index];
        ++new_index;
      }
      ++old_index;
    }
    button_pair_count = new_index;
    while (new_index < BUTTON_PAIRS_STORED) {
      button_pairs[new_index].invalidate();
      ++new_index;
    }
    return found;
  }

  void forget_all() {
    while (button_pair_count > 0) {
      button_pair_count -= 1;
      button_pairs[button_pair_count].invalidate();
    }
  }

  void store() {
    for (uint8_t i = 0; i < BUTTON_PAIRS_STORED; ++i) {
      uint32_t bits = button_pairs[i].extract_bits();
      eeprom_update_dword(&eeprom_address[i], bits);
    }
  }
};

uint32_t* const TransmitterButtonStorage::eeprom_address = 0;
