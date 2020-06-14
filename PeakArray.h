template <uint8_t STORED, typename T>
class PeakArray {
    uint8_t count;
    T values[STORED];

  public:
    void initialize_idle() {
      count = 0xFF;
    }

    void initialize_active() {
      count = 0;
    }

    void append(uint8_t value) {
      if (is_active()) {
        if (count < STORED) {
          values[count] = value;
        }
        ++count;
        // if we get so many consecutive appends that count saturates, just pretend we are idle
      }
    }

    bool is_active() const {
      return count != 0xFF;
    }

    uint8_t counted() const {
      return count;
    }

    T value(uint8_t p) const {
      return values[p];
    }
};
