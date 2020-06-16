template <uint8_t STORED, typename T>
class PeakArray {
    uint8_t count;
    T values[STORED];

  public:
    void initialize() {
      count = 0;
    }

    void append(uint8_t value) {
      if (count < STORED) {
        values[count] = value;
      }
      ++count;
      // never mind that we might get 256 or more consecutive appends
    }

    uint8_t counted() const {
      return count;
    }

    T value(uint8_t p) const {
      return values[p];
    }
};
