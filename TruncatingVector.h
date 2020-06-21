// Vector remembering only the first STORED values
template <typename size_type, size_type STORED, typename T>
class TruncatingVector {
    size_type count;
    T values[STORED];

  public:
    void reset() {
      count = 0;
    }

    void push_back(T value) {
      if (count < STORED) {
        values[count] = value;
      }
      ++count;
      // may overflow size_type, that's up to the caller
    }

    size_type size() const {
      return count;
    }

    T operator[](size_type p) const {
      return values[p];
    }
};
