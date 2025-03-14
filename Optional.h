// Non-initializing minimal std::optional
template<typename T>
class Optional {
  bool present;
  union {
    T value;
  } u;

public:
  bool has_value() const {
    return present;
  }

  T value() const {
    return u.value;
  }

  Optional& operator=(T value) {
    present = true;
    u.value = value;
    return *this;
  }

  void reset() {
    present = false;
  }
};
