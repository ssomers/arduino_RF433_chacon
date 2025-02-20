template<bool enable> struct SerialOrNot_t;
template<> struct SerialOrNot_t<false> {
  void begin(long) {}
  template<typename T> void print(T) {}
  template<typename T, typename F> void print(T, F) {}
  void println() {}
  template<typename T> void println(T) {}
  template<typename T> void write(T) {}
};
template<> struct SerialOrNot_t<true> {
  void begin(long rate) {
    Serial.begin(rate);
  }
  template<typename T> void print(T t) {
    Serial.print(t);
  }
  template<typename T, typename F> void print(T t, F f) {
    Serial.print(t, f);
  }
  void println() {
    Serial.println();
  }
  template<typename T> void println(T t) {
    Serial.println(t);
  }
  template<typename T> void write(T t) {
    Serial.write(t);
  }
};
