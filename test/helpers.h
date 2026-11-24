/*
 * Helpers shared by all test files: an independent CRC reference, frame
 * builders and checks for the events recorded by the Arduino fakes.
 */
#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <doctest/doctest.h>

#include <vector>

#include "MockLog.h"
#include "Wire.h"

typedef std::vector<uint8_t> Bytes;

static const uint8_t SENSOR_ADDRESS = 0x40;

// Bit-serial reference implementation of the Sensirion CRC-8
// (polynomial x^8 + x^5 + x^4 + 1, init 0x00, no reflection, no final XOR).
// The message is shifted in one bit at a time, i.e. this is deliberately not
// the byte-wise algorithm used by the library.
inline uint8_t referenceCrc(const Bytes& data) {
  uint16_t reg = 0;
  for (size_t i = 0; i < data.size() + 1; i++) {
    const uint8_t byte = i < data.size() ? data[i] : 0x00;  // 8 zero bits flush the register
    for (int bit = 7; bit >= 0; bit--) {
      reg = static_cast<uint16_t>((reg << 1) | ((byte >> bit) & 0x01));
      if (reg & 0x100) {
        reg ^= 0x131;
      }
    }
  }
  return static_cast<uint8_t>(reg);
}

// A measurement as it is sent by the sensor: MSB, LSB, CRC
inline Bytes frame(const uint16_t raw) {
  Bytes result;
  result.push_back(static_cast<uint8_t>(raw >> 8));
  result.push_back(static_cast<uint8_t>(raw & 0xFF));
  result.push_back(referenceCrc(result));
  return result;
}

inline Bytes bytes(const uint8_t a) {
  return Bytes(1, a);
}

inline Bytes bytes(const uint8_t a, const uint8_t b) {
  Bytes result;
  result.push_back(a);
  result.push_back(b);
  return result;
}

// Creates a clean bus and an empty event log for each test case
struct BusFixture {
  TwoWire bus;

  BusFixture() { mock::reset(); }

  static size_t eventCount() { return mock::events().size(); }
};

inline void checkWrite(const size_t index, const TwoWire& bus, const Bytes& expected) {
  REQUIRE(mock::events().size() > index);
  const mock::Event& event = mock::events()[index];
  CHECK(event.kind == mock::Event::WRITE);
  CHECK(event.bus == &bus);
  CHECK(event.address == SENSOR_ADDRESS);
  CHECK(event.data == expected);
}

inline void checkDelay(const size_t index, const unsigned long ms) {
  REQUIRE(mock::events().size() > index);
  const mock::Event& event = mock::events()[index];
  CHECK(event.kind == mock::Event::DELAY);
  CHECK(event.ms == ms);
}

inline void checkRead(const size_t index, const TwoWire& bus, const size_t requested) {
  REQUIRE(mock::events().size() > index);
  const mock::Event& event = mock::events()[index];
  CHECK(event.kind == mock::Event::READ);
  CHECK(event.bus == &bus);
  CHECK(event.address == SENSOR_ADDRESS);
  CHECK(event.requested == requested);
}

// A complete query: command, wait for the conversion, read the result
inline void checkQuery(const size_t index, const TwoWire& bus, const Bytes& command, const unsigned long ms, const size_t requested) {
  checkWrite(index, bus, command);
  checkDelay(index + 1, ms);
  checkRead(index + 2, bus, requested);
}

// The conversion formulas of the datasheets. The two status bits must be
// cleared by the caller where the datasheet requires it.
inline double celsius(const uint16_t raw) {
  return -46.85 + 175.72 * raw / 65536.0;
}

inline double relativeHumidity(const uint16_t raw) {
  return -6.0 + 125.0 * raw / 65536.0;
}

#endif
