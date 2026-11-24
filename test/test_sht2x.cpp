#include <doctest/doctest.h>

#include <cmath>

#include "SHT2x.h"
#include "helpers.h"

namespace {

// Gives the tests access to the protected building blocks
class ExposedSHT2x : public SHT2x {
  public:
    explicit ExposedSHT2x(TwoWire& bus) : SHT2x(bus) {}
    using SHT2x::convertToCelsius;
    using SHT2x::crc8;
    using SHT2x::queryCommand;
    // Only the single byte overload is accessible to derived classes
    bool writeCommand(const uint8_t command, uint8_t* data = NULL, const size_t len = 0) {
      return SHT2x::writeCommand(command, data, len);
    }
};

// Reserved bits (3..5) of the user register as found on a real SHT21 after power up
const uint8_t USER_REGISTER_DEFAULT = 0x3A;

}  // namespace

TEST_SUITE("SHT2x CRC") {
  TEST_CASE("reference implementation matches the examples of the Sensirion CRC application note") {
    CHECK(referenceCrc(bytes(0xDC)) == 0x79);
    CHECK(referenceCrc(bytes(0x68, 0x3A)) == 0x7C);
    CHECK(referenceCrc(bytes(0x4E, 0x85)) == 0x6B);
  }

  TEST_CASE("crc8 matches the examples of the Sensirion CRC application note") {
    const uint8_t one[] = {0xDC};
    const uint8_t two[] = {0x68, 0x3A};
    const uint8_t three[] = {0x4E, 0x85};
    CHECK(ExposedSHT2x::crc8(one, sizeof(one)) == 0x79);
    CHECK(ExposedSHT2x::crc8(two, sizeof(two)) == 0x7C);
    CHECK(ExposedSHT2x::crc8(three, sizeof(three)) == 0x6B);
  }

  TEST_CASE("crc8 of no data is the init value") {
    const uint8_t data[] = {0xFF};
    CHECK(ExposedSHT2x::crc8(data, 0) == 0x00);
  }

  TEST_CASE("crc8 matches the reference for all two byte messages with a single bit set and for all ones") {
    for (unsigned int bit = 0; bit < 16; bit++) {
      const uint16_t value = static_cast<uint16_t>(1u << bit);
      const uint8_t data[] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
      CAPTURE(value);
      CHECK(ExposedSHT2x::crc8(data, sizeof(data)) == referenceCrc(bytes(data[0], data[1])));
    }
    const uint8_t ones[] = {0xFF, 0xFF};
    CHECK(ExposedSHT2x::crc8(ones, sizeof(ones)) == referenceCrc(bytes(0xFF, 0xFF)));
  }
}

TEST_SUITE("SHT2x temperature") {
  TEST_CASE_FIXTURE(BusFixture, "no hold master measurement: command 0xF3, 85 ms conversion time, 3 byte read") {
    SHT2x sensor(bus);
    bus.queueRead(frame(0x683A));

    const float temperature = sensor.readTemp();

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xF3), 85, 3);
    // 0x683A is the example of the CRC application note, the status bits (0b10) must be cleared
    CHECK(temperature == doctest::Approx(celsius(0x6838)).epsilon(1e-6));
    CHECK(temperature == doctest::Approx(24.6862).epsilon(1e-5));
  }

  TEST_CASE_FIXTURE(BusFixture, "hold master measurement uses command 0xE3") {
    SHT2x sensor(bus);
    bus.queueRead(frame(0x6838));

    const float temperature = sensor.readTemp(true);

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xE3), 85, 3);
    CHECK(temperature == doctest::Approx(celsius(0x6838)).epsilon(1e-6));
  }

  TEST_CASE_FIXTURE(BusFixture, "conversion follows T = -46.85 + 175.72 * S / 2^16 over the full range") {
    SHT2x sensor(bus);
    const uint16_t values[] = {0x0000, 0x0004, 0x4000, 0x6838, 0x8000, 0xC000, 0xFFFC};
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
      CAPTURE(values[i]);
      bus.queueRead(frame(values[i]));
      // The tolerance is far below the 0.0027 K error caused by dividing by 2^16 - 1
      CHECK(std::fabs(sensor.readTemp() - celsius(values[i])) < 1e-4);
    }
    bus.queueRead(frame(0x0000));
    CHECK(sensor.readTemp() == doctest::Approx(-46.85));
    bus.queueRead(frame(0xFFFC));
    CHECK(sensor.readTemp() == doctest::Approx(128.8593).epsilon(1e-6));
  }

  TEST_CASE("convertToCelsius implements the datasheet formula") {
    CHECK(ExposedSHT2x::convertToCelsius(0) == doctest::Approx(-46.85));
    CHECK(ExposedSHT2x::convertToCelsius(0x8000) == doctest::Approx(-46.85 + 175.72 / 2));
  }

  TEST_CASE_FIXTURE(BusFixture, "raw value: status bits are part of the CRC, but not of the result") {
    SHT2x sensor(bus);
    for (uint16_t status = 0; status < 4; status++) {
      CAPTURE(status);
      bus.queueRead(frame(static_cast<uint16_t>(0xABCC | status)));
      CHECK(sensor.readTempRaw() == 0xABCC);
    }
    bus.queueRead(frame(0xFFFF));
    CHECK(sensor.readTempRaw(true) == 0xFFFC);
    checkWrite(eventCount() - 3, bus, bytes(0xE3));
  }

  TEST_CASE_FIXTURE(BusFixture, "a wrong CRC is reported as NaN") {
    SHT2x sensor(bus);
    Bytes response = frame(0x6838);

    SUBCASE("corrupted checksum") { response[2] ^= 0x01; }
    SUBCASE("corrupted MSB") { response[0] ^= 0x80; }
    SUBCASE("corrupted LSB") { response[1] ^= 0x10; }

    bus.queueRead(response);
    CHECK(std::isnan(sensor.readTemp()));
  }

  TEST_CASE_FIXTURE(BusFixture, "a missing sensor is reported as NaN and not as -46.85 degree C") {
    SHT2x sensor(bus);

    SUBCASE("no hold master") {
      CHECK(std::isnan(sensor.readTemp()));
      checkQuery(0, bus, bytes(0xF3), 85, 3);
    }
    SUBCASE("hold master") {
      CHECK(std::isnan(sensor.readTemp(true)));
      checkQuery(0, bus, bytes(0xE3), 85, 3);
    }
    SUBCASE("raw") {
      CHECK(sensor.readTempRaw() == 0);
    }
  }

  TEST_CASE_FIXTURE(BusFixture, "a short read is reported as NaN") {
    SHT2x sensor(bus);

    SUBCASE("checksum missing") { bus.queueRead(bytes(0x68, 0x38)); }
    SUBCASE("only one byte") { bus.queueRead(bytes(0x00)); }
    // {0x00, 0x00} followed by a missing checksum must not be mistaken for a valid frame (CRC of zeros is zero)
    SUBCASE("zeros without checksum") { bus.queueRead(bytes(0x00, 0x00)); }

    CHECK(std::isnan(sensor.readTemp()));
  }

  TEST_CASE_FIXTURE(BusFixture, "a command that is not acknowledged is reported as NaN and nothing is read") {
    SHT2x sensor(bus);
    bus.queueWriteStatus(2);  // NACK on address
    bus.queueRead(frame(0x6838));

    CHECK(std::isnan(sensor.readTemp()));
    REQUIRE(eventCount() == 1);
    checkWrite(0, bus, bytes(0xF3));
  }
}

TEST_SUITE("SHT2x humidity") {
  TEST_CASE_FIXTURE(BusFixture, "no hold master measurement: command 0xF5, 3 byte read after the conversion time") {
    SHT2x sensor(bus);
    // Example of the SHT21 datasheet: the sensor sends 0x6352, i.e. 0x6350 plus status bits 0b10 -> 42.5 %RH
    bus.queueRead(frame(0x6352));

    const float humidity = sensor.readHumidity();

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xF5), 29 + 85, 3);
    CHECK(humidity == doctest::Approx(relativeHumidity(0x6350)).epsilon(1e-6));
    CHECK(humidity == doctest::Approx(42.5).epsilon(1e-3));
  }

  TEST_CASE_FIXTURE(BusFixture, "hold master measurement uses command 0xE5") {
    SHT2x sensor(bus);
    bus.queueRead(frame(0x4E85));

    const float humidity = sensor.readHumidity(true);

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xE5), 29 + 85, 3);
    CHECK(humidity == doctest::Approx(relativeHumidity(0x4E84)).epsilon(1e-6));
  }

  TEST_CASE_FIXTURE(BusFixture, "conversion follows RH = -6 + 125 * S / 2^16 over the full range") {
    SHT2x sensor(bus);
    const uint16_t values[] = {0x0000, 0x0004, 0x4000, 0x6350, 0x8000, 0xC000, 0xFFFC};
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
      CAPTURE(values[i]);
      // A humidity measurement is flagged with status bits 0b10
      bus.queueRead(frame(static_cast<uint16_t>(values[i] | 0x0002)));
      // The tolerance is far below the 0.0019 %RH error caused by dividing by 2^16 - 1
      CHECK(std::fabs(sensor.readHumidity() - relativeHumidity(values[i])) < 1e-4);
    }
    bus.queueRead(frame(0x0002));
    CHECK(sensor.readHumidity() == doctest::Approx(-6.0));
    bus.queueRead(frame(0xFFFE));
    CHECK(sensor.readHumidity() == doctest::Approx(118.9924).epsilon(1e-6));
  }

  TEST_CASE_FIXTURE(BusFixture, "raw value: status bits are part of the CRC, but not of the result") {
    SHT2x sensor(bus);
    bus.queueRead(frame(0x6352));
    CHECK(sensor.readHumidityRaw() == 0x6350);
    bus.queueRead(frame(0xFFFF));
    CHECK(sensor.readHumidityRaw(true) == 0xFFFC);
    checkWrite(eventCount() - 3, bus, bytes(0xE5));
  }

  TEST_CASE_FIXTURE(BusFixture, "a wrong CRC is reported as NaN") {
    SHT2x sensor(bus);
    Bytes response = frame(0x6352);
    response[2] ^= 0xFF;
    bus.queueRead(response);
    CHECK(std::isnan(sensor.readHumidity()));
  }

  TEST_CASE_FIXTURE(BusFixture, "a missing sensor is reported as NaN and not as -6 %RH") {
    SHT2x sensor(bus);

    SUBCASE("no hold master") {
      CHECK(std::isnan(sensor.readHumidity()));
      checkQuery(0, bus, bytes(0xF5), 29 + 85, 3);
    }
    SUBCASE("hold master") {
      CHECK(std::isnan(sensor.readHumidity(true)));
      checkQuery(0, bus, bytes(0xE5), 29 + 85, 3);
    }
    SUBCASE("raw") {
      CHECK(sensor.readHumidityRaw() == 0);
    }
  }

  TEST_CASE_FIXTURE(BusFixture, "a short read is reported as NaN") {
    SHT2x sensor(bus);
    bus.queueRead(bytes(0x00, 0x00));
    CHECK(std::isnan(sensor.readHumidity()));
  }

  TEST_CASE_FIXTURE(BusFixture, "a command that is not acknowledged is reported as NaN and nothing is read") {
    SHT2x sensor(bus);
    bus.queueWriteStatus(3);  // NACK on data
    bus.queueRead(frame(0x6352));

    CHECK(std::isnan(sensor.readHumidity()));
    REQUIRE(eventCount() == 1);
    checkWrite(0, bus, bytes(0xF5));
  }
}

TEST_SUITE("SHT2x soft reset") {
  TEST_CASE_FIXTURE(BusFixture, "reset sends 0xFE and waits 15 ms for the sensor to reboot") {
    SHT2x sensor(bus);

    sensor.reset();

    REQUIRE(eventCount() == 2);
    checkWrite(0, bus, bytes(0xFE));
    checkDelay(1, 15);
  }
}

TEST_SUITE("SHT2x user register") {
  TEST_CASE_FIXTURE(BusFixture, "the register is read with command 0xE7 and a single byte read without delay") {
    SHT2x sensor(bus);
    bus.queueRead(bytes(USER_REGISTER_DEFAULT));

    sensor.getHeaterState();

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xE7), 0, 1);
  }

  TEST_CASE_FIXTURE(BusFixture, "getHeaterState decodes bit 2") {
    SHT2x sensor(bus);
    bus.queueRead(bytes(0x04));
    CHECK(sensor.getHeaterState() == true);
    bus.queueRead(bytes(0xFB));
    CHECK(sensor.getHeaterState() == false);
  }

  TEST_CASE_FIXTURE(BusFixture, "setHeaterState only changes bit 2 and writes with command 0xE6") {
    SHT2x sensor(bus);

    SUBCASE("on") {
      bus.queueRead(bytes(USER_REGISTER_DEFAULT));
      sensor.setHeaterState(true);
      REQUIRE(eventCount() == 4);
      checkQuery(0, bus, bytes(0xE7), 0, 1);
      checkWrite(3, bus, bytes(0xE6, 0x3E));
    }
    SUBCASE("on, all other bits cleared") {
      bus.queueRead(bytes(0x00));
      sensor.setHeaterState(true);
      checkWrite(3, bus, bytes(0xE6, 0x04));
    }
    SUBCASE("off") {
      bus.queueRead(bytes(0xFF));
      sensor.setHeaterState(false);
      checkWrite(3, bus, bytes(0xE6, 0xFB));
    }
    SUBCASE("off is the default") {
      bus.queueRead(bytes(0x3E));
      sensor.setHeaterState();
      checkWrite(3, bus, bytes(0xE6, 0x3A));
    }
  }

  TEST_CASE_FIXTURE(BusFixture, "getOtpReloadStatus decodes the inverted 'disable OTP reload' bit 1") {
    SHT2x sensor(bus);
    bus.queueRead(bytes(USER_REGISTER_DEFAULT));  // OTP reload is disabled by default
    CHECK(sensor.getOtpReloadStatus() == false);
    bus.queueRead(bytes(0x38));
    CHECK(sensor.getOtpReloadStatus() == true);
  }

  TEST_CASE_FIXTURE(BusFixture, "setOtpReloadStatus only changes bit 1") {
    SHT2x sensor(bus);

    SUBCASE("enable clears the bit") {
      bus.queueRead(bytes(0xFF));
      sensor.setOtpReloadStatus(true);
      REQUIRE(eventCount() == 4);
      checkQuery(0, bus, bytes(0xE7), 0, 1);
      checkWrite(3, bus, bytes(0xE6, 0xFD));
    }
    SUBCASE("disable sets the bit") {
      bus.queueRead(bytes(0x00));
      sensor.setOtpReloadStatus(false);
      checkWrite(3, bus, bytes(0xE6, 0x02));
    }
    SUBCASE("disabled is the default") {
      bus.queueRead(bytes(0x38));
      sensor.setOtpReloadStatus();
      checkWrite(3, bus, bytes(0xE6, 0x3A));
    }
  }

  TEST_CASE_FIXTURE(BusFixture, "getVddLowStatus decodes the end of battery bit 6") {
    SHT2x sensor(bus);
    bus.queueRead(bytes(0x40));
    CHECK(sensor.getVddLowStatus() == true);
    bus.queueRead(bytes(0xBF));
    CHECK(sensor.getVddLowStatus() == false);
  }

  TEST_CASE_FIXTURE(BusFixture, "getResolution decodes bits 7 and 0") {
    SHT2x sensor(bus);
    bus.queueRead(bytes(0x3A));
    CHECK(sensor.getResolution() == RH12_T14);
    bus.queueRead(bytes(0x3B));
    CHECK(sensor.getResolution() == RH8_T12);
    bus.queueRead(bytes(0xBA));
    CHECK(sensor.getResolution() == RH10_T13);
    bus.queueRead(bytes(0xBB));
    CHECK(sensor.getResolution() == RH11_T11);
    bus.queueRead(bytes(0x7E));
    CHECK(sensor.getResolution() == RH12_T14);
  }

  TEST_CASE_FIXTURE(BusFixture, "setResolution only changes bits 7 and 0") {
    SHT2x sensor(bus);

    SUBCASE("RH12_T14") {
      bus.queueRead(bytes(0xFF));
      sensor.setResolution(RH12_T14);
      REQUIRE(eventCount() == 4);
      checkQuery(0, bus, bytes(0xE7), 0, 1);
      checkWrite(3, bus, bytes(0xE6, 0x7E));
    }
    SUBCASE("RH8_T12") {
      bus.queueRead(bytes(0xBA));
      sensor.setResolution(RH8_T12);
      checkWrite(3, bus, bytes(0xE6, 0x3B));
    }
    SUBCASE("RH10_T13") {
      bus.queueRead(bytes(0x3B));
      sensor.setResolution(RH10_T13);
      checkWrite(3, bus, bytes(0xE6, 0xBA));
    }
    SUBCASE("RH11_T11") {
      bus.queueRead(bytes(0x00));
      sensor.setResolution(RH11_T11);
      checkWrite(3, bus, bytes(0xE6, 0x81));
    }
    SUBCASE("RH12_T14 is the default") {
      bus.queueRead(bytes(0xBB));
      sensor.setResolution();
      checkWrite(3, bus, bytes(0xE6, 0x3A));
    }
  }

  TEST_CASE_FIXTURE(BusFixture, "nothing is written if the register cannot be read, the reserved bits must not be overwritten") {
    SHT2x sensor(bus);

    SUBCASE("setHeaterState, sensor does not answer") { sensor.setHeaterState(true); }
    SUBCASE("setOtpReloadStatus, sensor does not answer") { sensor.setOtpReloadStatus(true); }
    SUBCASE("setResolution, sensor does not answer") { sensor.setResolution(RH11_T11); }

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xE7), 0, 1);
  }

  TEST_CASE_FIXTURE(BusFixture, "nothing is written if the read command is not acknowledged") {
    SHT2x sensor(bus);
    bus.queueWriteStatus(2);
    bus.queueRead(bytes(USER_REGISTER_DEFAULT));

    SUBCASE("setHeaterState") { sensor.setHeaterState(true); }
    SUBCASE("setOtpReloadStatus") { sensor.setOtpReloadStatus(true); }
    SUBCASE("setResolution") { sensor.setResolution(RH11_T11); }

    REQUIRE(eventCount() == 1);
    checkWrite(0, bus, bytes(0xE7));
  }

  TEST_CASE_FIXTURE(BusFixture, "getters fall back to the all bits cleared state if the register cannot be read") {
    SHT2x sensor(bus);
    CHECK(sensor.getHeaterState() == false);
    CHECK(sensor.getVddLowStatus() == false);
    CHECK(sensor.getOtpReloadStatus() == true);
    CHECK(sensor.getResolution() == RH12_T14);
  }
}

TEST_SUITE("SHT2x resolution helpers") {
  TEST_CASE("convertResTemperature returns the number of bits") {
    CHECK(SHT2x::convertResTemperature(RH12_T14) == 14);
    CHECK(SHT2x::convertResTemperature(RH8_T12) == 12);
    CHECK(SHT2x::convertResTemperature(RH10_T13) == 13);
    CHECK(SHT2x::convertResTemperature(RH11_T11) == 11);
  }

  TEST_CASE("convertResHumidity returns the number of bits") {
    CHECK(SHT2x::convertResHumidity(RH12_T14) == 12);
    CHECK(SHT2x::convertResHumidity(RH8_T12) == 8);
    CHECK(SHT2x::convertResHumidity(RH10_T13) == 10);
    CHECK(SHT2x::convertResHumidity(RH11_T11) == 11);
  }

  TEST_CASE("values that are not a valid resolution fall back to the power up default") {
    CHECK(SHT2x::convertResTemperature(static_cast<Resolution>(0x02)) == 14);
    CHECK(SHT2x::convertResHumidity(static_cast<Resolution>(0x02)) == 12);
  }

  TEST_CASE("the enum values are the bit patterns of the user register") {
    CHECK(static_cast<uint8_t>(RH12_T14) == 0x00);
    CHECK(static_cast<uint8_t>(RH8_T12) == 0x01);
    CHECK(static_cast<uint8_t>(RH10_T13) == 0x80);
    CHECK(static_cast<uint8_t>(RH11_T11) == 0x81);
  }
}

TEST_SUITE("SHT2x serial number") {
  // The numeric example of the Sensirion application note "Electronic
  // Identification Code": serial number 0x0080'7654'3210'0112
  static Bytes firstAccess() {
    const uint8_t snb[] = {0x76, 0x54, 0x32, 0x10};
    Bytes result;
    for (size_t i = 0; i < sizeof(snb); i++) {
      result.push_back(snb[i]);
      result.push_back(referenceCrc(bytes(snb[i])));
    }
    return result;
  }

  static Bytes secondAccess() {
    Bytes result = frame(0x0112);       // SNC_1, SNC_0, CRC
    const Bytes sna = frame(0x0080);    // SNA_1, SNA_0, CRC
    result.insert(result.end(), sna.begin(), sna.end());
    return result;
  }

  TEST_CASE_FIXTURE(BusFixture, "application note example: commands, read lengths and composition") {
    SHT2x sensor(bus);
    bus.queueRead(firstAccess());
    bus.queueRead(secondAccess());
    SerialNumber serial = {0xDEAD, 0xDEADBEEF, 0xDEAD};

    CHECK(sensor.readSerial(serial) == true);

    REQUIRE(eventCount() == 6);
    checkQuery(0, bus, bytes(0xFA, 0x0F), 0, 8);
    checkQuery(3, bus, bytes(0xFC, 0xC9), 0, 6);
    CHECK(serial.a == 0x0080);
    CHECK(serial.b == 0x76543210);
    CHECK(serial.c == 0x0112);
  }

  TEST_CASE_FIXTURE(BusFixture, "all bits of the serial number are transferred") {
    SHT2x sensor(bus);
    Bytes first;
    const uint8_t snb[] = {0xFE, 0xDC, 0xBA, 0x98};
    for (size_t i = 0; i < sizeof(snb); i++) {
      first.push_back(snb[i]);
      first.push_back(referenceCrc(bytes(snb[i])));
    }
    Bytes second = frame(0x8001);
    const Bytes sna = frame(0xF00F);
    second.insert(second.end(), sna.begin(), sna.end());
    bus.queueRead(first);
    bus.queueRead(second);
    SerialNumber serial = {0, 0, 0};

    CHECK(sensor.readSerial(serial) == true);
    CHECK(serial.a == 0xF00F);
    CHECK(serial.b == 0xFEDCBA98);
    CHECK(serial.c == 0x8001);
  }

  TEST_CASE_FIXTURE(BusFixture, "a wrong CRC in any position is detected") {
    SHT2x sensor(bus);
    SerialNumber serial = {0, 0, 0};

    for (size_t position = 0; position < 4; position++) {
      CAPTURE(position);
      Bytes first = firstAccess();
      first[2 * position + 1] ^= 0x01;
      bus.queueRead(first);
      bus.queueRead(secondAccess());
      CHECK(sensor.readSerial(serial) == false);
    }
    for (size_t position = 0; position < 2; position++) {
      CAPTURE(position);
      Bytes second = secondAccess();
      second[3 * position + 2] ^= 0x01;
      bus.queueRead(firstAccess());
      bus.queueRead(second);
      CHECK(sensor.readSerial(serial) == false);
    }
  }

  TEST_CASE_FIXTURE(BusFixture, "a missing sensor is not mistaken for serial number 0 (the CRC of zeros is zero)") {
    SHT2x sensor(bus);
    SerialNumber serial = {0, 0, 0};
    CHECK(sensor.readSerial(serial) == false);
  }

  TEST_CASE_FIXTURE(BusFixture, "short reads are detected") {
    SHT2x sensor(bus);
    SerialNumber serial = {0, 0, 0};

    SUBCASE("first access does not answer") {
      bus.queueRead(Bytes());
      bus.queueRead(secondAccess());
    }
    SUBCASE("second access does not answer") {
      bus.queueRead(firstAccess());
    }
    SUBCASE("first access is too short") {
      bus.queueRead(Bytes(6, 0x00));
      bus.queueRead(secondAccess());
    }
    SUBCASE("second access is too short") {
      bus.queueRead(firstAccess());
      bus.queueRead(Bytes(5, 0x00));
    }

    CHECK(sensor.readSerial(serial) == false);
  }

  TEST_CASE_FIXTURE(BusFixture, "commands that are not acknowledged are detected") {
    SHT2x sensor(bus);
    SerialNumber serial = {0, 0, 0};
    bus.queueRead(firstAccess());
    bus.queueRead(secondAccess());

    SUBCASE("first command") {
      bus.queueWriteStatus(2);
    }
    SUBCASE("second command") {
      bus.queueWriteStatus(0);
      bus.queueWriteStatus(2);
    }

    CHECK(sensor.readSerial(serial) == false);
  }
}

TEST_SUITE("SHT2x bus handling") {
  TEST_CASE_FIXTURE(BusFixture, "the sensor uses the bus it was given") {
    TwoWire otherBus;
    SHT2x sensor(otherBus);
    otherBus.queueRead(frame(0x6838));
    bus.queueRead(frame(0x0000));

    CHECK(sensor.readTemp() == doctest::Approx(celsius(0x6838)).epsilon(1e-6));
    checkQuery(0, otherBus, bytes(0xF3), 85, 3);
  }

  TEST_CASE_FIXTURE(BusFixture, "writeCommand reports success only if all bytes were queued and acknowledged") {
    ExposedSHT2x sensor(bus);
    uint8_t payload[] = {0x12, 0x34};

    SUBCASE("success without payload") {
      CHECK(sensor.writeCommand(0xFE) == true);
      checkWrite(0, bus, bytes(0xFE));
    }
    SUBCASE("success with payload") {
      CHECK(sensor.writeCommand(0xE6, payload, sizeof(payload)) == true);
      Bytes expected = bytes(0xE6, 0x12);
      expected.push_back(0x34);
      checkWrite(0, bus, expected);
    }
    SUBCASE("transmit buffer too small for the command") {
      bus.setWriteLimit(0);
      CHECK(sensor.writeCommand(0xFE) == false);
    }
    SUBCASE("transmit buffer too small for the payload") {
      bus.setWriteLimit(2);
      CHECK(sensor.writeCommand(0xE6, payload, sizeof(payload)) == false);
    }
    SUBCASE("NACK on address") {
      bus.queueWriteStatus(2);
      CHECK(sensor.writeCommand(0xFE) == false);
    }
    SUBCASE("NACK on data") {
      bus.queueWriteStatus(3);
      CHECK(sensor.writeCommand(0xE6, payload, sizeof(payload)) == false);
    }
    SUBCASE("other bus error") {
      bus.queueWriteStatus(4);
      CHECK(sensor.writeCommand(0xFE) == false);
    }

    // The transaction is always terminated
    CHECK(eventCount() == 1);
  }

  TEST_CASE_FIXTURE(BusFixture, "queryCommand does not read if the transmit buffer rejects the command") {
    ExposedSHT2x sensor(bus);
    bus.setWriteLimit(1);
    bus.queueRead(Bytes(8, 0x00));
    uint8_t result[8] = {0};

    CHECK(sensor.queryCommand(static_cast<uint16_t>(0xFA0F), result, sizeof(result)) == false);
    CHECK(eventCount() == 1);
  }

  TEST_CASE_FIXTURE(BusFixture, "queryCommand reports the number of bytes read") {
    ExposedSHT2x sensor(bus);
    uint8_t result[2] = {0, 0};

    SUBCASE("complete") {
      bus.queueRead(bytes(0xAB, 0xCD));
      CHECK(sensor.queryCommand(static_cast<uint8_t>(0xE7), result, sizeof(result), 7) == true);
      checkQuery(0, bus, bytes(0xE7), 7, 2);
      CHECK(result[0] == 0xAB);
      CHECK(result[1] == 0xCD);
    }
    SUBCASE("short") {
      bus.queueRead(bytes(0xAB));
      CHECK(sensor.queryCommand(static_cast<uint8_t>(0xE7), result, sizeof(result)) == false);
      CHECK(result[0] == 0xAB);
      CHECK(result[1] == 0x00);
    }
    SUBCASE("the device sends more than requested") {
      Bytes response = bytes(0xAB, 0xCD);
      response.push_back(0xEF);
      bus.queueRead(response);
      CHECK(sensor.queryCommand(static_cast<uint8_t>(0xE7), result, sizeof(result)) == true);
      CHECK(mock::events()[2].data == bytes(0xAB, 0xCD));
    }
  }
}
