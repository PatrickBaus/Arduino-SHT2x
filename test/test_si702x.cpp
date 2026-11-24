#include <doctest/doctest.h>

#include <cmath>

#include "SHT2x.h"
#include "helpers.h"

TEST_SUITE("Si702x measurements") {
  TEST_CASE_FIXTURE(BusFixture, "temperature, no hold master: command 0xF3, 11 ms conversion time") {
    Si702x sensor(bus);
    bus.queueRead(frame(0x6838));

    const float temperature = sensor.readTemp();

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xF3), 11, 3);
    CHECK(temperature == doctest::Approx(celsius(0x6838)).epsilon(1e-6));
  }

  TEST_CASE_FIXTURE(BusFixture, "temperature, hold master: command 0xE3") {
    Si702x sensor(bus);
    bus.queueRead(frame(0x6838));

    const float temperature = sensor.readTemp(true);

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xE3), 11, 3);
    CHECK(temperature == doctest::Approx(celsius(0x6838)).epsilon(1e-6));
  }

  TEST_CASE_FIXTURE(BusFixture, "humidity, no hold master: command 0xF5, 12 ms + 11 ms conversion time") {
    Si702x sensor(bus);
    // The Si7021 datasheet defines the conversion on the complete 16 bit word, there are no status bits to remove
    bus.queueRead(frame(0x7C82));

    const float humidity = sensor.readHumidity();

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xF5), 12 + 11, 3);
    CHECK(humidity == doctest::Approx(relativeHumidity(0x7C82)).epsilon(1e-6));
  }

  TEST_CASE_FIXTURE(BusFixture, "humidity, hold master: command 0xE5") {
    Si702x sensor(bus);
    bus.queueRead(frame(0x7C82));

    const float humidity = sensor.readHumidity(true);

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xE5), 12 + 11, 3);
    CHECK(humidity == doctest::Approx(relativeHumidity(0x7C82)).epsilon(1e-6));
  }

  TEST_CASE_FIXTURE(BusFixture, "conversions use 2^16 as the divisor") {
    Si702x sensor(bus);
    bus.queueRead(frame(0xFFFC));
    CHECK(std::fabs(sensor.readTemp() - celsius(0xFFFC)) < 1e-4);
    bus.queueRead(frame(0xFFFE));
    CHECK(std::fabs(sensor.readHumidity() - relativeHumidity(0xFFFE)) < 1e-4);
  }

  TEST_CASE_FIXTURE(BusFixture, "raw values are returned through the base class interface") {
    Si702x sensor(bus);
    bus.queueRead(frame(0x6838));
    bus.queueRead(frame(0x7C82));

    CHECK(sensor.readTempRaw() == 0x6838);
    CHECK(sensor.readHumidityRaw(true) == 0x7C82);
    checkWrite(3, bus, bytes(0xE5));
  }

  TEST_CASE_FIXTURE(BusFixture, "a wrong CRC is reported as NaN") {
    Si702x sensor(bus);
    Bytes response = frame(0x6838);
    response[2] ^= 0x01;
    bus.queueRead(response);
    CHECK(std::isnan(sensor.readTemp()));
    bus.queueRead(response);
    CHECK(std::isnan(sensor.readHumidity()));
  }

  TEST_CASE_FIXTURE(BusFixture, "a missing sensor is reported as NaN") {
    Si702x sensor(bus);
    CHECK(std::isnan(sensor.readTemp()));
    CHECK(std::isnan(sensor.readTemp(true)));
    CHECK(std::isnan(sensor.readHumidity()));
    CHECK(std::isnan(sensor.readHumidity(true)));
  }

  TEST_CASE_FIXTURE(BusFixture, "a short read is reported as NaN") {
    Si702x sensor(bus);
    bus.queueRead(bytes(0x00, 0x00));
    CHECK(std::isnan(sensor.readTemp()));
    bus.queueRead(bytes(0x00, 0x00));
    CHECK(std::isnan(sensor.readHumidity()));
  }

  TEST_CASE_FIXTURE(BusFixture, "a command that is not acknowledged is reported as NaN") {
    Si702x sensor(bus);
    bus.queueWriteStatus(2);
    bus.queueRead(frame(0x6838));
    CHECK(std::isnan(sensor.readTemp()));
    CHECK(eventCount() == 1);

    bus.queueWriteStatus(2);
    CHECK(std::isnan(sensor.readHumidity()));
    CHECK(eventCount() == 2);
  }
}

TEST_SUITE("Si702x temperature of the last humidity measurement") {
  TEST_CASE_FIXTURE(BusFixture, "fetchTemp: command 0xE0, 2 byte read without checksum and without delay") {
    Si702x sensor(bus);
    bus.queueRead(bytes(0x68, 0x38));

    const float temperature = sensor.fetchTemp();

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xE0), 0, 2);
    CHECK(temperature == doctest::Approx(celsius(0x6838)).epsilon(1e-6));
  }

  TEST_CASE_FIXTURE(BusFixture, "fetchTempRaw returns the 16 bit word") {
    Si702x sensor(bus);
    bus.queueRead(bytes(0xFF, 0xFC));
    CHECK(sensor.fetchTempRaw() == 0xFFFC);
    bus.queueRead(bytes(0x80, 0x01));
    CHECK(sensor.fetchTempRaw() == 0x8001);
  }

  TEST_CASE_FIXTURE(BusFixture, "a missing sensor or a short read is reported as NaN") {
    Si702x sensor(bus);

    SUBCASE("no answer") {}
    SUBCASE("short read") { bus.queueRead(bytes(0x68)); }
    SUBCASE("command not acknowledged") {
      bus.queueWriteStatus(2);
      bus.queueRead(bytes(0x68, 0x38));
    }

    CHECK(std::isnan(sensor.fetchTemp()));
  }

  TEST_CASE_FIXTURE(BusFixture, "fetchTempRaw returns 0 without a sensor") {
    Si702x sensor(bus);
    CHECK(sensor.fetchTempRaw() == 0);
  }
}

TEST_SUITE("Si702x firmware revision") {
  TEST_CASE_FIXTURE(BusFixture, "command 0x84 0xB8, single byte read") {
    Si702x sensor(bus);
    bus.queueRead(bytes(0xFF));

    sensor.readFirmwareVersion();

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0x84, 0xB8), 0, 1);
  }

  TEST_CASE_FIXTURE(BusFixture, "0xFF is firmware version 1.0, 0x20 is firmware version 2.0") {
    Si702x sensor(bus);
    bus.queueRead(bytes(0xFF));
    CHECK(sensor.readFirmwareVersion() == 1);
    bus.queueRead(bytes(0x20));
    CHECK(sensor.readFirmwareVersion() == 2);
  }

  TEST_CASE_FIXTURE(BusFixture, "unknown revisions are returned unchanged, a missing sensor reads as 0") {
    Si702x sensor(bus);
    bus.queueRead(bytes(0x30));
    CHECK(sensor.readFirmwareVersion() == 0x30);
    CHECK(sensor.readFirmwareVersion() == 0);
  }
}

TEST_SUITE("Si702x heater control register") {
  TEST_CASE_FIXTURE(BusFixture, "getHeaterPower: command 0x11, returns the lower nibble") {
    Si702x sensor(bus);
    bus.queueRead(bytes(0xA7));

    CHECK(sensor.getHeaterPower() == 0x07);

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0x11), 0, 1);

    bus.queueRead(bytes(0x0F));
    CHECK(sensor.getHeaterPower() == 0x0F);
    CHECK(sensor.getHeaterPower() == 0);  // no sensor
  }

  TEST_CASE_FIXTURE(BusFixture, "setHeaterPower: command 0x51, only the lower nibble is changed") {
    Si702x sensor(bus);

    SUBCASE("reserved bits set") {
      bus.queueRead(bytes(0xA0));
      sensor.setHeaterPower(0x05);
      REQUIRE(eventCount() == 4);
      checkQuery(0, bus, bytes(0x11), 0, 1);
      checkWrite(3, bus, bytes(0x51, 0xA5));
    }
    SUBCASE("previous value is replaced") {
      bus.queueRead(bytes(0x0F));
      sensor.setHeaterPower(0x00);
      checkWrite(3, bus, bytes(0x51, 0x00));
    }
    SUBCASE("maximum") {
      bus.queueRead(bytes(0x00));
      sensor.setHeaterPower(0x0F);
      checkWrite(3, bus, bytes(0x51, 0x0F));
    }
    SUBCASE("values above the maximum are clamped and do not touch the reserved bits") {
      bus.queueRead(bytes(0x00));
      sensor.setHeaterPower(0xF0);
      checkWrite(3, bus, bytes(0x51, 0x0F));
    }
    SUBCASE("16 is clamped") {
      bus.queueRead(bytes(0x50));
      sensor.setHeaterPower(16);
      checkWrite(3, bus, bytes(0x51, 0x5F));
    }
  }

  TEST_CASE_FIXTURE(BusFixture, "nothing is written if the register cannot be read, the reserved bits must not be overwritten") {
    Si702x sensor(bus);

    SUBCASE("sensor does not answer") {
      sensor.setHeaterPower(0x05);
      REQUIRE(eventCount() == 3);
      checkQuery(0, bus, bytes(0x11), 0, 1);
    }
    SUBCASE("command not acknowledged") {
      bus.queueWriteStatus(2);
      bus.queueRead(bytes(0xA0));
      sensor.setHeaterPower(0x05);
      REQUIRE(eventCount() == 1);
      checkWrite(0, bus, bytes(0x11));
    }
  }
}

TEST_SUITE("Si702x inherited functions") {
  TEST_CASE_FIXTURE(BusFixture, "the heater is enabled through the user register") {
    Si702x sensor(bus);
    bus.queueRead(bytes(0x3A));

    sensor.setHeaterState(true);

    REQUIRE(eventCount() == 4);
    checkQuery(0, bus, bytes(0xE7), 0, 1);
    checkWrite(3, bus, bytes(0xE6, 0x3E));
  }

  TEST_CASE_FIXTURE(BusFixture, "the timing of the sensor is used when called through the base class") {
    Si702x si(bus);
    SHT2x& sensor = si;
    bus.queueRead(frame(0x6838));
    bus.queueRead(frame(0x7C82));

    sensor.readTemp();
    sensor.readHumidity();

    REQUIRE(eventCount() == 6);
    checkDelay(1, 11);
    checkDelay(4, 12 + 11);
  }
}
