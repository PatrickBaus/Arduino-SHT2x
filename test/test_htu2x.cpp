#include <doctest/doctest.h>

#include <cmath>

#include "SHT2x.h"
#include "helpers.h"

TEST_SUITE("HTU2x") {
  TEST_CASE_FIXTURE(BusFixture, "temperature, no hold master: command 0xF3, 50 ms conversion time") {
    HTU2x sensor(bus);
    bus.queueRead(frame(0x683A));

    const float temperature = sensor.readTemp();

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xF3), 50, 3);
    CHECK(temperature == doctest::Approx(celsius(0x6838)).epsilon(1e-6));
  }

  TEST_CASE_FIXTURE(BusFixture, "temperature, hold master: command 0xE3") {
    HTU2x sensor(bus);
    bus.queueRead(frame(0x6838));

    const float temperature = sensor.readTemp(true);

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xE3), 50, 3);
    CHECK(temperature == doctest::Approx(celsius(0x6838)).epsilon(1e-6));
  }

  TEST_CASE_FIXTURE(BusFixture, "humidity, no hold master: command 0xF5, 16 ms + 50 ms conversion time") {
    HTU2x sensor(bus);
    // Example of the HTU21D datasheet: 0x7C80 -> 54.8 %RH
    bus.queueRead(frame(0x7C82));

    const float humidity = sensor.readHumidity();

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xF5), 16 + 50, 3);
    CHECK(humidity == doctest::Approx(relativeHumidity(0x7C80)).epsilon(1e-6));
    CHECK(humidity == doctest::Approx(54.8).epsilon(1e-3));
  }

  TEST_CASE_FIXTURE(BusFixture, "humidity, hold master: command 0xE5") {
    HTU2x sensor(bus);
    bus.queueRead(frame(0x7C82));

    const float humidity = sensor.readHumidity(true);

    REQUIRE(eventCount() == 3);
    checkQuery(0, bus, bytes(0xE5), 16 + 50, 3);
    CHECK(humidity == doctest::Approx(relativeHumidity(0x7C80)).epsilon(1e-6));
  }

  TEST_CASE_FIXTURE(BusFixture, "the public raw overloads return the value without status bits and report the CRC check") {
    HTU2x sensor(bus);
    uint16_t value = 0;

    bus.queueRead(frame(0xABCF));
    CHECK(sensor.readTempRaw(value) == true);
    CHECK(value == 0xABCC);

    bus.queueRead(frame(0x1236));
    CHECK(sensor.readHumidityRaw(value, true) == true);
    CHECK(value == 0x1234);

    Bytes broken = frame(0xABCC);
    broken[2] ^= 0x80;
    bus.queueRead(broken);
    CHECK(sensor.readTempRaw(value, true) == false);
    bus.queueRead(broken);
    CHECK(sensor.readHumidityRaw(value) == false);
  }

  TEST_CASE_FIXTURE(BusFixture, "a wrong CRC is reported as NaN") {
    HTU2x sensor(bus);
    Bytes response = frame(0x6838);
    response[2] ^= 0x01;
    bus.queueRead(response);
    CHECK(std::isnan(sensor.readTemp()));
    bus.queueRead(response);
    CHECK(std::isnan(sensor.readHumidity()));
  }

  TEST_CASE_FIXTURE(BusFixture, "a missing sensor is reported as NaN") {
    HTU2x sensor(bus);
    CHECK(std::isnan(sensor.readTemp()));
    CHECK(std::isnan(sensor.readTemp(true)));
    CHECK(std::isnan(sensor.readHumidity()));
    CHECK(std::isnan(sensor.readHumidity(true)));
  }

  TEST_CASE_FIXTURE(BusFixture, "a short read is reported as NaN") {
    HTU2x sensor(bus);
    bus.queueRead(bytes(0x00, 0x00));
    CHECK(std::isnan(sensor.readTemp()));
    bus.queueRead(bytes(0x00, 0x00));
    CHECK(std::isnan(sensor.readHumidity()));
  }

  TEST_CASE_FIXTURE(BusFixture, "a command that is not acknowledged is reported as NaN") {
    HTU2x sensor(bus);
    bus.queueWriteStatus(2);
    bus.queueRead(frame(0x6838));
    CHECK(std::isnan(sensor.readTemp()));
    CHECK(eventCount() == 1);

    bus.queueWriteStatus(2);
    CHECK(std::isnan(sensor.readHumidity()));
    CHECK(eventCount() == 2);
  }

  TEST_CASE_FIXTURE(BusFixture, "the timing of the sensor is used when called through the base class") {
    HTU2x htu(bus);
    SHT2x& sensor = htu;
    bus.queueRead(frame(0x6838));
    bus.queueRead(frame(0x7C82));

    CHECK(sensor.readTempRaw() == 0x6838);
    CHECK(sensor.readHumidityRaw() == 0x7C80);

    REQUIRE(eventCount() == 6);
    checkDelay(1, 50);
    checkDelay(4, 16 + 50);
  }

  TEST_CASE_FIXTURE(BusFixture, "user register and soft reset are inherited from the SHT2x") {
    HTU2x sensor(bus);
    bus.queueRead(bytes(0x02));

    sensor.setResolution(RH11_T11);
    sensor.reset();

    REQUIRE(eventCount() == 6);
    checkQuery(0, bus, bytes(0xE7), 0, 1);
    checkWrite(3, bus, bytes(0xE6, 0x83));
    checkWrite(4, bus, bytes(0xFE));
    checkDelay(5, 15);
  }
}
