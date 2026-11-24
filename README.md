Sensirion SHT2x Arduino Library
===================

Note: These devices are all NRD except for the TE parts.

This repository contains an Arduino library for the following Sensirion devices:
 * [SHT20](https://sensirion.com/products/catalog/SHT20/)
 * [SHT21](https://sensirion.com/products/catalog/SHT21/)
 * [SHT21A](https://sensirion.com/products/catalog/SHT21A/)
 * [SHT25](https://sensirion.com/products/catalog/SHT25/)
The following Silicon Labs devices:
 * [Si7006](https://www.silabs.com/sensors/humidity/si7006-13-20-21-34/device.si7006-a20-im)
 * [Si7020](https://www.silabs.com/sensors/humidity/si7006-13-20-21-34/device.si7020-a20-im)
 * [Si7021](https://www.silabs.com/sensors/humidity/si7006-13-20-21-34/device.si7021-a20-gm)
The following TE Connectivity (Measurement Specialties) devices:
 * [HTU20](https://www.te.com/usa-en/product-CAT-HSC0002.html)
 * [HTU21](https://www.te.com/usa-en/product-CAT-HSC0005.html)

The library uses I²C transactions and allows to select the I²C controller if there is more than one.

Usage
-----
```cpp
#include <Wire.h>   // #include <i2c_t3.h> on the Teensy platform.
#include "src/SHT2x.h"

SHT2x sensor(Wire);
//HTU2x sensor(Wire);
//Si702x sensor(Wire);

void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  // The call to readTemp() will automatically block for the time the measurement takes to complete
  Serial.print(sensor.readTemp());  // Read the temperarture as a float in °C
  Serial.println(" °C");
  Serial.print(sensor.readHumdity());  // Read the humidity as a float in %rH
  Serial.println(" %rH");
  delay(2000);
}
```

Installation
-----
Currently the library does not support the Arduino library manager, so it is highly recommended to copy the full library to a subfolder called
```
src/
```
within your Arduino project.

Tests
-----
The library is tested on the host (no hardware required) against a fake of the Arduino `Wire` API, which records the bus traffic and
plays back scripted sensor responses. The tests live in [test/](test/), the fakes in [test/mocks/](test/mocks/). They use
[doctest](https://github.com/doctest/doctest), which is downloaded by CMake. To build and run them you need CMake (>= 3.14), a C++11
compiler (GCC) and, for the coverage report, [gcovr](https://gcovr.com/):
```
cmake -S test -B build
cmake --build build
ctest --test-dir build --output-on-failure
gcovr --root . --filter src/ --exclude-throw-branches --exclude-unreachable-branches --fail-under-line 100 --fail-under-branch 100 build
```
The last command prints the line coverage of `src/` and fails if the line or branch coverage drops below 100 %.
