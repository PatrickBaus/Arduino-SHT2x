#include <Wire.h>   // #include <i2c_t3.h> on the Teensy platform.
#include "src/SHT2x.h"

SHT2x sensor(Wire);
//HTU2x sensor(Wire);
//Si702x sensor(Wire);

void setup() {
  Wire.begin();
  Serial.begin(115200);
  sensor.reset();
  sensor.setResolution(RH12_T14);   //Sets the resolution for the sensor to 14 bit temperature resolution and 12 bit humidity resolution.
}

void loop() {
  // The call to readTemp() will automatically block for the time the measurement takes to complete
  Serial.print(sensor.readTemp());  // Read the temperarture as a float in °C
  Serial.println(" °C");
  Serial.print(sensor.readHumidity());  // Read the humidity as a float in %rH
  Serial.println(" %rH");
  delay(2000);
}
