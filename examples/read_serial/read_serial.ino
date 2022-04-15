#include <Wire.h>   // #include <i2c_t3.h> on the Teensy platform.
#include "src/SHT2x.h"

SHT2x sensor(Wire);
//HTU2x sensor(Wire);
//Si702x sensor(Wire);

void setup() {
  Wire.begin();
  Serial.begin(115200);
  SerialNumber serialNumber;
  sensor.readSerial(serialNumber);
  Serial.print("Serial A: ");
  Serial.println(serialNumber.a);
  Serial.print("Serial B: ");
  Serial.println(serialNumber.b);
  Serial.print("Serial C: ");
  Serial.println(serialNumber.c);
}

void loop() {

}
