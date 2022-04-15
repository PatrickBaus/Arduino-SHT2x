/*
 *  The SHT2x library is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "SHT2x.h"

#if defined(__arm__) && defined(TEENSYDUINO)
SHT2x::SHT2x(i2c_t3& i2cBus) : _pI2cBus(&i2cBus) {};
#else
SHT2x::SHT2x(TwoWire& i2cBus) : _pI2cBus(&i2cBus) {};
#endif

uint8_t SHT2x::crc8(const uint8_t* data, const size_t len) {
  // Calculate CRC with the following parameters (datasheet p. 10):
  // Type: CRC-8
  // Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
  // Reflextion in/out: false/false
  // Init: 0x00
  // Final XOR: 0x00

  uint8_t crc = 0x00; // init to starting value
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t crcBit = 8; crcBit > 0; crcBit--) {
      // Apply the polynomial
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
  }
  return crc; // No final XOR needed since it is 0x00;
}

float SHT2x::convertToCelsius(const uint16_t value) {
  return ((float)value * 175.72) / 65535.0f - 46.85f;
}

float SHT2x::convertToRh(const uint16_t value) {
  return ((float)value * 125) / 65535.0f - 6.f;
}

bool SHT2x::writeCommand(const uint8_t* cmd, const size_t cmdLen, uint8_t* data, const size_t len) {
  _pI2cBus->beginTransmission(I2C_ADDRESS);
  bool result = _pI2cBus->write(cmd, cmdLen) == cmdLen;
  if (len) {
    result = result & (_pI2cBus->write(data, len) == len);
  }
  _pI2cBus->endTransmission();
  return result;
}

bool SHT2x::writeCommand(const uint8_t command, uint8_t* data, const size_t len) {
  uint8_t cmd[1];

  cmd[0] = command;

  return this->writeCommand(cmd, sizeof(cmd), data, len);
}

bool SHT2x::writeCommand(const uint16_t command, uint8_t* data, const size_t len) {
  uint8_t cmd[2];

  cmd[0] = command >> 8;
  cmd[1] = command & 0xFF;

  return this->writeCommand(cmd, sizeof(cmd), data, len);
}

bool SHT2x::delayedRead(uint8_t* result, const size_t len, const uint16_t delayms) {
  delay(delayms);

  _pI2cBus->requestFrom(I2C_ADDRESS, len);
  size_t numberOfBytesread = _pI2cBus->readBytes(result, len);

  if (numberOfBytesread == len) {
    return true;
  }
  return false;
}

bool SHT2x::queryCommand(const uint8_t command, uint8_t* result, const size_t len, const uint16_t delayms) {
  this->writeCommand(command);
  return delayedRead(result, len, delayms);
}

bool SHT2x::queryCommand(const uint16_t command, uint8_t* result, const size_t len, const uint16_t delayms) {
  this->writeCommand(command);
  return delayedRead(result, len, delayms);
}

bool SHT2x::readSerial(SerialNumber& serial) {
  uint8_t data[8] = {0};
  // Read the first register
  this->queryCommand(SHT2x::CMD_READ_SERIAL_1, data, sizeof(data));
  serial.b = data[0] << 24 | data[2] << 16 | data[4] << 8 | data[6] << 0;
  // Test CRC
  bool result =
       SHT2x::crc8(&(data[0]), 1) == data[1]
    && SHT2x::crc8(&(data[2]), 1) == data[3]
    && SHT2x::crc8(&(data[4]), 1) == data[5]
    && SHT2x::crc8(&(data[6]), 1) == data[7]
  ;

  // Read the second register (only 6 bytes)
  this->queryCommand(SHT2x::CMD_READ_SERIAL_2, data, 6);
  serial.c = data[0] << 8 | data[1] << 0;
  serial.b = data[3] << 8 | data[4] << 0;
  // Test CRC
  result =
       result
    && SHT2x::crc8(&(data[0]), 2) == data[2]
    && SHT2x::crc8(&(data[3]), 2) == data[5]
  ;

  return result;
}

void SHT2x::reset(void) {
  this->writeCommand(SHT2x::CMD_SOFT_RESET);
  delay(15); // The sensor needs 15 ms max to return to idle (taken from the Si7021 datasheet p. 5)
}

float SHT2x::readTemp(const bool holdMaster) {
  uint16_t value;
  if (this->readTempRaw(value, holdMaster)) {
    return SHT2x::convertToCelsius(value);
  } else {
    return nan("");
  }
}

uint16_t SHT2x::readTempRaw(const bool holdMaster) {
  uint16_t result;
  this->readTempRaw(result, holdMaster);
  return result;
}

bool SHT2x::readTempRaw(uint16_t &value, const bool holdMaster) {
  uint8_t data[3] = {0};
  if (holdMaster) {
    this->queryCommand(SHT2x::CMD_MEASURE_TEMPERATURE_HOLD_MASTER, data, sizeof(data), 85 /*ms*/);
  } else {
    this->queryCommand(SHT2x::CMD_MEASURE_TEMPERATURE, data, sizeof(data), 85 /*ms*/);
  }
  value = data[0] << 8 | data[1] << 0;
  return SHT2x::crc8(data, 2) == data[2];
}

float SHT2x::readHumidity(const bool holdMaster) {
  uint16_t value;
  if (this->readHumidityRaw(value, holdMaster)) {
    return SHT2x::convertToRh(value);
  } else {
    return nan("");
  }
}

uint16_t SHT2x::readHumidityRaw(const bool holdMaster) {
  uint16_t result;
  this->readHumidityRaw(result, holdMaster);
  return result;
}

bool SHT2x::readHumidityRaw(uint16_t &value, const bool holdMaster) {
  uint8_t data[3] = {0};
  if (holdMaster) {
    this->queryCommand(SHT2x::CMD_MEASURE_HUMIDITY_HOLD_MASTER, data, sizeof(data), 29+85 /*ms*/);
  } else {
    this->queryCommand(SHT2x::CMD_MEASURE_HUMIDITY, data, sizeof(data), 29+85 /*ms*/);
  }
  value = data[0] << 8 | data[1] << 0;
  return SHT2x::crc8(data, 2) == data[2];
}

bool SHT2x::readUserRegister(uint8_t& userRegister) {
  uint8_t data[1] = {0};
  bool result = this->queryCommand(SHT2x::CMD_READ_USER_REGISTER, data, sizeof(data));
  userRegister = data[0];

  return result;
}

bool SHT2x::writeUserRegister(uint8_t userRegister) {
  uint8_t data[1] = {userRegister};
  bool result = this->writeCommand(SHT2x::CMD_WRITE_USER_REGISTER, data, sizeof(data));

  return result;
}

void SHT2x::setHeaterState(const bool enabled) {
  uint8_t userReg;
  this->readUserRegister(userReg);
  userReg = (userReg & ~SHT2x::STATUS_HEATER) | enabled * SHT2x::STATUS_HEATER;
  this->writeUserRegister(userReg);
}

bool SHT2x::getHeaterState(void) {
  uint8_t userReg;
  this->readUserRegister(userReg);
  return userReg & SHT2x::STATUS_HEATER;
}

void SHT2x::setOtpReloadStatus(const bool enabled) {
  uint8_t userReg;
  this->readUserRegister(userReg);
  userReg = (userReg & ~SHT2x::STATUS_OTP_RELOAD) | !enabled * SHT2x::STATUS_OTP_RELOAD;
  this->writeUserRegister(userReg);
}

bool SHT2x::getOtpReloadStatus(void) {
  uint8_t userReg;
  this->readUserRegister(userReg);
  return !(userReg & SHT2x::STATUS_OTP_RELOAD);
}

bool SHT2x::getVddLowStatus(void) {
  uint8_t userReg;
  this->readUserRegister(userReg);
  return userReg & SHT2x::STATUS_END_OF_BATTERY;
}

void SHT2x::setResolution(const Resolution resolution) {
  uint8_t userReg = resolution;
  this->readUserRegister(userReg);
  userReg = (userReg & ~RH11_T11) | resolution;
  this->writeUserRegister(userReg);
}

Resolution SHT2x::getResolution(void) {
  uint8_t userReg;
  this->readUserRegister(userReg);
  return Resolution(userReg & RH11_T11);
}

uint8_t SHT2x::convertResTemperature(Resolution resolution) {
  switch (resolution) {
    case RH12_T14:
      return 14;
    case RH8_T12:
      return 12;
    case RH10_T13:
      return 13;
    case RH11_T11:
      return 11;
    default:
      return 14;  // Will never be reached
  }
}

uint8_t SHT2x::convertResHumidity(Resolution resolution) {
  switch (resolution) {
    case RH12_T14:
      return 12;
    case RH8_T12:
      return 8;
    case RH10_T13:
      return 10;
    case RH11_T11:
      return 11;
    default:
      return 12;  // Will never be reached
  }
}


#if defined(__arm__) && defined(TEENSYDUINO)
HTU2x::HTU2x(i2c_t3& i2cBus) : SHT2x(i2cBus) {};
#else
HTU2x::HTU2x(TwoWire& i2cBus) : SHT2x(i2cBus) {};
#endif

bool HTU2x::readHumidityRaw(uint16_t &value, const bool holdMaster) {
  uint8_t data[3] = {0};
  if (holdMaster) {
    this->queryCommand(SHT2x::CMD_MEASURE_HUMIDITY_HOLD_MASTER, data, sizeof(data), 16+50 /*ms*/);
  } else {
    this->queryCommand(SHT2x::CMD_MEASURE_HUMIDITY, data, sizeof(data), 16+50 /*ms*/);
  }
  value = data[0] << 8 | data[1] << 0;
  return SHT2x::crc8(data, 2) == data[2];
}

bool HTU2x::readTempRaw(uint16_t &value, const bool holdMaster) {
  uint8_t data[3] = {0};
  if (holdMaster) {
    this->queryCommand(SHT2x::CMD_MEASURE_TEMPERATURE_HOLD_MASTER, data, sizeof(data), 50 /*ms*/);
  } else {
    this->queryCommand(SHT2x::CMD_MEASURE_TEMPERATURE, data, sizeof(data), 50 /*ms*/);
  }
  value = data[0] << 8 | data[1] << 0;
  return SHT2x::crc8(data, 2) == data[2];
}


#if defined(__arm__) && defined(TEENSYDUINO)
Si702x::Si702x(i2c_t3& i2cBus) : SHT2x(i2cBus) {};
#else
Si702x::Si702x(TwoWire& i2cBus) : SHT2x(i2cBus) {};
#endif

uint8_t Si702x::readFirmwareVersion(void) {
  uint8_t data[1] = {0};
  this->queryCommand(Si702x::CMD_READ_FIRMWARE, data, sizeof(data));

  if (data[0] == 0xFF) {
    return 1;
  } else if (data[0] == 0x20) {
    return 2;
  }
  return data[0];
}

bool Si702x::readHumidityRaw(uint16_t &value, const bool holdMaster) {
  uint8_t data[3] = {0};
  if (holdMaster) {
    this->queryCommand(SHT2x::CMD_MEASURE_HUMIDITY_HOLD_MASTER, data, sizeof(data), 12+11 /*ms*/);
  } else {
    this->queryCommand(SHT2x::CMD_MEASURE_HUMIDITY, data, sizeof(data), 12+11 /*ms*/);
  }
  value = data[0] << 8 | data[1] << 0;
  return SHT2x::crc8(data, 2) == data[2];
}

bool Si702x::readTempRaw(uint16_t &value, const bool holdMaster) {
  uint8_t data[3] = {0};
  if (holdMaster) {
    this->queryCommand(SHT2x::CMD_MEASURE_TEMPERATURE_HOLD_MASTER, data, sizeof(data), 11 /*ms*/);
  } else {
    this->queryCommand(SHT2x::CMD_MEASURE_TEMPERATURE, data, sizeof(data), 11 /*ms*/);
  }
  value = data[0] << 8 | data[1] << 0;
  return SHT2x::crc8(data, 2) == data[2];
}

float Si702x::fetchTemp(void) {
  uint16_t value;
  if (this->fetchTempRaw(value)) {
    return SHT2x::convertToCelsius(value);
  } else {
    return nan("");
  }
}

uint16_t Si702x::fetchTempRaw(void) {
  uint16_t value;
  this->fetchTempRaw(value);
  return value;
}

bool Si702x::fetchTempRaw(uint16_t &value) {
  uint8_t data[2] = {0};
  bool result = this->queryCommand(Si702x::CMD_FETCH_TEMPERATURE, data, sizeof(data));

  value = ((uint16_t)data[0] << 8) | data[1] << 0;
  return result;
}

bool Si702x::readHeaterControlRegister(uint8_t& reg) {
  uint8_t data[1] = {0};
  bool result = this->queryCommand(Si702x::CMD_READ_HEATER_REGISTER, data, sizeof(data));
  reg = data[0];

  return result;
}

bool Si702x::writeHeaterControlRegister(const uint8_t reg) {
  uint8_t data[1] = {reg};
  bool result = this->writeCommand(Si702x::CMD_WRITE_HEATER_REGISTER, data, sizeof(data));

  return result;
}

uint8_t Si702x::getHeaterPower(void) {
  uint8_t result;
  this->readHeaterControlRegister(result);
  return result & 0b1111;
};

void Si702x::setHeaterPower(uint8_t value) {
  uint8_t heaterRegister;
  this->readHeaterControlRegister(heaterRegister);
  heaterRegister &= 0xF0;
  value = value > 0b1111 ? 0b1111 : value;
  this->writeHeaterControlRegister(heaterRegister | value);
}
