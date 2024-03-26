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
#ifndef SHT2x_H
#define SHT2x_H

#include <stdint.h>
#include <stdbool.h>

// Match Teensy 3.6 (__MK66FX1M0__), Teensy 3.5 (__MK64FX512__), Teensy 3.2/3.1 (__MK20DX256__), Teensy 3.0 (__MK20DX128__), Teensy LC(__MKL26Z64__)
#if defined(__arm__) && defined(TEENSYDUINO) && (defined(__MK66FX1M0__) || defined(__MK64FX512__) || defined(__MK20DX256__) || defined(__MK20DX128__) || defined(__MKL26Z64__))
  #include <i2c_t3.h>
#else
  #include <Wire.h>
#endif

enum Resolution: uint8_t {
  RH12_T14 = (0 << 7) | (0 << 0),
  RH8_T12 = (0 << 7) | (1 << 0),
  RH10_T13 = (1 << 7) | (0 << 0),
  RH11_T11 = (1 << 7) | (1 << 0)
};

struct SerialNumber {
   uint16_t a;
   uint32_t b;
   uint16_t c;
};

class SHT2x {
  public:
    #if defined(__arm__) && defined(TEENSYDUINO) && (defined(__MK66FX1M0__) || defined(__MK64FX512__) || defined(__MK20DX256__) || defined(__MK20DX128__) || defined(__MKL26Z64__))
    SHT2x(i2c_t3& i2cBus);
    #else
    SHT2x(TwoWire& i2cBus);
    #endif
    void setHeaterState(const bool enabled=false);
    void reset(void);
    float readTemp(const bool holdMaster=false);
    uint16_t readTempRaw(const bool holdMaster=false);
    float readHumidity(const bool holdMaster=false);
    uint16_t readHumidityRaw(const bool holdMaster=false);
    bool readSerial(SerialNumber& serial);
    bool getHeaterState(void);
    void setOtpReloadStatus(const bool enabled=false);
    bool getOtpReloadStatus(void);
    bool getVddLowStatus(void);
    void setResolution(const Resolution resolution=RH12_T14);
    Resolution getResolution(void);
    static uint8_t convertResTemperature(Resolution resolution);
    static uint8_t convertResHumidity(Resolution resolution);
  private:
    #if defined(__arm__) && defined(TEENSYDUINO) && (defined(__MK66FX1M0__) || defined(__MK64FX512__) || defined(__MK20DX256__) || defined(__MK20DX128__) || defined(__MKL26Z64__))
    i2c_t3* _pI2cBus = NULL;
    #else
    TwoWire* _pI2cBus = NULL;
    #endif

    static const uint8_t I2C_ADDRESS = 0x40;
    static const uint8_t CMD_SOFT_RESET = 0xFE;
    static const uint16_t CMD_READ_SERIAL_1 = 0xFA0F;
    static const uint16_t CMD_READ_SERIAL_2 = 0xFCC9;
    static const uint8_t CMD_READ_USER_REGISTER = 0xE7;
    static const uint8_t  CMD_WRITE_USER_REGISTER = 0xE6;
    static const uint8_t STATUS_OTP_RELOAD = 1 << 1;
    static const uint8_t STATUS_HEATER = 1 << 2;
    static const uint8_t STATUS_END_OF_BATTERY = 1 << 6;
    static const uint8_t STATUS_RESOLUTION = (1 << 7) | (1 << 0);

    static float convertToRh(const uint16_t value);
    bool delayedRead(uint8_t* result, const size_t len, const uint16_t delayms);
    bool writeCommand(const uint16_t command, uint8_t* data=NULL, const size_t len=0);
    bool writeCommand(const uint8_t* cmd, const size_t cmdLen, uint8_t* data=NULL, const size_t len=0);
    virtual bool readTempRaw(uint16_t &value, const bool holdMaster=false);
    virtual bool readHumidityRaw(uint16_t &value, const bool holdMaster=false);
    bool readUserRegister(uint8_t& userRegister);
    bool writeUserRegister(const uint8_t userRegister);
protected:
    static const uint8_t CMD_MEASURE_TEMPERATURE = 0xF3;
    static const uint8_t CMD_MEASURE_TEMPERATURE_HOLD_MASTER = 0xE3;
    static const uint8_t CMD_MEASURE_HUMIDITY = 0xF5;
    static const uint8_t CMD_MEASURE_HUMIDITY_HOLD_MASTER = 0xE5;

    static float convertToCelsius(const uint16_t value);
    bool writeCommand(const uint8_t command, uint8_t* data=NULL, const size_t len=0);
    bool queryCommand(const uint8_t command, uint8_t* result, const size_t len, const uint16_t delayms=0);
    bool queryCommand(const uint16_t command, uint8_t* result, const size_t len, const uint16_t delayms=0);
    static uint8_t crc8(const uint8_t* data, const size_t len);
};

class HTU2x: public SHT2x {
  public:
    #if defined(__arm__) && defined(TEENSYDUINO) && (defined(__MK66FX1M0__) || defined(__MK64FX512__) || defined(__MK20DX256__) || defined(__MK20DX128__) || defined(__MKL26Z64__))
    HTU2x(i2c_t3& i2cBus);
    #else
    HTU2x(TwoWire& i2cBus);
    #endif
    bool readTempRaw(uint16_t &value, const bool holdMaster=false) override;
    bool readHumidityRaw(uint16_t &value, const bool holdMaster=false) override;
};

class Si702x: public SHT2x {
  public:
    #if defined(__arm__) && defined(TEENSYDUINO) && (defined(__MK66FX1M0__) || defined(__MK64FX512__) || defined(__MK20DX256__) || defined(__MK20DX128__) || defined(__MKL26Z64__))
    Si702x(i2c_t3& i2cBus);
    #else
    Si702x(TwoWire& i2cBus);
    #endif
    uint8_t readFirmwareVersion(void);
    float fetchTemp(void);
    uint16_t fetchTempRaw(void);
    uint8_t getHeaterPower(void);
    void setHeaterPower(uint8_t value);
  private:
    static const uint16_t CMD_READ_FIRMWARE = 0x84B8;
    static const uint8_t CMD_FETCH_TEMPERATURE = 0xE0;
    static const uint8_t CMD_READ_HEATER_REGISTER = 0x11;
    static const uint8_t CMD_WRITE_HEATER_REGISTER = 0x51;

    bool fetchTempRaw(uint16_t &value);
    bool readTempRaw(uint16_t &value, const bool holdMaster=false) override;
    bool readHumidityRaw(uint16_t &value, const bool holdMaster=false) override;
    bool readHeaterControlRegister(uint8_t& userRegister);
    bool writeHeaterControlRegister(const uint8_t userRegister);

};
#endif
