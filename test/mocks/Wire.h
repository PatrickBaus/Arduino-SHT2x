/*
 * Minimal host-side fake of the Arduino TwoWire class.
 *
 * - Everything written between beginTransmission() and endTransmission() is
 *   recorded as one WRITE event in the shared log (see MockLog.h).
 * - Every requestFrom() is recorded as one READ event and answered with the
 *   next scripted response (queueRead()). Without a scripted response the
 *   device does not answer (NACK, zero bytes), a response shorter than the
 *   request models a short read.
 * - endTransmission() returns the next scripted status (queueWriteStatus()),
 *   0 (success) by default. Non-zero values model a NACK or bus error.
 * - setWriteLimit() limits how many bytes write() accepts per transaction to
 *   model a full transmit buffer.
 */
#ifndef MOCK_WIRE_H
#define MOCK_WIRE_H

#include <deque>
#include <vector>

#include "Arduino.h"
#include "MockLog.h"

class TwoWire {
  public:
    TwoWire();

    // The subset of the Arduino API used by the library
    void begin();
    void beginTransmission(uint8_t address);
    size_t write(uint8_t data);
    size_t write(const uint8_t* data, size_t len);
    uint8_t endTransmission(void);
    uint8_t requestFrom(uint8_t address, uint8_t quantity);
    uint8_t requestFrom(int address, int quantity);
    int available(void);
    int read(void);
    size_t readBytes(uint8_t* buffer, size_t len);

    // Test scripting interface
    void queueRead(const std::vector<uint8_t>& response);
    void queueWriteStatus(uint8_t status);
    void setWriteLimit(size_t limit);

  private:
    uint8_t _txAddress;
    std::vector<uint8_t> _txBuffer;
    std::deque<uint8_t> _rxBuffer;
    std::deque<std::vector<uint8_t> > _responses;
    std::deque<uint8_t> _writeStatus;
    size_t _writeLimit;
};

extern TwoWire Wire;

#endif
