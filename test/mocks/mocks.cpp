#include "Arduino.h"
#include "MockLog.h"
#include "Wire.h"

namespace mock {

std::vector<Event>& events() {
  static std::vector<Event> log;
  return log;
}

void reset() {
  events().clear();
}

}  // namespace mock

void delay(unsigned long ms) {
  mock::Event event = mock::Event();
  event.kind = mock::Event::DELAY;
  event.ms = ms;
  mock::events().push_back(event);
}

TwoWire Wire;

TwoWire::TwoWire() : _txAddress(0), _writeLimit(32) {}

void TwoWire::begin() {}

void TwoWire::beginTransmission(uint8_t address) {
  _txAddress = address;
  _txBuffer.clear();
}

size_t TwoWire::write(uint8_t data) {
  return write(&data, 1);
}

size_t TwoWire::write(const uint8_t* data, size_t len) {
  size_t written = 0;
  while (written < len && _txBuffer.size() < _writeLimit) {
    _txBuffer.push_back(data[written++]);
  }
  return written;
}

uint8_t TwoWire::endTransmission(void) {
  uint8_t status = 0;
  if (!_writeStatus.empty()) {
    status = _writeStatus.front();
    _writeStatus.pop_front();
  }

  mock::Event event = mock::Event();
  event.kind = mock::Event::WRITE;
  event.bus = this;
  event.address = _txAddress;
  event.data = _txBuffer;
  event.status = status;
  mock::events().push_back(event);

  _txBuffer.clear();
  return status;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  std::vector<uint8_t> response;
  if (!_responses.empty()) {
    response = _responses.front();
    _responses.pop_front();
  }
  if (response.size() > quantity) {
    response.resize(quantity);
  }
  _rxBuffer.assign(response.begin(), response.end());

  mock::Event event = mock::Event();
  event.kind = mock::Event::READ;
  event.bus = this;
  event.address = address;
  event.data = response;
  event.requested = quantity;
  mock::events().push_back(event);

  return static_cast<uint8_t>(response.size());
}

uint8_t TwoWire::requestFrom(int address, int quantity) {
  return requestFrom(static_cast<uint8_t>(address), static_cast<uint8_t>(quantity));
}

int TwoWire::available(void) {
  return static_cast<int>(_rxBuffer.size());
}

int TwoWire::read(void) {
  if (_rxBuffer.empty()) {
    return -1;
  }
  const uint8_t value = _rxBuffer.front();
  _rxBuffer.pop_front();
  return value;
}

size_t TwoWire::readBytes(uint8_t* buffer, size_t len) {
  size_t count = 0;
  while (count < len && !_rxBuffer.empty()) {
    buffer[count++] = _rxBuffer.front();
    _rxBuffer.pop_front();
  }
  return count;
}

void TwoWire::queueRead(const std::vector<uint8_t>& response) {
  _responses.push_back(response);
}

void TwoWire::queueWriteStatus(uint8_t status) {
  _writeStatus.push_back(status);
}

void TwoWire::setWriteLimit(size_t limit) {
  _writeLimit = limit;
}
