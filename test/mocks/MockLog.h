/*
 * Shared, ordered log of everything the code under test did to the outside
 * world: I2C write transactions, I2C read requests and calls to delay().
 * Tests assert on this log to check the exact bytes on the wire and the
 * order/timing of the bus traffic.
 */
#ifndef MOCK_LOG_H
#define MOCK_LOG_H

#include <stddef.h>
#include <stdint.h>
#include <vector>

class TwoWire;

namespace mock {

struct Event {
  enum Kind { WRITE, READ, DELAY };

  Kind kind;
  const TwoWire* bus;         // WRITE/READ: the bus the transaction ran on
  uint8_t address;            // WRITE/READ: 7 bit I2C address
  std::vector<uint8_t> data;  // WRITE: bytes sent, READ: bytes returned by the device
  size_t requested;           // READ: number of bytes the master asked for
  uint8_t status;             // WRITE: value returned by endTransmission()
  unsigned long ms;           // DELAY: duration
};

std::vector<Event>& events();
void reset();

}  // namespace mock

#endif
