/*
 * Minimal host-side stand-in for the Arduino core header. It only provides
 * what the SHT2x library uses and records calls to delay() in the shared
 * event log (see MockLog.h) instead of sleeping.
 */
#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

void delay(unsigned long ms);

#endif
