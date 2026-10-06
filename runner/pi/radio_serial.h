/* A radio link's end on a serial port, on Linux or a Mac: see radio_serial.c. */
#ifndef RADIO_SERIAL_H
#define RADIO_SERIAL_H
#include "radio_io.h"
/* The port opened raw at baud (custom speeds too); 0 if it can't be (said on stderr). */
radio_io *radio_serial_open(const char *dev, int baud, const char *name);
void radio_serial_close(radio_io *R);
#endif
