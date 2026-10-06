/* The command module's end of the pilot's radio link on the ESP32 (radio_io.h). */
#ifndef RADIO_MODULE_H
#define RADIO_MODULE_H
#include "radio_io.h"
/* An ExpressLRS transmitter module on the radio UART: its input on tx, its output on rx (tx = rx: a module bay's one
 * wire, inverted, half duplex), started. */
radio_io *radio_module_start(int tx, int rx, int baud);
#endif
