/* The drone's end of the pilot's radio link on the ESP32 (radio_io.h): which link the settings ask for. */
#ifndef RADIO_ELRS_H
#define RADIO_ELRS_H
#include "radio_io.h"
#include "hw.h"
/* An ExpressLRS receiver on the radio UART (crsf=rx,tx), started; 0 if none is wired. */
radio_io *radio_elrs_start(const hw_config *c);
#endif
