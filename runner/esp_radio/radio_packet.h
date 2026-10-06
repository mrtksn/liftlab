/* A packet link's radio_io, whatever carries the packets (ESP-NOW, UDP): plink at one end, a transport below it.
 * (Inside esp_radio: see esp_radio.h.) */
#ifndef RADIO_PACKET_H
#define RADIO_PACKET_H
#include "esp_radio.h"

typedef struct pk_link pk_link;
struct pk_link {
  radio_io io;                 /* (first: a radio_io * is a pk_link *) */
  plink L;
  /* the transport: a packet that came (its length; 0 none, waiting at most wait_ms for one), rssi its signal [dBm]
   * (0: unknown); send one (0 sent). taken: plink took the last packet recv gave (UDP answers its sender). */
  int (*recv)(pk_link *K, uint8_t *p, int cap, int *rssi, int wait_ms);
  int (*send)(pk_link *K, const uint8_t *p, int n);
  void (*taken)(pk_link *K);
  void *t;                     /* the transport's own state */
  uint32_t send_fail;
  esp_radio_say say;
};
/* plink set up for role with the binding phrase's key and a random session; io's read and write set. */
void pk_init(pk_link *K, const char *name, int role, const char *bind, esp_radio_say say);
double pk_now(void);
/* (say, formatted) */
void pk_say(pk_link *K, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
#endif
