/* Versioned wiring restore; the caller supplies new defaults before restoring old fields. */
#ifndef LB_HW_MIGRATION_H
#define LB_HW_MIGRATION_H
#include "hw.h"
#include <stddef.h>
#include <string.h>
static inline int lb_hw_restore(hw_config *c, const void *blob, size_t n) {
  uint32_t version; if(n<sizeof version)return 0;memcpy(&version,blob,sizeof version);
  size_t prefix=0;
  if(version==HW_VERSION && n==sizeof *c)prefix=n;
  else if(version==7 && n==offsetof(hw_config,radio_baud))prefix=n;
  else if(version==6 && n==offsetof(hw_config,radio_kind))prefix=n;
  else if(version==5 && n==offsetof(hw_config,motor_driver))prefix=n;
  else if(version==4 && n==((offsetof(hw_config,imu_driver)+3u)&~3u))prefix=offsetof(hw_config,imu_driver);
  else if(version==3 && n==offsetof(hw_config,crsf_rx))prefix=n;
  else if(version==2 && n==offsetof(hw_config,link_baud))prefix=n;
  if(!prefix)return 0;
  memcpy(c,blob,prefix);c->version=HW_VERSION;if(version==2)c->link_baud=115200;
  return 1;
}
#endif
