/* BMP180/BMP085 compensation, Bosch datasheet revision 2.5, figure 4.
 * 64-bit intermediates avoid overflow; invalid calibration/results are rejected. */
#ifndef LIFTLAB_BMP180_H
#define LIFTLAB_BMP180_H
#include <stdint.h>
typedef struct { int16_t ac1,ac2,ac3,b1,b2,mb,mc,md; uint16_t ac4,ac5,ac6; } lb_bmp180_cal;
static inline int lb_bmp180_pressure(const lb_bmp180_cal *c,int32_t ut,int32_t up,int oss,int32_t *pa) {
  if(oss<0 || oss>3 || !c->ac4 || !c->ac5 || !c->ac6 || ut<0 || ut>65535 || up<0 || up>524287) return -1;
  int64_t x1=((int64_t)(ut-c->ac6)*c->ac5)>>15, den=x1+c->md;
  if(!den) return -1;
  int64_t x2=((int64_t)c->mc*2048)/den, b5=x1+x2, b6=b5-4000;
  x1=((int64_t)c->b2*((b6*b6)>>12))>>11; x2=((int64_t)c->ac2*b6)>>11;
  int64_t b3=((((int64_t)c->ac1*4+x1+x2)*(1<<oss))+2)/4;
  x1=((int64_t)c->ac3*b6)>>13; x2=((int64_t)c->b1*((b6*b6)>>12))>>16;
  int64_t x3=(x1+x2+2)>>2, b4=((int64_t)c->ac4*(x3+32768))>>15;
  int64_t b7=((int64_t)up-b3)*(50000>>oss);
  if(b4<=0 || b7<0) return -1;
  int64_t p=b7<0x80000000LL ? b7*2/b4 : (b7/b4)*2;
  x1=(p>>8)*(p>>8); x1=(x1*3038)>>16; x2=(-7357*p)>>16;
  p+=(x1+x2+3791)>>4;
  if(p<30000 || p>110000) return -1;
  *pa=(int32_t)p; return 0;
}
#endif
