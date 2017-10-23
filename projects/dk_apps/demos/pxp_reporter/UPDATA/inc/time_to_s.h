#ifndef CSYSTIME_H
#define CSYSTIME_H

#include "stdbool.h"
#include "stdint.h"

typedef struct SysTime{
      uint16_t  year;
      uint8_t   month;
      uint8_t   day;
      uint8_t   hour;
      uint8_t   min;
      uint8_t   sec;
      uint16_t  milliseconds;
} SysTime_t;

uint32_t DateSwSec(SysTime_t date);

#endif
