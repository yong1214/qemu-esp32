#ifndef RTC_COMMON_H
#define RTC_COMMON_H

#include "qemu/osdep.h"

typedef struct RTCDateTimeBin {
    uint8_t seconds;  /* 0-59 */
    uint8_t minutes;  /* 0-59 */
    uint8_t hours;    /* 0-23 */
    uint8_t day;      /* 1-31 */
    uint8_t month;    /* 1-12 */
    uint8_t year;     /* 0-99 */
    uint8_t dow;      /* 1-7 (optional) */
} RTCDateTimeBin;

static inline uint8_t rtc_bin2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
static inline uint8_t rtc_bcd2bin(uint8_t v) { return (uint8_t)(((v >> 4) * 10) + (v & 0x0F)); }

#endif /* RTC_COMMON_H */


