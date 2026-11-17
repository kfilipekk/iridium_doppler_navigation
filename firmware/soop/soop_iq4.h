//soop_iq4.h - raw I/Q at 4 bits a component, for recording a whole pass on the card
#ifndef SOOP_IQ4_H
#define SOOP_IQ4_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

//A record is a float32 step (capture counts) then n bytes, one a sample: I in the high nibble,
//Q in the low, two's complement -8..7, standing for (code + 0.5) * step. The capture is
//noise-led (a burst sits below the noise across +-250 kHz), so 16 levels at 0.3352 sigma - the
//best uniform step for a Gaussian - lose ~0.1 dB, at a quarter of the int16 size.
#define SOOP_IQ4_STEP_SIGMA 0.3352f

size_t soop_iq4_pack(const int16_t *iq, size_t n, uint8_t *out);     //returns bytes written
void   soop_iq4_unpack(const uint8_t *rec, size_t n, int16_t *iq);

#ifdef __cplusplus
}
#endif
#endif //SOOP_IQ4_H
