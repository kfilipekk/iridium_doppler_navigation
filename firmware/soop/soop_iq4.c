//soop_iq4.c - see soop_iq4.h
#include "soop_iq4.h"

#include <math.h>
#include <string.h>

static int code_of(float x, float inv_step)
{
    //mid-rise: (code + 0.5) * step, code in -8..7
    int c = (int)floorf(x * inv_step);
    return c < -8 ? -8 : (c > 7 ? 7 : c);
}

size_t soop_iq4_pack(const int16_t *iq, size_t n, uint8_t *out)
{
    //the step from this record's own rms, about zero so a dc offset cannot clip: a burst barely
    //moves it, the noise floor sets it
    double s2 = 0.0;
    for (size_t k = 0; k < 2 * n; k++)
        s2 += (double)iq[k] * iq[k];
    const float sigma = n ? (float)sqrt(s2 / (2.0 * n)) : 1.0f;
    float step = SOOP_IQ4_STEP_SIGMA * (sigma > 1.0f ? sigma : 1.0f);
    memcpy(out, &step, sizeof step);
    const float inv = 1.0f / step;
    for (size_t k = 0; k < n; k++) {
        const int ci = code_of(iq[2 * k], inv), cq = code_of(iq[2 * k + 1], inv);
        out[sizeof step + k] = (uint8_t)(((ci & 0xF) << 4) | (cq & 0xF));
    }
    return sizeof step + n;
}

void soop_iq4_unpack(const uint8_t *rec, size_t n, int16_t *iq)
{
    float step;
    memcpy(&step, rec, sizeof step);
    for (size_t k = 0; k < n; k++) {
        const uint8_t b = rec[sizeof step + k];
        const int ci = (b >> 4) >= 8 ? (int)(b >> 4) - 16 : (int)(b >> 4);
        const int cq = (b & 0xF) >= 8 ? (int)(b & 0xF) - 16 : (int)(b & 0xF);
        iq[2 * k] = (int16_t)lrintf((ci + 0.5f) * step);
        iq[2 * k + 1] = (int16_t)lrintf((cq + 0.5f) * step);
    }
}
