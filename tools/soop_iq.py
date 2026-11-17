#!/usr/bin/env python3
"""Unpack the board's 4-bit I/Q recordings (APM/SOOP/iqNNNN.iq4) to interleaved int16.

Usage:
  soop_iq.py FILE.iq4 [OUT.bin]

Each record is a float32 step then 4096 bytes, one a sample: I in the high nibble, Q in the
low, two's complement -8..7, standing for (code + 0.5) * step (firmware/soop/soop_iq4.c). The
output is what dsp_host and soop.cfg replay read: int16 I, Q at 500 kSPS. The capture centre
is the LO (Ring Alert + 62.5 kHz unless record_lo_hz moved it); SOI in the board log gives
each file's first sample and its board time.
"""
import os
import sys

import numpy as np

BLOCK = 4096


def unpack(path):
    raw = np.fromfile(path, dtype=np.uint8)
    rec = 4 + BLOCK
    n = len(raw) // rec
    raw = raw[: n * rec].reshape(n, rec)
    step = raw[:, :4].copy().view("<f4")[:, 0]
    b = raw[:, 4:]
    hi = (b >> 4).astype(np.int8)
    lo = (b & 0xF).astype(np.int8)
    hi[hi >= 8] -= 16
    lo[lo >= 8] -= 16
    i = np.rint((hi + 0.5) * step[:, None])
    q = np.rint((lo + 0.5) * step[:, None])
    out = np.empty((n * BLOCK, 2), dtype=np.int16)
    out[:, 0] = np.clip(i, -32768, 32767).ravel()
    out[:, 1] = np.clip(q, -32768, 32767).ravel()
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + ".bin"
    iq = unpack(src)
    iq.tofile(dst)
    print(f"{len(iq)} samples ({len(iq) / 500e3:.1f} s) -> {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
