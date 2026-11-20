#!/usr/bin/env python3
"""Unpack the board's raw I/Q recordings (APM/SOOP/iqNNNN.*) to interleaved int16.

Usage:
  soop_iq.py FILE [OUT.bin] [--swap] [--no-fill]

Formats, by extension:
  .iq4   a float32 step then 4096 bytes a record, one byte a sample: I in the high nibble, Q in
         the low, two's complement -8..7, standing for (code + 0.5) * step (soop_iq4.c). Blocks
         the recorder dropped are simply missing: the board log's SOD records (and its SOQ
         counter, see soop_reprocess.py) say where.
  .bin   the same as int16 I, Q, 4096 samples a record.
  .iq4b  .iq4 with each record's capture block number (uint32) in front, so the file carries
  .i16b  its own timeline: missing blocks are filled with zeros in the output (unless --no-fill)
         and the first sample's capture index is printed (SOI in the log gives its board time).
The output is what dsp_host and soop.cfg replay read: int16 I, Q at 500 kSPS, centred on the
LO (Ring Alert + 62.5 kHz unless record_lo_hz moved it). --swap exchanges I and Q: Rev E's
spectrum is mirrored about the LO as recorded up to 2026-11-20 (firmware since then turns it
over itself and logs SOW when it does).
"""
import os
import sys

import numpy as np

BLOCK = 4096


def unpack4(b):
    hi = (b >> 4).astype(np.int8)
    lo = (b & 0xF).astype(np.int8)
    hi[hi >= 8] -= 16
    lo[lo >= 8] -= 16
    return hi, lo


def read(path):
    """(block numbers or None, int16 array (n_blocks, BLOCK, 2))."""
    ext = os.path.splitext(path)[1]
    raw = np.fromfile(path, dtype=np.uint8)
    numbered = ext in (".iq4b", ".i16b")
    four = ext in (".iq4", ".iq4b")
    head = 4 if numbered else 0
    rec = head + (4 + BLOCK if four else 4 * BLOCK)
    n = len(raw) // rec
    raw = raw[:n * rec].reshape(n, rec)
    blocks = raw[:, :4].copy().view("<u4")[:, 0].astype(np.int64) if numbered else None
    body = raw[:, head:]
    out = np.empty((n, BLOCK, 2), dtype=np.int16)
    if four:
        step = body[:, :4].copy().view("<f4")[:, 0]
        hi, lo = unpack4(body[:, 4:])
        out[..., 0] = np.clip(np.rint((hi + 0.5) * step[:, None]), -32768, 32767)
        out[..., 1] = np.clip(np.rint((lo + 0.5) * step[:, None]), -32768, 32767)
    else:
        out[:] = body.copy().view("<i2").reshape(n, BLOCK, 2)
    return blocks, out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        print(__doc__)
        return 2
    src = args[0]
    dst = args[1] if len(args) > 1 else os.path.splitext(src)[0] + ".bin.raw"
    blocks, iq = read(src)
    if "--swap" in sys.argv:
        iq = iq[..., ::-1]
    if blocks is not None and "--no-fill" not in sys.argv:
        b0 = int(blocks[0])
        full = np.zeros((int(blocks[-1]) - b0 + 1, BLOCK, 2), np.int16)
        full[blocks - b0] = iq
        print(f"first capture sample {b0 * BLOCK}; {len(full) - len(iq)} missing blocks zero-filled")
        iq = full
    iq.tofile(dst)
    print(f"{iq.shape[0] * BLOCK} samples ({iq.shape[0] * BLOCK / 500e3:.1f} s) -> {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
