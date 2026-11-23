#!/usr/bin/env python3
"""After a field session: did the receiver work live? One page of pass/fail from the board's log.

Usage:
  soop_check_session.py LOG.npz [--iq DIR]

LOG.npz is soop_extract.py's. Checks, each a line:
  orientation  SOW: the spectrum right way up and locked (decodes among strong bursts)
  decodes      SOR: Ring Alerts decoded on board, per minute
  bursts       SOB: how many the navigation fused, unmatched, ambiguous
  clock        SOS: the clock estimate stayed physical (|clock| < 6 kHz, drift < 2 Hz/s)
  fixes        SOF against the u-blox: horizontal error while the fix was offered, and how
               often it sat inside 2 x its stated accuracy
  recording    SOQ: blocks dropped against written; with --iq, the files' own block numbers
               agree with SOI and leave no gap the counters do not explain
"""
import argparse
import glob
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def line(ok, name, text):
    ok = None if ok is None else bool(ok)
    print("  %-4s %-12s %s" % ("ok" if ok else ("--" if ok is None else "FAIL"), name, text))
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("npz")
    ap.add_argument("--iq")
    a = ap.parse_args()
    D = np.load(a.npz)
    get = lambda k: D[k] if k in D and len(D[k]) else np.zeros((0, 1))
    sow, sor, sob, sos, sof, gps, soq, soi = (get(k) for k in
                                              ("sow", "sor", "sob", "sos", "sof", "gps", "soq", "soi"))
    span_min = (sos[-1, 0] - sos[0, 0]) / 60.0 if len(sos) > 1 else 0.0
    print(f"{os.path.basename(a.npz)}: {span_min:.0f} min of receiver status")
    res = []
    if len(sow):
        locked = sow[:, 4] > 0
        flips = int(np.sum(np.diff(sow[:, 1]) != 0))
        frac = sow[:, 3].sum() / max(sow[:, 2].sum(), 1)
        res.append(line(bool(locked[-1]) and frac > 0.2, "orientation",
                        f"swap {int(sow[-1, 1])}, locked {locked.mean() * 100:.0f}% of checks, {flips} flips, "
                        f"{frac * 100:.0f}% of strong bursts decoded"))
    else:
        res.append(line(None, "orientation", "no SOW records (firmware older than the I/Q orientation check?)"))
    rate = len(sor) / max(span_min, 1e-9)
    res.append(line(rate > 5, "decodes", f"{len(sor)} Ring Alerts decoded, {rate:.1f}/min"))
    if len(sos):
        fused, amb, unm = sos[-1, 5] - sos[0, 5], sos[-1, 6] - sos[0, 6], sos[-1, 7] - sos[0, 7]
        tot = max(fused + amb + unm, 1)
        res.append(line(fused / tot > 0.5, "bursts", f"{len(sob)} bursts; fused {fused:.0f} ({fused / tot * 100:.0f}%), "
                        f"unmatched {unm:.0f}, ambiguous {amb:.0f}"))
        clk = sos[:, 4]
        nz = clk != 0
        drift = np.abs(np.diff(clk[nz])) / np.maximum(np.diff(sos[nz, 0]), 1e-3) if nz.sum() > 2 else np.zeros(1)
        res.append(line(np.abs(clk).max() < 6000 and np.percentile(drift, 99) < 2.0, "clock",
                        f"{clk[nz].min() if nz.any() else 0:+.0f} .. {clk[nz].max() if nz.any() else 0:+.0f} Hz, "
                        f"99% of steps under {np.percentile(drift, 99):.2f} Hz/s"))
    if len(sof) and len(gps):
        import soop_truth as T
        v = sof[:, 6] > 0
        if v.sum() > 10:
            t = sof[v, 0]
            rr, vr, lat, lon, alt, spd = T.truth_at(gps, -0.02, t)
            ok = np.isfinite(lat)
            dn = np.radians(sof[v, 1][ok] - lat[ok]) * 6371000.0
            de = np.radians(sof[v, 2][ok] - lon[ok]) * 6371000.0 * np.cos(np.radians(lat[ok]))
            e = np.hypot(dn, de)
            inside = np.mean(e <= 2 * sof[v, 4][ok]) * 100
            res.append(line(np.median(e) < 1000, "fixes",
                            f"{v.sum()} offered: median {np.median(e):.0f} m, p95 {np.percentile(e, 95):.0f} m, "
                            f"{inside:.0f}% inside 2 x hacc (u-blox as truth; meaningful when gps_pos 0)"))
        else:
            res.append(line(False, "fixes", f"only {v.sum()} fixes offered"))
    if len(soq):
        d, w = soq[-1, 2] - soq[0, 2], soq[-1, 3] - soq[0, 3]
        res.append(line(d / max(d + w, 1) < 0.05, "recording", f"{w:.0f} blocks written, {d:.0f} dropped "
                        f"({d / max(d + w, 1) * 100:.1f}%)"))
    if a.iq and len(soi):
        import soop_iq
        bad = 0
        for row in soi:
            fs = glob.glob(os.path.join(a.iq, "iq%04d.iq4b" % int(row[1]))) + \
                 glob.glob(os.path.join(a.iq, "iq%04d.i16b" % int(row[1])))
            if not fs:
                continue
            blocks, _ = soop_iq.read(fs[0])
            first_ok = int(blocks[0]) * 4096 >= int(row[2])
            back = int(np.sum(np.diff(blocks) <= 0))
            bad += (not first_ok) + back
        res.append(line(bad == 0, "iq files", f"{len(soi)} files, block numbers {'consistent' if bad == 0 else f'{bad} problems'}"))
    n_fail = sum(1 for r in res if r is False)
    print(f"{'ALL OK' if n_fail == 0 else f'{n_fail} FAILED'}")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
