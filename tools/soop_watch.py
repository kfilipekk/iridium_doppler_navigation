#!/usr/bin/env python3
"""Watch a soop_session.py recording as it grows: the Iridium fix against the surveyed spot.

Usage:
  soop_watch.py SESSION.csv [--ref LAT LON] [--once] [--every S]

The reference is where the antenna is. Without --ref it is the mean u-blox position over the
anchored seconds before the first GPS denial. Each denial window is scored on its own: the
Iridium-only horizontal error against the reference, and how often it sat inside its own
reported accuracy (the flight EKF needs that to hold).
"""
import argparse
import csv
import math
import os
import sys
import time

BARS = " ▁▂▃▄▅▆▇█"


def read(path):
    marks, g1, g2 = [], [], []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if len(row) < 3:
                continue
            t = row[0]
            if row[1] == "MARK":
                marks.append((t, row[2]))
            elif row[1] in ("GPS1", "GPS2") and len(row) >= 7:
                rec = (t, int(row[2]), float(row[3]), float(row[4]), float(row[5]), int(row[6]))
                (g1 if row[1] == "GPS1" else g2).append(rec)
    return marks, g1, g2


def secs(hms):
    h, m, s = hms.split(":")
    return int(h) * 3600 + int(m) * 60 + float(s)


def err_m(lat, lon, ref):
    dn = math.radians(lat - ref[0]) * 6371000.0
    de = math.radians(lon - ref[1]) * 6371000.0 * math.cos(math.radians(ref[0]))
    return math.hypot(dn, de)


def windows(marks, end_t):
    """[(start, end, denied)] from the GPS DISABLED / ENABLED marks."""
    out, state, start = [], None, None
    for t, text in marks:
        if text in ("GPS DISABLED", "GPS ENABLED"):
            denied = text == "GPS DISABLED"
            if state is not None and denied != state:
                out.append((start, secs(t), state))
            if state is None or denied != state:
                state, start = denied, secs(t)
    if state is not None:
        out.append((start, end_t, state))
    return out


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else float("nan")


def spark(v, width=48):
    if not v:
        return ""
    step = max(1, len(v) // width)
    v = [max(v[i:i + step]) for i in range(0, len(v), step)][-width:]
    top = max(v) or 1.0
    return "".join(BARS[min(8, int(8 * x / top))] for x in v) + f"  max {top:.0f} m"


def report(path, ref_arg):
    marks, g1, g2 = read(path)
    if not g2:
        return "waiting for data..."
    end_t = secs(g2[-1][0])
    win = windows(marks, end_t)
    first_den = next((s for s, e, d in win if d), end_t)
    if ref_arg:
        ref, how = ref_arg, "given"
    else:
        good = [r for r in g1 if r[1] >= 3 and secs(r[0]) < first_den]
        if not good:
            return "no anchored u-blox fix yet to take the reference from"
        ref = (sum(r[2] for r in good) / len(good), sum(r[3] for r in good) / len(good))
        how = f"mean of {len(good)} u-blox fixes"
    lines = [f"{os.path.basename(path)}   reference {ref[0]:.7f} {ref[1]:.7f} ({how})", ""]

    last1, last2 = (g1[-1] if g1 else None), g2[-1]
    state = "DENIED" if win and win[-1][2] else "GPS on"
    phase_s = end_t - win[-1][0] if win else 0
    lines.append(f"now {last2[0][:8]} UTC   {state} for {phase_s / 60:.1f} min")
    if last1:
        lines.append(f"  u-blox   fix {last1[1]}  {last1[5]:2d} sats  hacc {last1[4]:6.1f} m")
    e = err_m(last2[2], last2[3], ref)
    lines.append(f"  Iridium  fix {last2[1]}  {last2[5]:2d} sats  hacc {last2[4]:6.1f} m   "
                 f"error {e:6.1f} m")
    lines.append("")
    lines.append(f"{'window':16s} {'min':>5s} {'median':>7s} {'p95':>7s} {'max':>7s} {'final':>7s} "
                 f"{'in 2 hacc':>9s}")
    for s, en, denied in win:
        if not denied:
            continue
        rows = [r for r in g2 if s <= secs(r[0]) <= en and r[2] != 0.0]
        errs = [err_m(r[2], r[3], ref) for r in rows]
        inside = sum(1 for r, x in zip(rows, errs) if x <= 2 * r[4]) / max(1, len(errs))
        lab = time.strftime("%H:%M", time.gmtime(s)) + f" +{(en - s) / 60:4.1f}m"
        lines.append(f"{lab:16s} {(en - s) / 60:5.1f} {pct(errs, .5):6.1f}m {pct(errs, .95):6.1f}m "
                     f"{max(errs or [0]):6.1f}m {errs[-1] if errs else 0:6.1f}m {inside:9.0%}")
        lines.append(f"  {spark(errs)}")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("session")
    ap.add_argument("--ref", type=float, nargs=2, metavar=("LAT", "LON"))
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--every", type=float, default=2.0)
    a = ap.parse_args()
    while True:
        text = report(a.session, a.ref)
        if a.once:
            print(text)
            return 0
        sys.stdout.write("\x1b[H\x1b[2J" + text + "\n")
        sys.stdout.flush()
        time.sleep(a.every)


if __name__ == "__main__":
    sys.exit(main())
