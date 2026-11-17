#!/usr/bin/env python3
"""When to test the Iridium receiver: windows ranked by geometry, not by elevation.

Usage:
  soop_plan.py LAT LON [--tle FILE] [--date YYYY-MM-DD] [--from HH:MM] [--hours H]
               [--min-minutes M]

Doppler positioning wants several satellites at once, part way up the sky. A satellite
straight overhead barely constrains the cross-track position, and alone it is the worst
case: at Castle Mound (2026-11-16) a lone pass at 62-80 deg sat 1.5 km off. So each minute
is scored by the satellites between MID_LO and MID_HI degrees, and a minute whose only
satellite above 10 deg is above LONE_HI is flagged. Times are UTC.
"""
import argparse
import datetime
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import soop_live

MID_LO, MID_HI = 20.0, 55.0     #deg: the satellites that pin the position down
LONE_HI = 70.0                  #deg: alone above this, the cross-track goes soft


def minutes(sky, ref, start_s, hours):
    import numpy as np
    t = np.arange(start_s, start_s + hours * 3600.0, 60.0)
    _, el, _ = sky.look(ref, t)
    out = []
    for k in range(t.size):
        e = el[:, k]
        up = e[e >= 10.0]
        mid = int(((e >= MID_LO) & (e <= MID_HI)).sum())
        lone = up.size == 1 and up[0] > LONE_HI
        out.append((float(t[k]), mid, int(up.size), bool(lone)))
    return out


def windows(mins, need=2, min_len=10):
    """Runs of minutes with at least `need` mid-elevation satellites and no lone overhead."""
    runs, cur = [], []
    for m in mins:
        if m[1] >= need and not m[3]:
            cur.append(m)
        else:
            if len(cur) >= min_len:
                runs.append(cur)
            cur = []
    if len(cur) >= min_len:
        runs.append(cur)
    return runs


def hm(s):
    s %= 86400
    return f"{int(s // 3600):02d}:{int(s % 3600 // 60):02d}"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("lat", type=float)
    ap.add_argument("lon", type=float)
    ap.add_argument("--tle", default=soop_live.TLE_DEFAULT)
    ap.add_argument("--hours", type=float, default=12.0)
    ap.add_argument("--date", help="UTC day, YYYY-MM-DD (default today)")
    ap.add_argument("--from", dest="start", help="HH:MM UTC (default now)")
    ap.add_argument("--min-minutes", type=int, default=10)
    a = ap.parse_args()
    sky = soop_live.Sky(a.tle)
    now = datetime.datetime.now(datetime.timezone.utc)
    day = datetime.date.fromisoformat(a.date) if a.date else now.date()
    if a.start:
        h, m = map(int, a.start.split(":"))
        start = h * 3600 + m * 60
    else:
        start = now.hour * 3600 + now.minute * 60
    #Sky.look counts seconds from today's UTC midnight
    start += (day - now.date()).days * 86400
    mins = minutes(sky, (a.lat, a.lon), start, a.hours)
    print(f"{a.lat:.4f} {a.lon:.4f}, {day}, {a.hours:g} h from {hm(start)} UTC")
    for need in (3, 2):
        runs = windows(mins, need, a.min_minutes)
        print(f"\n{need}+ satellites at {MID_LO:.0f}-{MID_HI:.0f} deg together, "
              f"{a.min_minutes}+ min, no lone overhead pass:")
        if not runs:
            print("  none")
        for r in sorted(runs, key=lambda r: -sum(m[1] for m in r))[:8]:
            mean = sum(m[1] for m in r) / len(r)
            print(f"  {hm(r[0][0])}-{hm(r[-1][0] + 60)}  {len(r):3d} min  "
                  f"mean {mean:.1f} mid-elevation, up to {max(m[2] for m in r)} above 10 deg")
    lone = [m for m in mins if m[3]]
    if lone:
        print(f"\navoid: {len(lone)} minutes with one satellite alone above {LONE_HI:.0f} deg, e.g. "
              + ", ".join(hm(m[0]) for m in lone[::max(1, len(lone) // 6)][:6]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
