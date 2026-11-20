#!/usr/bin/env python3
"""From an element-set history, the set for each satellite nearest a moment, as a plain TLE file.

Usage:
  soop_tle_pick.py HISTORY.3le "2026-11-18 06:50" OUT.tle [--before] [--report]

HISTORY is a Space-Track gp_history download in 3le (name line, then lines 1 and 2, repeated,
any order). For each NORAD number it keeps the set whose epoch is nearest the moment (UTC), or
with --before the newest at or before it - what a receiver could have had at the time, against
the nearest, which only a post-processing can. --report prints each satellite's age. The output
is what soop_truth.py, soop_ins.py, soop_nav_replay.py and the board read.
"""
import argparse
import datetime
import sys


def parse(path):
    from sgp4.api import Satrec
    lines = [l.rstrip("\n\r") for l in open(path)]
    out = {}
    for i in range(len(lines) - 1):
        if lines[i].startswith("1 ") and lines[i + 1].startswith("2 "):
            name = lines[i - 1].strip() if i > 0 and not lines[i - 1].startswith("2 ") else "?"
            if name.startswith("0 "):
                name = name[2:]
            s = Satrec.twoline2rv(lines[i], lines[i + 1])
            ep = (s.jdsatepoch + s.jdsatepochF - 2440587.5) * 86400.0
            out.setdefault(s.satnum, []).append((ep, name, lines[i], lines[i + 1]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("history")
    ap.add_argument("when", help="UTC, 'YYYY-MM-DD HH:MM'")
    ap.add_argument("out")
    ap.add_argument("--before", action="store_true")
    ap.add_argument("--report", action="store_true")
    a = ap.parse_args()
    t = datetime.datetime.strptime(a.when, "%Y-%m-%d %H:%M").replace(
        tzinfo=datetime.timezone.utc).timestamp()
    hist = parse(a.history)
    ages = []
    with open(a.out, "w") as f:
        f.write(f"# nearest{' earlier' if a.before else ''} element set to {a.when} UTC, from {a.history}\n")
        for num in sorted(hist):
            sets = hist[num]
            if a.before:
                sets = [s for s in sets if s[0] <= t]
                if not sets:
                    continue
            ep, name, l1, l2 = min(sets, key=lambda s: abs(s[0] - t))
            f.write(f"{name}\n{l1}\n{l2}\n")
            ages.append((num, (t - ep) / 3600.0, len(hist[num])))
    if a.report:
        for num, h, n in ages:
            print(f"{num}: {h:+6.1f} h ({n} sets in the history)")
    hs = sorted(abs(x[1]) for x in ages)
    print(f"{len(ages)} satellites -> {a.out}; age median {hs[len(hs) // 2]:.1f} h, max {hs[-1]:.1f} h")
    return 0


if __name__ == "__main__":
    sys.exit(main())
