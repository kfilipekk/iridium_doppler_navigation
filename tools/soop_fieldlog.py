#!/usr/bin/env python3
"""Reduce a field session to one JSON file: the Iridium fix against the surveyed spot, and the
receiver's own state from the board's dataflash logs.

Usage:
  soop_fieldlog.py OUT.json --session SESSION.csv [--log BIN ...] [--ref LAT LON]

SESSION.csv is what soop_session.py wrote (both receivers once a second over MAVLink, with
the GPS-denial marks). Each BIN adds per-second receiver status (SOS) and every burst (SOB).
Needs pymavlink for the BIN files.
"""
import argparse
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import soop_watch as w


def session(path, ref_arg):
    marks, g1, g2 = w.read(path)
    end_t = w.secs(g2[-1][0])
    win = w.windows(marks, end_t)
    first_den = next((s for s, e, d in win if d), end_t)
    good = [r for r in g1 if r[1] >= 3 and w.secs(r[0]) < first_den]
    ref = ref_arg or (statistics.fmean(r[2] for r in good), statistics.fmean(r[3] for r in good))
    denied = lambda t: any(d and s <= t <= e for s, e, d in win)
    series = []
    for r in g2:
        t = w.secs(r[0])
        if r[2] == 0.0:
            continue
        series.append(dict(t=round(t, 1), err=round(w.err_m(r[2], r[3], ref), 2), hacc=r[4],
                           sats=r[5], fix=r[1], denied=denied(t)))
    windows = []
    for s, e, d in win:
        rows = [x for x in series if s <= x["t"] <= e]
        errs = [x["err"] for x in rows]
        windows.append(dict(start=s, end=e, denied=d, n=len(errs),
                            median=w.pct(errs, .5) if errs else None,
                            p95=w.pct(errs, .95) if errs else None,
                            max=max(errs) if errs else None,
                            inside2=(sum(1 for x in rows if x["err"] <= 2 * x["hacc"]) / len(rows))
                            if rows else None))
    return dict(ref=ref, series=series, windows=windows)


def board_log(path):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(path)
    sos, sob, msgs, t_utc = [], [], [], None
    while True:
        x = m.recv_match(type=["SOS", "SOB", "MSG", "GPS"])
        if x is None:
            break
        k = x.get_type()
        if k == "GPS" and t_utc is None and x.I == 0 and x.GWk > 0:
            #log time -> UTC seconds of day, from the first u-blox record that knows the week
            t_utc = ((x.GMS - 18000) / 1000.0) % 86400 - x.TimeUS / 1e6
        elif k == "SOS":
            sos.append(dict(t=x.TimeUS / 1e6, cpu=x.CPU, bps=x.Bps, ovr=x.Ovr, nt=x.NT,
                            clk=x.Clk, fused=x.Fused, unm=x.Unm, nse=x.Nse))
        elif k == "SOB":
            sob.append(dict(t=x.TimeUS / 1e6, tb=x.T, f=x.F, cn0=x.CN0, res=x.Res))
        elif k == "MSG" and "SoOP" in x.Message:
            msgs.append(dict(t=x.TimeUS / 1e6, text=x.Message))
    off = t_utc or 0.0
    for lst in (sos, sob, msgs):
        for r in lst:
            r["t"] = round(r["t"] + off, 2)
    return dict(name=os.path.basename(path), utc_known=t_utc is not None, sos=sos, sob=sob,
                msgs=msgs)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out")
    ap.add_argument("--session", required=True)
    ap.add_argument("--log", nargs="*", default=[])
    ap.add_argument("--ref", type=float, nargs=2, metavar=("LAT", "LON"))
    a = ap.parse_args()
    data = dict(session=session(a.session, a.ref), logs=[board_log(p) for p in a.log])
    json.dump(data, open(a.out, "w"))
    s = data["session"]
    print(f"{len(s['series'])} Iridium fixes, {len(s['windows'])} windows, "
          f"{sum(len(l['sob']) for l in data['logs'])} bursts -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
