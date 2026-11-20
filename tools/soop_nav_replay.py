#!/usr/bin/env python3
"""Replay the on-board navigation over a session's bursts, scored against the u-blox, by scenario.

Usage:
  soop_nav_replay.py LOG.npz TLE BURSTS.csv [--mode blind|anchored] [--start-sigma M]
                     [--start-offset N E] [--vel ublox|none] [--vel-noise MS]
                     [--alt truth|none] [--alt-sigma M] [--gps-until H] [--from H] [--to H]
                     [--cfg KEY=VAL ...] [--out FIXES.csv] [--tag NAME]

LOG.npz is soop_extract.py's, BURSTS.csv soop_reprocess.py's (or the board's own: --board-bursts).
Modes:
  blind     as the flight config ran: the u-blox gives time only (U records, every good epoch),
            one start position (its first good fix, --start-sigma, optionally moved by
            --start-offset metres), and its velocity (--vel ublox) or none.
  anchored  its position too (G records) until --gps-until (UTC hours), then as blind.
            --vel-hold stops its velocity there too: GPS-free from then on (give the filter
            room with --cfg vel_sigma=..., the velocity error it may follow).
--alt truth stands in for an air-data/baro altitude: the u-blox height + noise of --alt-sigma
(a cabin barometer reads the cabin, not the aircraft). Scored: horizontal error of every 1 s fix
against the u-blox at the same instant, by flight phase, and how often it sat inside 2 x hacc.
"""
import argparse
import csv
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import soop_truth as T

NAV_HOST = os.path.join(os.path.dirname(HERE), "firmware", "soop", "nav_host")
J2000_UNIX = 946728000.0
LAG = 0.075            #the u-blox's fix arrives this long after its epoch (--lag)


def phase_of(spd, alt):
    if spd < 5:
        return "ground"
    if spd < 60:
        return "taxi/road"
    if alt > 8000:
        return "cruise"
    return "climb/descent"


def build(a, D, bursts, obs_path):
    gps = D["gps"]
    to_utc, _, _ = T.time_map(gps, LAG)
    good = (gps[:, 1] >= 3) & (gps[:, 2] >= 6) & (gps[:, 9] < 5.0) & (gps[:, 11] > 2000)
    g = gps[good]
    tb = g[:, 0] - LAG                                   #epoch board time
    utc = g[:, 11] * 604800.0 + g[:, 12] / 1000.0 + T.GPS_UNIX - T.LEAP
    day0 = math.floor((utc[0] - J2000_UNIX + 43200.0) / 86400.0) * 86400.0 - 43200.0 + J2000_UNIX
    lines = []
    sod = lambda u: u - day0
    lo = (a.from_h or 0) * 3600 + day0 - 86400 * 0
    span = lambda u: (a.from_h is None or (u - day0) % 86400 >= a.from_h * 3600) and \
                     (a.to_h is None or (u - day0) % 86400 <= a.to_h * 3600)
    rng = np.random.default_rng(0)
    r_all = T.geo_ecef(g[:, 3], g[:, 4], g[:, 5] + 45.0)
    started = False
    for k in range(len(g)):
        if not span(utc[k]):
            continue
        t = tb[k]
        anchored = a.mode == "anchored" and (a.gps_until is None or (utc[k] - day0) % 86400 < a.gps_until * 3600)
        if not started:
            r0 = r_all[k].copy()
            if a.start_offset:
                e, n, u = T.enu_basis(np.array([g[k, 3]]), np.array([g[k, 4]]))
                r0 = r0 + a.start_offset[0] * n[0] + a.start_offset[1] * e[0]
            lines.append((t - 1e-3, "P %.6f %.3f %.3f %.3f %.1f" % (t - 1e-3, *r0, a.start_sigma)))
            started = True
        if anchored:
            lines.append((t, "G %.6f %.6f %.3f %.3f %.3f" % (t, sod(utc[k]), *r_all[k])))
        else:
            lines.append((t, "U %.6f %.6f" % (t, sod(utc[k]))))
        if a.vel == "ublox" and (anchored or not a.vel_hold):
            spd, crs, vz = g[k, 6], math.radians(g[k, 7]), g[k, 8]
            nz = rng.normal(0, a.vel_noise, 3) if a.vel_noise > 0 else np.zeros(3)
            lines.append((t + 1e-4, "E %.6f %.4f %.4f %.4f" % (t + 1e-4, spd * math.cos(crs) + nz[0],
                                                             spd * math.sin(crs) + nz[1], vz + nz[2])))
        if a.alt == "truth" and k % 5 == 0:
            lines.append((t + 2e-4, "A %.6f %.3f" % (t + 2e-4, g[k, 5] + 45.0 + rng.normal(0, a.alt_sigma))))
    n_b = 0
    for b in bursts:
        if not span(to_utc(b[0])):
            continue
        lines.append((b[0], "D %.6f %.3f %.3f %.2f %d" % tuple(b[:5])))
        n_b += 1
    lines.sort(key=lambda r: r[0])
    jd = 2451545.0 + (day0 - J2000_UNIX) / 86400.0
    with open(obs_path, "w") as f:
        f.write("H %.1f %.12f\n" % (math.floor(jd - 0.5) + 0.5, jd - (math.floor(jd - 0.5) + 0.5)))
        f.write("\n".join(l for _, l in lines) + "\n")
    return n_b, to_utc


def load_bursts(path, board):
    if board:
        s = np.load(path)["sob"]
        return np.column_stack([s[:, 0], s[:, 1], s[:, 2], s[:, 3], s[:, 5]])
    rows = list(csv.DictReader(open(path)))
    b = np.array([[float(r["t"]), float(r["f_hz"]), float(r["f_sigma_hz"]), float(r["cn0_dbhz"]),
                   int(r["ra_sv"]) if int(r["ira"]) else -1] for r in rows])
    return b[np.argsort(b[:, 0])]


def score(fixes, D, to_utc, tag):
    gps = D["gps"]
    t = np.array([f[0] for f in fixes])
    rr, vr, lat, lon, alt, spd = T.truth_at(gps, LAG, t)
    ok = np.isfinite(rr[:, 0])
    lat_f = np.array([f[1] for f in fixes]); lon_f = np.array([f[2] for f in fixes])
    hacc = np.array([f[3] for f in fixes]); nt = np.array([f[4] for f in fixes])
    dn = np.radians(lat_f - lat) * 6371000.0
    de = np.radians(lon_f - lon) * 6371000.0 * np.cos(np.radians(lat))
    err = np.hypot(dn, de)
    has = ok & (nt > 0) & (lat_f != 0)
    phases = np.array([phase_of(s, h) for s, h in zip(spd, alt)])
    rows = []
    for ph in ("all", "ground", "taxi/road", "climb/descent", "cruise"):
        m = has & ((phases == ph) if ph != "all" else True)
        if m.sum() < 10:
            continue
        e = err[m]
        rows.append((tag, ph, int(m.sum()), np.median(e), np.percentile(e, 95), e.max(),
                     np.mean(e <= 2 * hacc[m]) * 100, np.median(hacc[m])))
    return rows, err, has


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("npz"); ap.add_argument("tle"); ap.add_argument("bursts")
    ap.add_argument("--board-bursts", action="store_true")
    ap.add_argument("--mode", default="blind", choices=("blind", "anchored"))
    ap.add_argument("--start-sigma", type=float, default=5000.0)
    ap.add_argument("--start-offset", type=float, nargs=2)
    ap.add_argument("--vel", default="ublox", choices=("ublox", "none"))
    ap.add_argument("--vel-noise", type=float, default=0.0)
    ap.add_argument("--vel-hold", action="store_true",
                    help="no u-blox velocity once GPS is lost: the last one held, and Doppler corrects it")
    ap.add_argument("--alt", default="none", choices=("truth", "none"))
    ap.add_argument("--alt-sigma", type=float, default=30.0)
    ap.add_argument("--gps-until", type=float)
    ap.add_argument("--from", dest="from_h", type=float)
    ap.add_argument("--to", dest="to_h", type=float)
    ap.add_argument("--cfg", action="append", default=[])
    ap.add_argument("--out")
    ap.add_argument("--tag", default="run")
    ap.add_argument("--lag", type=float, default=0.075,
                    help="u-blox time-tag lag against the burst stamps, s (2026-11-20 calibration: -0.03)")
    a = ap.parse_args()
    global LAG
    LAG = a.lag
    D = np.load(a.npz)
    bursts = load_bursts(a.bursts, a.board_bursts)
    obs = (a.out or "/tmp/soop_nav_replay") + ".obs"
    n_b, to_utc = build(a, D, bursts, obs)
    subprocess.run(["make", "-s", "-C", os.path.dirname(NAV_HOST), "nav_host"], check=True)
    extra = sum((["--cfg", c] for c in a.cfg), [])
    if a.vel == "none":
        extra += ["--vel-sigma", "250"]
    r = subprocess.run([NAV_HOST, a.tle, obs] + extra, capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(r.stderr)
    fixes = []
    for ln in r.stdout.splitlines()[1:]:
        p = ln.split(",")
        if p[0] == "F":
            fixes.append((float(p[1]), float(p[2]), float(p[3]), float(p[8]), int(p[11])))
    rows, err, has = score(fixes, D, to_utc, a.tag)
    print(r.stderr.strip().splitlines()[0])
    print("%-14s %-13s %6s %8s %8s %8s %7s %8s" % ("run", "phase", "fixes", "median", "p95", "max",
                                                    "in2sig", "hacc"))
    for row in rows:
        print("%-14s %-13s %6d %8.0f %8.0f %8.0f %6.0f%% %8.0f" % row)
    if a.out:
        with open(a.out, "w") as f:
            f.write("t,utc,lat,lon,hacc,nt,err\n")
            for fx, e, h in zip(fixes, err, has):
                f.write("%.3f,%.3f,%.7f,%.7f,%.1f,%d,%s\n" % (fx[0], to_utc(fx[0]), fx[1], fx[2], fx[3],
                                                             fx[4], "%.1f" % e if h else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
