#!/usr/bin/env python3
"""Replay a board's dataflash log through the navigation on the host, the way the board fed it.

Usage:
  soop_replay.py LOG.BIN TLE [--ref LAT LON] [--out DIR] [--cfg KEY=VAL ...] [--static]

The board logs every burst the DSP measured (SOB) and every vehicle sample the navigation
used is in the log too: EKF velocity (XKF1), barometer (BARO) and the u-blox (GPS, GPA). This
rebuilds nav_host's input from them, merged by board time as AP_SoOP does, so the flight C
code can run again with one thing changed (--cfg, firmware/soop/nav_host.c) and be scored
against the reference. The GPS is withheld wherever the board's own u-blox had no good fix,
which is where a soop_session.py plan denied it.
"""
import argparse
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SOOP = os.path.join(os.path.dirname(HERE), "firmware", "soop")
sys.path.insert(0, HERE)
import soop_solver as sol

STILL_GYRO = 0.03       #rad/s, and
STILL_ACC = 0.3         #m/s^2 off gravity, and steady:
STILL_ACC_SD = 0.1      #m/s^2 of |accel| spread, with
STILL_GPS_SPD = 1.0     #the u-blox under this when it has a fix,
MOVING_SPD = 3.0        #(its last word over this latching 'moving'), held for
STILL_HOLD_S = 2.0      #as AP_SoOP's detector
UNDULATION_M = 45.7     #EGM96 at Cambridge; the board logs MSL altitude, the filter wants WGS84
J2000_JD = 2451545.0


def gps_good(g, hacc):
    #AP_SoOP.cpp: a 3D fix, 6 satellites, hacc under 5 m and a known week
    return g.Status >= 3 and g.NSats >= 6 and hacc < 5.0 and g.GWk > 0


def build_obs(path, out_path, deny=(), still=True):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(path)
    lines, utc_day0 = [], None
    hacc, baro_off, n = 99.0, None, dict(D=0, E=0, G=0, A=0, Z=0)
    last_e = last_a = -1.0
    still_since, last_z, armed = None, -1.0, False
    acc_m = acc_m2 = None
    moving, gps_rest = False, True
    while True:
        x = m.recv_match(type=["GPS", "GPA", "BARO", "XKF1", "SOB", "IMU", "ARM"])
        if x is None:
            break
        k = x.get_type()
        t = x.TimeUS / 1e6
        if k == "ARM":
            armed = bool(x.ArmState)
        elif k == "IMU" and x.I == 0 and still:
            gyro = math.sqrt(x.GyrX ** 2 + x.GyrY ** 2 + x.GyrZ ** 2)
            acc = math.sqrt(x.AccX ** 2 + x.AccY ** 2 + x.AccZ ** 2)
            #the board's running spread is over ~1 s at 50 Hz; the log has IMU at 25 Hz
            a = 0.1
            acc_m = acc if acc_m is None else acc_m + a * (acc - acc_m)
            acc_m2 = acc * acc if acc_m2 is None else acc_m2 + a * (acc * acc - acc_m2)
            acc_sd = math.sqrt(max(0.0, acc_m2 - acc_m * acc_m))
            if (armed or gyro > STILL_GYRO or abs(acc - 9.80665) > STILL_ACC or acc_sd > STILL_ACC_SD
                    or not gps_rest):
                still_since = None
            elif still_since is None:
                still_since = t
            elif t - still_since >= STILL_HOLD_S and t - last_z >= 0.5:
                lines.append((t, "Z %.6f" % t))
                last_z = t
                n["Z"] += 1
        elif k == "GPA" and x.I == 0:
            hacc = x.HAcc
        elif k == "GPS" and x.I == 0 and x.GWk > 0:
            #GPS time -> UTC seconds since J2000 (18 leap seconds since 2017)
            gps_s = x.GWk * 604800.0 + x.GMS / 1000.0 - 18.0
            utc_j2000 = gps_s + (2444244.5 - J2000_JD) * 86400.0
            if utc_day0 is None:
                utc_day0 = math.floor((utc_j2000 + 43200.0) / 86400.0) * 86400.0 - 43200.0
            sod = (utc_j2000 - utc_day0) % 86400.0
            if gps_good(x, hacc):
                if x.Spd > MOVING_SPD:
                    moving = True
                elif x.Spd < STILL_GPS_SPD:
                    moving = False
                gps_rest = x.Spd < STILL_GPS_SPD
            else:
                gps_rest = not moving
            denied = any(a <= sod <= b for a, b in deny)
            if gps_good(x, hacc) and not denied:
                h = x.Alt + UNDULATION_M
                r = sol.geodetic_to_ecef(x.Lat, x.Lng, h)
                lines.append((t, "G %.6f %.6f %.3f %.3f %.3f" % (t, utc_j2000 - utc_day0, *r)))
                n["G"] += 1
                if baro_off is None and last_a >= 0:
                    baro_off = h - last_baro
        elif k == "XKF1" and x.C == 0 and t - last_e >= 0.1:
            lines.append((t, "E %.6f %.4f %.4f %.4f" % (t, x.VN, x.VE, x.VD)))
            last_e = t
            n["E"] += 1
        elif k == "BARO" and x.I == 0:
            last_baro = x.Alt
            if baro_off is not None and t - last_a >= 0.5:
                lines.append((t, "A %.6f %.3f" % (t, x.Alt + baro_off)))
                n["A"] += 1
            if last_a < 0 or t - last_a >= 0.5:
                last_a = t
        elif k == "SOB":
            lines.append((x.T, "D %.6f %.3f %.3f %.2f" % (x.T, x.F, x.Sig, x.CN0)))
            n["D"] += 1
    lines.sort(key=lambda r: r[0])
    jd = J2000_JD + utc_day0 / 86400.0
    with open(out_path, "w") as f:
        f.write("H %.1f %.12f\n" % (math.floor(jd - 0.5) + 0.5, jd - (math.floor(jd - 0.5) + 0.5)))
        f.write("\n".join(l for _, l in lines) + "\n")
    return n, utc_day0


def run(obs, tle, extra):
    exe = os.path.join(SOOP, "nav_host")
    subprocess.run(["make", "-s", "-C", SOOP, "nav_host"], check=True)
    r = subprocess.run([exe, tle, obs] + extra, capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(r.stderr)
    fixes = []
    for ln in r.stdout.splitlines()[1:]:
        p = ln.split(",")
        if p[0] == "F":
            fixes.append(dict(t=float(p[1]), lat=float(p[2]), lon=float(p[3]),
                              hacc=float(p[8]), nis=float(p[10]), nt=int(p[11]), valid=int(p[12])))
    return fixes, r.stderr


def score(fixes, obs_path, ref, windows):
    """Horizontal error against ref over each window (UTC seconds of day), from board time
    mapped to UTC through the obs file's GPS records."""
    import numpy as np
    tb, tu = [], []
    for ln in open(obs_path):
        if ln[0] == "G":
            p = ln.split()
            tb.append(float(p[1]))
            tu.append(float(p[2]))
    k, c = np.polyfit(tb, tu, 1)
    out = []
    for s, e in windows:
        errs, hacc = [], []
        for f in fixes:
            u = k * f["t"] + c
            if s + 60 <= u <= e and f["lat"]:
                dn = math.radians(f["lat"] - ref[0]) * 6371000.0
                de = math.radians(f["lon"] - ref[1]) * 6371000.0 * math.cos(math.radians(ref[0]))
                errs.append(math.hypot(dn, de))
                hacc.append(f["hacc"])
        errs_s = sorted(errs)
        q = lambda p: errs_s[min(len(errs_s) - 1, int(p * len(errs_s)))] if errs_s else float("nan")
        out.append(dict(n=len(errs), median=q(.5), p95=q(.95), max=max(errs or [0]),
                        inside=sum(1 for x, h in zip(errs, hacc) if x <= 2 * h) / max(1, len(errs))))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log")
    ap.add_argument("tle")
    ap.add_argument("--ref", type=float, nargs=2, metavar=("LAT", "LON"))
    ap.add_argument("--out", default=".")
    ap.add_argument("--cfg", action="append", default=[])
    ap.add_argument("--static", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    obs = os.path.join(a.out, "obs.txt")
    n, _ = build_obs(a.log, obs)
    print(f"obs: {n}")
    extra = sum((["--cfg", c] for c in a.cfg), []) + (["--static"] if a.static else [])
    fixes, err = run(obs, a.tle, extra)
    print(err.strip())
    print(f"{len(fixes)} fixes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
