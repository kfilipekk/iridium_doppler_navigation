#!/usr/bin/env python3
"""Pull what the Iridium analysis needs out of a board log into one .npz, once.

Usage:
  soop_extract.py LOG.BIN [OUT.npz]

A dataflash log with IMU_FAST is hundreds of MB and pymavlink reads it at a few MB/s, so the
analysis tools (soop_truth.py) work from this instead. Arrays, all on board time t (s):
  gps   t, status, nsats, lat, lon, alt(MSL), spd, crs, vz, hacc, sacc, week, ms
  sob   t_burst, f, sigma, cn0, res, sat, ch, inn       (every burst the DSP measured)
  sos   t, cpu, ovr, nt, clk, fused, amb, unm, nse      (the navigation's status, 1 Hz)
  sof   t, lat, lon, alt, hacc, nt, valid, still        (the board's own Iridium fix)
  imu   t, gx, gy, gz, ax, ay, az, temp                 (IMU 0, the mean of each IMU_DEC samples)
  imu1  the same for IMU 1 (the ICM-42605)
  baro  t, alt, press, temp
  xkf   t, vn, ve, vd                                   (EKF velocity, core 0)
  soi   t, file, first_sample, board_t                  (I/Q file starts)
  sod   t, first_sample                                 (I/Q blocks the recorder dropped)
  sog   every SOG field                                 (capture gaps the DSP saw)
"""
import os
import sys

import numpy as np

IMU_DEC = 2         #400 Hz -> 200 Hz, averaged: a point sample of every 4th aliases a bag being
                    #handled (3.4 rad/s on 2026-11-19) into an attitude error the INS keeps


def extract(path):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(path)
    out = {k: [] for k in ("gps", "sob", "sos", "sof", "imu", "imu1", "baro", "xkf", "soi", "sod", "sog")}
    hacc = sacc = np.nan
    acc_imu = {0: [], 1: []}
    want = ["GPS", "GPA", "SOB", "SOS", "SOF", "IMU", "BARO", "XKF1", "SOI", "SOD", "SOG"]
    while True:
        x = m.recv_match(type=want)
        if x is None:
            break
        k = x.get_type()
        t = x.TimeUS / 1e6
        if k == "GPA":
            if x.I == 0:
                hacc, sacc = x.HAcc, x.SAcc
        elif k == "GPS":
            if x.I == 0:
                out["gps"].append((t, x.Status, x.NSats, x.Lat, x.Lng, x.Alt, x.Spd, x.GCrs,
                                   x.VZ, hacc, sacc, x.GWk, x.GMS))
        elif k == "SOB":
            out["sob"].append((x.T, x.F, x.Sig, x.CN0, x.Res, x.Sat, x.Ch, x.Inn))
        elif k == "SOS":
            out["sos"].append((t, x.CPU, x.Ovr, x.NT, x.Clk, x.Fused, x.Amb, x.Unm, x.Nse))
        elif k == "SOF":
            out["sof"].append((t, x.Lat, x.Lng, x.Alt, x.HAcc, x.NT, x.V, x.St))
        elif k == "IMU":
            if x.I in acc_imu:
                q = acc_imu[x.I]
                q.append((t, x.GyrX, x.GyrY, x.GyrZ, x.AccX, x.AccY, x.AccZ, x.T))
                if len(q) == IMU_DEC:
                    out["imu" if x.I == 0 else "imu1"].append(tuple(np.mean(q, axis=0)))
                    q.clear()
        elif k == "BARO":
            if x.I == 0:
                out["baro"].append((t, x.Alt, x.Press, x.Temp))
        elif k == "XKF1":
            if x.C == 0:
                out["xkf"].append((t, x.VN, x.VE, x.VD))
        elif k == "SOI":
            out["soi"].append((t, x.N, x.S, x.T))
        elif k == "SOD":
            out["sod"].append((t, x.S))
        elif k == "SOG":
            out["sog"].append(tuple(float(getattr(x, f)) for f in x.get_fieldnames()))
    return {k: np.array(v, dtype=float) for k, v in out.items()}


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + ".npz"
    d = extract(src)
    np.savez_compressed(dst, **d)
    print(dst, {k: v.shape for k, v in d.items()})
    return 0


if __name__ == "__main__":
    sys.exit(main())
