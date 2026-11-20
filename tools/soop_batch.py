#!/usr/bin/env python3
"""What a window of Doppler bursts can say about position, with no filter in the way.

Usage:
  soop_batch.py LOG.npz TLE TRUTH.npz [--win S] [--step S] [--start-off M] [--vel ublox|zero]
                [--alt-sigma M] [--beta-sigma HZ] [--sigma HZ] [--channels RA|all]

Takes the bursts soop_truth.py associated (satellite and channel, from TRUTH.npz) - so this
is the ceiling association allows, not what the filter will reach - and, per window, solves by
Gauss-Newton for the receiver's position at the window's middle, its clock and clock drift, and
each satellite's transmit offset (prior 0 +- beta-sigma). The receiver moves with the u-blox
velocity (--vel ublox; a moving test is otherwise underdetermined in a window) and its height is
held to the u-blox's within --alt-sigma (an air-data or barometric height stands in). The start
is the truth moved --start-off metres north. Prints the horizontal error and the formal sigma.
"""
import argparse
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import soop_truth as T


def solve(t, f, sat_i, ch, im, rr_off, vr, R_fn, r0, sig, beta_sig, alt_sig, up, h_true, iters=12,
          temp=None, knots=None, rate=None, tau_sig=0.0):
    """x = [dr(3), c0, c1, beta(n_sat), kT?, clock knots?]: receiver at r0 + dr + (its truth-
    relative path). The TCXO follows the board's temperature (~25-33 Hz/degC, 2026-11-17/18), so
    with temp given the clock carries kT * (T - mean); with knots (s) it is piecewise linear."""
    sats = np.unique(sat_i)
    smap = {s: k for k, s in enumerate(sats)}
    ns = len(sats)
    extra = []
    if temp is not None:
        extra.append(temp - np.mean(temp))
    if knots:
        tm0 = np.median(t)
        for kn in np.arange(t.min() + knots, t.max(), knots):
            extra.append(np.maximum(t - kn, 0.0))         #hinge: a change of drift at kn
    E = np.array(extra).T if extra else np.zeros((len(t), 0))
    ne = E.shape[1]
    #each satellite's along-track timing (s): old elements put it km off along its path, a
    #Doppler error of (Doppler rate) x tau that one pass's bursts can separate from position
    nt_ = ns if tau_sig > 0 and rate is not None else 0
    x = np.zeros(5 + ns + ne + nt_)
    tm = np.median(t)
    for _ in range(iters):
        rr = r0 + x[:3] + rr_off                          #receiver at each burst
        R, V = R_fn
        d = R - rr
        rho = np.linalg.norm(d, axis=1)
        u = d / rho[:, None]
        rdot = np.sum((V - vr) * u, 1)
        fch = T.f_ch(ch)
        dop = -fch / T.C * rdot
        sgn = np.where(im > 0, -1.0, 1.0)
        meas = sgn * f + T.LO_OFFSET - ch * T.SPACING      #-> Doppler + clock + beta
        bi = np.array([smap[s] for s in sat_i])
        pred = dop + x[3] + x[4] * (t - tm) + x[5 + bi] + (E @ x[5 + ns:5 + ns + ne] if ne else 0.0)
        if nt_:
            pred = pred + rate * x[5 + ns + ne + bi]
        y = meas - pred
        #d dop / d receiver position: (fch/c) * (dv - (u.dv) u) / rho, dv = V - vr
        dv = V - vr
        gr = (fch / T.C)[:, None] * (dv - np.sum(u * dv, 1)[:, None] * u) / rho[:, None]
        H = np.zeros((len(t), 5 + ns + ne + nt_))
        H[:, :3] = gr
        H[:, 3] = 1.0
        H[:, 4] = t - tm
        H[np.arange(len(t)), 5 + bi] = 1.0
        if ne:
            H[:, 5 + ns:5 + ns + ne] = E
        if nt_:
            H[np.arange(len(t)), 5 + ns + ne + bi] = rate
        #robust weights (Huber at 2.5 sigma), priors on beta and height
        r_ = y / sig
        w = np.where(np.abs(r_) < 2.5, 1.0, 2.5 / np.abs(r_))
        A = H * (np.sqrt(w) / sig)[:, None]
        b = y * np.sqrt(w) / sig
        P_rows, P_b = [], []
        for k in range(ns):
            row = np.zeros(5 + ns + ne + nt_); row[5 + k] = 1.0 / beta_sig
            P_rows.append(row); P_b.append(-x[5 + k] / beta_sig)
            if nt_:
                row = np.zeros(5 + ns + ne + nt_); row[5 + ns + ne + k] = 1.0 / tau_sig
                P_rows.append(row); P_b.append(-x[5 + ns + ne + k] / tau_sig)
        row = np.zeros(5 + ns + ne + nt_); row[:3] = up / alt_sig
        P_rows.append(row); P_b.append((h_true - np.dot(up, r0 + x[:3])) / alt_sig)
        A = np.vstack([A, np.array(P_rows)])
        b = np.concatenate([b, np.array(P_b)])
        dx, *_ = np.linalg.lstsq(A, b, rcond=None)
        x += dx
        if np.linalg.norm(dx[:3]) < 0.01:
            break
    cov = np.linalg.pinv(A.T @ A)
    return x, cov, w


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("npz"); ap.add_argument("tle"); ap.add_argument("truth")
    ap.add_argument("--win", type=float, default=600.0)
    ap.add_argument("--step", type=float, default=300.0)
    ap.add_argument("--start-off", type=float, default=5000.0)
    ap.add_argument("--alt-sigma", type=float, default=30.0)
    ap.add_argument("--beta-sigma", type=float, default=20.0)
    ap.add_argument("--sigma", type=float, default=20.0)
    ap.add_argument("--channels", default="RA", choices=("RA", "all"))
    ap.add_argument("--min", type=int, default=40)
    ap.add_argument("--lag", type=float, default=0.075)
    ap.add_argument("--temp", action="store_true", help="clock follows the IMU temperature")
    ap.add_argument("--knots", type=float, help="piecewise-linear clock, a knot every S s")
    ap.add_argument("--tau", type=float, default=0.0, help="per-satellite along-track timing prior, s")
    a = ap.parse_args()
    D = np.load(a.npz); R_ = np.load(a.truth)
    gps = D["gps"]
    to_utc, _, _ = T.time_map(gps, a.lag)
    sats, norad = T.load_tles(a.tle)
    nidx = {n: i for i, n in enumerate(norad)}
    m = np.isfinite(R_["resid"]) & (R_["norad"] >= 0)
    if a.channels == "RA":
        m &= (R_["ch"] == 0) & (R_["image"] == 0)
    t_all = R_["t"][m]; f_all = R_["f"][m]; s_all = R_["norad"][m]
    ch_all = R_["ch"][m].astype(float); im_all = R_["image"][m]
    out = []
    for t0 in np.arange(t_all.min(), t_all.max() - a.win / 2, a.step):
        k = (t_all >= t0) & (t_all < t0 + a.win)
        if k.sum() < a.min:
            continue
        t, f, s, ch, im = t_all[k], f_all[k], s_all[k], ch_all[k], im_all[k]
        rr, vr, lat, lon, alt, spd = T.truth_at(gps, a.lag, t)
        ok = np.isfinite(rr[:, 0])
        t, f, s, ch, im, rr, vr = t[ok], f[ok], s[ok], ch[ok], im[ok], rr[ok], vr[ok]
        if len(t) < a.min:
            continue
        mid = np.argmin(np.abs(t - np.median(t)))
        r_mid = rr[mid]
        rr_off = rr - r_mid                               #the path, from the u-blox velocity
        e, n, up = T.enu_basis(np.array([lat[ok][mid]]), np.array([lon[ok][mid]]))
        r0 = r_mid + a.start_off * n[0]
        R, V = T.sat_ecef(sats, to_utc(t))
        si = np.array([nidx[x] for x in s]); ii = np.arange(len(t))
        Rs, Vs = R[si, ii], V[si, ii]
        temp = np.interp(t, D["imu"][:, 0], D["imu"][:, 7]) if a.temp else None
        rate = None
        if a.tau > 0:                                     #each burst's Doppler rate, Hz/s
            R2, V2 = T.sat_ecef(sats, to_utc(t) + 0.5)
            Rs2, Vs2 = R2[si, ii], V2[si, ii]
            d1, d2 = Rs - rr, Rs2 - rr
            u1 = d1 / np.linalg.norm(d1, axis=1)[:, None]; u2 = d2 / np.linalg.norm(d2, axis=1)[:, None]
            rate = -T.F_RA / T.C * (np.sum((Vs2 - vr) * u2, 1) - np.sum((Vs - vr) * u1, 1)) / 0.5
        x, cov, w = solve(t, f, s, ch, im, rr_off, vr, (Rs, Vs), r0, a.sigma, a.beta_sigma,
                          a.alt_sigma, up[0], np.dot(up[0], r_mid), temp=temp, knots=a.knots,
                          rate=rate, tau_sig=a.tau)
        dr = r0 + x[:3] - r_mid
        en = np.array([np.dot(dr, e[0]), np.dot(dr, n[0])])
        Ch = np.array([[e[0] @ cov[:3, :3] @ e[0], e[0] @ cov[:3, :3] @ n[0]],
                       [n[0] @ cov[:3, :3] @ e[0], n[0] @ cov[:3, :3] @ n[0]]])
        u0 = to_utc(np.median(t)) % 86400
        out.append((u0, len(t), len(np.unique(s)), np.hypot(*en), np.sqrt(np.trace(Ch)), en[0], en[1],
                    x[3], np.median(spd[ok]), np.median(alt[ok])))
        print("%02d:%02d  n %4d sats %2d  err %7.0f m (E %+6.0f N %+6.0f)  sigma %6.0f  clk %+6.0f Hz  "
              "spd %3.0f alt %5.0f" % (u0 // 3600, u0 % 3600 // 60, len(t), out[-1][2], out[-1][3],
                                       en[0], en[1], out[-1][4], x[3], out[-1][8], out[-1][9]))
    if out:
        e = np.array([o[3] for o in out])
        print(f"windows {len(out)}: median {np.median(e):.0f} m, p95 {np.percentile(e, 95):.0f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
