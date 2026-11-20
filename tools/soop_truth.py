#!/usr/bin/env python3
"""Label every burst the board heard with its satellite and channel, from the u-blox truth.

Usage:
  soop_truth.py LOG.npz TLE [--bursts REP.csv] [--lag S] [--out OUT.npz] [--mask DEG]

--bursts takes soop_reprocess.py's bursts in place of the board's own (SOB).

No navigation filter is involved: with the receiver's true position and velocity known (the
u-blox, interpolated to each burst), every visible satellite's Doppler on each simplex channel
can be predicted, and what is left between a burst and its true source is the board's clock
offset (common to all satellites, slowly varying) plus the satellite's own transmit offset and
the measurement noise. So the clock is found first, as the peak of all burst-candidate
residuals in a sliding window, and each burst then goes to the candidate nearest that clock.

Out (npz): per burst - board t, UTC, f, sigma, cn0, the board's result, matched satellite
(NORAD, -1 none), channel, residual after the clock, elevation, azimuth, range rate, the
receiver's speed and altitude; and the clock track (t, Hz, bursts in the window).
Model, as soop_nav.c predict(): F + LO_OFFSET - (f_ch - f_RA) = -f_ch/c * rdot + beta + clock.
Every simplex channel 1626.0-1626.5 MHz is a candidate (ch -6..5, Ring Alert 0, 41.667 kHz
apart), straight and as its image about the LO (F -> -F: what I/Q imbalance, or a spectrum the
wrong way up, puts in the band the DSP searches).
"""
import argparse
import math
import os
import sys

import numpy as np

C = 299792458.0
F_RA = 1626270833.0
SPACING = 41666.667
CHANNELS = np.arange(-6, 6)  #simplex channels by offset from Ring Alert, x SPACING
LO_OFFSET = 62500.0
GPS_UNIX = 315964800.0
LEAP = 18.0
WE = 7.2921150e-5
CLK_SPAN = 12000.0          #Hz either side searched for the clock (a few ppm at 1.6 GHz)
CLK_BIN = 25.0
CLK_WIN = 180.0             #s of bursts behind each clock estimate
MATCH_HZ = 250.0            #a burst this close to clock + Doppler is that satellite's


def f_ch(j):
    return F_RA + j * SPACING


#candidate hypotheses: (channel offset j, image 0/1)
HYP = [(int(j), im) for im in (0, 1) for j in CHANNELS]


def load_tles(path):
    from sgp4.api import Satrec
    lines = [l.rstrip() for l in open(path)]
    sats, norad = [], []
    for i in range(len(lines) - 1):
        if lines[i].startswith("1 ") and lines[i + 1].startswith("2 "):
            s = Satrec.twoline2rv(lines[i], lines[i + 1])
            sats.append(s)
            norad.append(s.satnum)
    return sats, np.array(norad)


def gmst(jd):
    d = jd - 2451545.0
    T = d / 36525.0
    g = 280.46061837 + 360.98564736629 * d + 0.000387933 * T * T - T ** 3 / 38710000.0
    return np.radians(g % 360.0)


def sat_ecef(sats, unix):
    """(n_sat, n_t, 3) positions and velocities, metres, Earth-fixed."""
    from sgp4.api import SatrecArray
    jd_full = unix / 86400.0 + 2440587.5
    jd = np.floor(jd_full)
    fr = jd_full - jd
    e, r, v = SatrecArray(sats).sgp4(jd, fr)
    r, v = r * 1000.0, v * 1000.0
    th = gmst(jd_full)
    c, s = np.cos(th), np.sin(th)
    x = c * r[..., 0] + s * r[..., 1]
    y = -s * r[..., 0] + c * r[..., 1]
    vx = c * v[..., 0] + s * v[..., 1] + WE * y
    vy = -s * v[..., 0] + c * v[..., 1] - WE * x
    R = np.stack([x, y, r[..., 2]], -1)
    V = np.stack([vx, vy, v[..., 2]], -1)
    R[e != 0] = np.nan
    return R, V


def geo_ecef(lat, lon, h):
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    la, lo = np.radians(lat), np.radians(lon)
    N = a / np.sqrt(1 - e2 * np.sin(la) ** 2)
    return np.stack([(N + h) * np.cos(la) * np.cos(lo), (N + h) * np.cos(la) * np.sin(lo),
                     (N * (1 - e2) + h) * np.sin(la)], -1)


def enu_basis(lat, lon):
    la, lo = np.radians(lat), np.radians(lon)
    e = np.stack([-np.sin(lo), np.cos(lo), np.zeros_like(lo)], -1)
    n = np.stack([-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)], -1)
    u = np.stack([np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)], -1)
    return e, n, u


def time_map(gps, lag):
    """board t -> unix UTC: the epoch a u-blox record describes is `lag` before it arrived."""
    ok = (gps[:, 1] >= 3) & (gps[:, 11] > 2000)
    g = gps[ok]
    utc = g[:, 11] * 604800.0 + g[:, 12] / 1000.0 + GPS_UNIX - LEAP
    tb = g[:, 0] - lag
    k, c = np.polyfit(tb - tb[0], utc, 1)
    res = utc - (k * (tb - tb[0]) + c)
    return (lambda t: k * (t - tb[0]) + c), float(np.std(res)), (k - 1.0) * 1e6


def truth_at(gps, lag, tq):
    """The u-blox position (ECEF) and velocity (ECEF), interpolated to board times tq; nan
    outside a good fix (or across a gap over 2 s)."""
    ok = gps[:, 1] >= 3
    g = gps[ok]
    te = g[:, 0] - lag
    idx = np.clip(np.searchsorted(te, tq), 1, len(te) - 1)
    gap = (te[idx] - te[idx - 1] > 2.0) | (tq < te[0]) | (tq > te[-1])
    w = np.clip((tq - te[idx - 1]) / np.maximum(te[idx] - te[idx - 1], 1e-6), 0, 1)
    lerp = lambda col: g[idx - 1, col] + w * (g[idx, col] - g[idx - 1, col])
    lat, lon, alt = lerp(3), lerp(4), lerp(5)
    spd, vz = lerp(6), lerp(8)
    #course: interpolate on the circle
    c0, c1 = np.radians(g[idx - 1, 7]), np.radians(g[idx, 7])
    crs = np.arctan2((1 - w) * np.sin(c0) + w * np.sin(c1), (1 - w) * np.cos(c0) + w * np.cos(c1))
    h = alt + 45.0                          #MSL -> ellipsoid; the geoid hardly moves Doppler
    r = geo_ecef(lat, lon, h)
    e, n, u = enu_basis(lat, lon)
    vn, ve, vd = spd * np.cos(crs), spd * np.sin(crs), vz
    v = n * vn[:, None] + e * ve[:, None] - u * vd[:, None]
    r[gap] = np.nan
    return r, v, lat, lon, alt, spd


def residuals(sob_t, sob_f, R, V, rr, vr, lat, lon, mask_deg):
    """Every (hypothesis, satellite, burst) clock estimate, with elevation: for channel j straight,
    F = j*SPACING - LO_OFFSET + D + clock; as its image, -F."""
    d = R - rr[None]                                  #(sat, burst, 3)
    rho = np.linalg.norm(d, axis=-1)
    u = d / rho[..., None]
    rdot = np.sum((V - vr[None]) * u, -1)             #m/s
    e, n, up = enu_basis(lat, lon)
    el = np.degrees(np.arcsin(np.sum(u * up[None], -1)))
    az = np.degrees(np.arctan2(np.sum(u * e[None], -1), np.sum(u * n[None], -1))) % 360
    res = np.empty((len(HYP),) + rdot.shape)
    for h, (j, im) in enumerate(HYP):
        F = -sob_f if im else sob_f
        res[h] = F[None, :] + LO_OFFSET - j * SPACING - (-f_ch(j) / C * rdot)
    vis = (el > mask_deg) & np.isfinite(rdot)
    return res, vis, el, az, rdot


def clock_track(t, res, vis):
    """The clock as the histogram peak of all visible candidates' residuals, every 30 s over
    CLK_WIN behind and ahead; then smoothed through the windows that had a clear peak."""
    edges = np.arange(-CLK_SPAN, CLK_SPAN + CLK_BIN, CLK_BIN)
    centres = np.arange(t.min(), t.max() + 30.0, 30.0)
    out = []
    for tc in centres:
        sel = np.abs(t - tc) <= CLK_WIN / 2
        if sel.sum() < 6:
            continue
        r = res[:, :, sel][vis[None, :, sel].repeat(len(HYP), 0)]
        h, _ = np.histogram(r, edges)
        h2 = h[:-1] + h[1:]                            #two-bin peak, as acquire()
        k = int(np.argmax(h2))
        best = h2[k]
        second = np.max(np.r_[h2[:max(k - 3, 0)], h2[k + 4:]]) if len(h2) > 8 else 0
        mid = edges[k + 1]
        near = r[np.abs(r - mid) < 2 * CLK_BIN]
        out.append((tc, float(np.median(near)) if len(near) else mid, int(best), int(second),
                    int(sel.sum())))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("npz")
    ap.add_argument("tle")
    ap.add_argument("--lag", type=float, default=0.075)
    ap.add_argument("--mask", type=float, default=-2.0)
    ap.add_argument("--out")
    ap.add_argument("--bursts")
    a = ap.parse_args()
    D = np.load(a.npz)
    gps, sob = D["gps"], D["sob"]
    extra = {}
    if a.bursts:
        import csv
        rows = list(csv.DictReader(open(a.bursts)))
        sob = np.array([[float(r["t"]), float(r["f_hz"]), float(r["f_sigma_hz"]),
                         float(r["cn0_dbhz"]), -1, int(r["ra_sv"]) if int(r["ira"]) else -1, -1, 0]
                        for r in rows])
        extra = dict(ira=np.array([int(r["ira"]) for r in rows]),
                     ra_sv=np.array([int(r["ra_sv"]) for r in rows]),
                     ra_beam=np.array([int(r["ra_beam"]) for r in rows]),
                     ra_xyz=np.array([[float(r["ra_x"]), float(r["ra_y"]), float(r["ra_z"])]
                                      for r in rows]),
                     len_s=np.array([float(r["len_s"]) for r in rows]))
        o = np.argsort(sob[:, 0])
        sob = sob[o]
        extra = {k: v[o] for k, v in extra.items()}
    to_utc, fit_sd, ppm = time_map(gps, a.lag)
    print(f"time map: board clock {ppm:+.2f} ppm, fit residual {fit_sd * 1e3:.1f} ms")
    rr, vr, lat, lon, alt, spd = truth_at(gps, a.lag, sob[:, 0])
    have = np.isfinite(rr[:, 0])
    sob = sob[have]
    extra = {k: v[have] for k, v in extra.items()}
    rr, vr, lat, lon, alt, spd = rr[have], vr[have], lat[have], lon[have], alt[have], spd[have]
    utc = to_utc(sob[:, 0])
    print(f"bursts with truth: {len(sob)} of {len(have)}")
    sats, norad = load_tles(a.tle)
    R, V = sat_ecef(sats, utc)
    res, vis, el, az, rdot = residuals(sob[:, 0], sob[:, 1], R, V, rr, vr, lat, lon, a.mask)
    ck = clock_track(sob[:, 0], res, vis)
    good = ck[:, 2] >= 2 * np.maximum(ck[:, 3], 1)
    print(f"clock windows: {len(ck)}, clear peak in {good.sum()}")
    clk = np.interp(sob[:, 0], ck[good, 0], ck[good, 1]) if good.any() else np.zeros(len(sob))
    #associate: nearest candidate to the clock, unambiguous
    r_c = np.abs(res - clk[None, None, :])
    r_c[~vis[None].repeat(len(HYP), 0)] = np.inf
    flat = r_c.reshape(-1, r_c.shape[-1])
    order = np.argsort(flat, axis=0)
    b0 = flat[order[0], np.arange(flat.shape[1])]
    b1 = flat[order[1], np.arange(flat.shape[1])]
    hit = (b0 < MATCH_HZ) & (b1 > 3 * b0)
    h_i, sat_i = np.unravel_index(order[0], (len(HYP), len(sats)))
    ch_i = np.array([HYP[h][0] for h in h_i])
    im_i = np.array([HYP[h][1] for h in h_i])
    idx = np.arange(len(sob))
    out = dict(
        t=sob[:, 0], utc=utc, f=sob[:, 1], sigma=sob[:, 2], cn0=sob[:, 3], res_board=sob[:, 4],
        sat_board=sob[:, 5], norad=np.where(hit, norad[sat_i], -1), ch=np.where(hit, ch_i, -99),
        image=np.where(hit, im_i, -1),
        resid=np.where(hit, res[h_i, sat_i, idx] - clk, np.nan),
        el=el[sat_i, idx], az=az[sat_i, idx], rdot=rdot[sat_i, idx], clk=clk,
        spd=spd, alt=alt, lat=lat, lon=lon, clock_track=ck,
        n_vis=vis.sum(0), **extra)
    m = hit
    print(f"matched {m.sum()} of {len(sob)} ({100 * m.mean():.1f}%), ambiguous/none "
          f"{(~m).sum()}")
    if m.any():
        r = out["resid"][m]
        print(f"residual after the clock: median |r| {np.median(np.abs(r)):.1f} Hz, "
              f"sd {np.std(r):.1f}, 95% {np.percentile(np.abs(r), 95):.1f}")
        lab = [("img" if i else "") + "%+d" % c for c, i in zip(out["ch"][m], out["image"][m])]
        u_, cnt = np.unique(lab, return_counts=True)
        print("by channel (offset from Ring Alert, img = mirror image):",
              dict(sorted(zip(u_.tolist(), cnt.tolist()), key=lambda kv: -kv[1])))
        if "ira" in out:
            k = m & (out["ira"] > 0)
            print(f"decoded Ring Alerts matched: {k.sum()} of {(out['ira'] > 0).sum()}, on Ring "
                  f"Alert straight: {(k & (out['ch'] == 0) & (out['image'] == 0)).sum()}")
        print(f"satellites heard: {len(np.unique(out['norad'][m]))}")
    print(f"clock: {ck[good, 1].min():.0f} .. {ck[good, 1].max():.0f} Hz")
    if a.out:
        np.savez_compressed(a.out, **out)
        print("->", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
