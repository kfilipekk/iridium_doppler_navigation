#!/usr/bin/env python3
"""GPS-free flight navigation: the board's IMU, corrected by Iridium Doppler, scored on the u-blox.

Usage:
  soop_ins.py LOG.npz TLE BURSTS.csv TRUTH.npz [--lag S] [--alt-sigma M] [--no-doppler]
              [--sig HZ] [--out FIXES.csv] [--minutes N]

A strapdown INS in local-level (NED) axes - Earth rate and transport rate kept, the IMU
(soop_extract.py's 100 Hz) in whatever attitude the board had in the bag - with an error-state
Kalman filter: position, velocity and attitude errors, gyro and accelerometer biases, and the
receiver clock and its drift (Gauss-Markov). It starts on the take-off roll: roll and pitch from
the specific force before it, heading from the roll itself (the levelled specific force against
the u-blox acceleration), the gyro bias from the quietest stretch before it. The u-blox position
and velocity aid it only until the wheels leave the runway; from there on the only aiding is
Iridium Doppler - each burst associated by the filter's own prediction (Ring Alert and the
channels either side, nearest in NIS, gated, unambiguous) - and, with --alt-sigma, a height
standing in for the aircraft's air-data altitude (the cabin barometer reads the cabin). The
clock is taken at the roll from the truth tool's track (GPS-assisted, before the loss).
--no-doppler is the INS alone, for comparison.
"""
import argparse
import collections
import csv
import math
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import soop_truth as T

OMEGA = 7.292115e-5
A_E, F_E = 6378137.0, 1 / 298.257223563
E2 = F_E * (2 - F_E)
# state: dp(3, NED m) dv(3) phi(3) bg(3) ba(3) clk dclk, cabin-schedule height bias
IP, IV, IA, IBG, IBA, IC, ID, IHB = 0, 3, 6, 9, 12, 15, 16, 17
NB = 18                    #base states; then per tracked satellite: transmit offset beta (Hz)
NSLOT = 14                 #and along-track timing tau (s), old elements put it km off its path
NX = NB + 2 * NSLOT


def radii(lat):
    s = math.sin(lat)
    RN = A_E / math.sqrt(1 - E2 * s * s)
    RM = RN * (1 - E2) / (1 - E2 * s * s)
    return RM, RN


def gravity(lat, h):
    s2 = math.sin(lat) ** 2
    g0 = 9.7803253359 * (1 + 0.00193185265241 * s2) / math.sqrt(1 - E2 * s2)
    return g0 - 3.086e-6 * h


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def C_en(lat, lon):
    """NED -> ECEF."""
    sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    return np.array([[-sl * co, -so, -cl * co], [-sl * so, co, -cl * so], [cl, 0, -sl]])


def rotvec(w):
    a = np.linalg.norm(w)
    if a < 1e-12:
        return np.eye(3) + skew(w)
    k = skew(w / a)
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


def euler_to_C(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = map(lambda f: f, (math.cos(roll), math.sin(roll), math.cos(pitch),
                                               math.sin(pitch), math.cos(yaw), math.sin(yaw)))
    return np.array([[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                     [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                     [-sp, sr * cp, cr * cp]])


class INS:
    def __init__(self, lat, lon, h, v, C, bg, clk, a):
        self.lat, self.lon, self.h = lat, lon, h
        self.v = np.array(v, float)
        self.C = C                       #body -> NED
        self.bg = np.array(bg, float)
        self.ba = np.zeros(3)
        self.clk, self.dclk = clk, 0.0
        P = np.zeros((NX, NX))
        P[IP:IP + 3, IP:IP + 3] = np.eye(3) * 3.0 ** 2
        P[IV:IV + 3, IV:IV + 3] = np.eye(3) * 0.3 ** 2
        P[IA:IA + 3, IA:IA + 3] = np.diag([0.02, 0.02, 0.08]) ** 2
        P[IBG:IBG + 3, IBG:IBG + 3] = np.eye(3) * a.bg0 ** 2
        P[IBA:IBA + 3, IBA:IBA + 3] = np.eye(3) * a.ba0 ** 2
        P[IC, IC] = 60.0 ** 2
        P[ID, ID] = 1.0 ** 2
        P[IHB, IHB] = a.cab_bias_sigma ** 2
        self.P = P
        self.hb = 0.0
        self.a = a
        self.Facc = np.zeros((NX, NX))
        self.tacc = 0.0
        self.q_rot = self.q_jerk = 0.0
        self.qscale = 1.0                  #innovation-based adaptation of the motion noise
        self.slot_sat = [-1] * NSLOT
        self.slot_heard = [-1e9] * NSLOT
        self.x = np.zeros(NX)              #the satellite states live here; the base is applied

    def mech(self, w_b, f_b, dt):
        lat, h = self.lat, self.h
        RM, RN = radii(lat)
        vN, vE, vD = self.v
        w_ie = np.array([OMEGA * math.cos(lat), 0, -OMEGA * math.sin(lat)])
        w_en = np.array([vE / (RN + h), -vN / (RM + h), -vE * math.tan(lat) / (RN + h)])
        w_in = w_ie + w_en
        w = w_b - self.bg
        f = f_b - self.ba
        self.C = self.C @ rotvec(w * dt) - skew(w_in * dt) @ self.C
        u, _, vt = np.linalg.svd(self.C)               #keep it a rotation
        self.C = u @ vt
        fn = self.C @ f
        g = np.array([0, 0, gravity(lat, h)])
        acc = fn + g - np.cross(2 * w_ie + w_en, self.v)
        v_old = self.v.copy()
        self.v = self.v + acc * dt
        vm = 0.5 * (v_old + self.v)
        self.lat += vm[0] / (RM + h) * dt
        self.lon += vm[1] / ((RN + h) * math.cos(lat)) * dt
        self.h -= vm[2] * dt
        #error dynamics, accumulated for a coarser covariance step
        F = np.zeros((NX, NX))
        F[IP:IP + 3, IV:IV + 3] = np.eye(3)
        F[IV:IV + 3, IA:IA + 3] = skew(fn)
        F[IV:IV + 3, IBA:IBA + 3] = -self.C
        F[IA:IA + 3, IA:IA + 3] = -skew(w_in)
        F[IA:IA + 3, IBG:IBG + 3] = self.C
        F[IC, ID] = 1.0
        F[ID, ID] = -1.0 / self.a.clkd_tau
        F[IHB, IHB] = -1.0 / self.a.cab_bias_tau
        self.Facc += F * dt
        self.tacc += dt
        #gyro scale factor and axis misalignment: attitude error grows with the angle turned, so
        #a bag being handled (3.4 rad/s, 2026-11-19) loosens the attitude and smooth cruise does not
        self.q_rot += (self.a.gyro_sf * np.linalg.norm(w) * dt) ** 2
        self.q_jerk += (self.a.acc_sf * max(np.linalg.norm(f) - 9.8, 0.0) * dt) ** 2
        if self.tacc >= 0.1:
            self.cov_step()

    def cov_step(self):
        dt = self.tacc
        if dt <= 0:
            return
        Phi = np.eye(NX) + self.Facc
        Q = np.zeros(NX)
        Q[IV:IV + 3] = self.a.vrw ** 2 * dt * self.qscale
        Q[IA:IA + 3] = (self.a.arw ** 2 * dt + self.q_rot) * self.qscale
        Q[IV:IV + 3] += self.q_jerk
        self.q_rot = self.q_jerk = 0.0
        Q[IBG:IBG + 3] = self.a.bg_rw ** 2 * dt
        Q[IBA:IBA + 3] = self.a.ba_rw ** 2 * dt
        Q[IC] = 0.3 ** 2 * dt
        Q[ID] = self.a.clkd_sigma ** 2 * 2 * dt / self.a.clkd_tau
        Q[IHB] = self.a.cab_bias_sigma ** 2 * 2 * dt / self.a.cab_bias_tau
        for k in range(NSLOT):
            if self.slot_sat[k] >= 0:
                Q[NB + 2 * k] = self.a.beta_rw ** 2 * dt
                Q[NB + 2 * k + 1] = self.a.tau_rw ** 2 * dt
        self.P = Phi @ self.P @ Phi.T + np.diag(Q)
        self.Facc[:] = 0
        self.tacc = 0.0

    def update(self, H, y, R):
        self.cov_step()
        P = self.P
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        dx = K @ y
        IKH = np.eye(NX) - K @ H
        self.P = IKH @ P @ IKH.T + K @ R @ K.T
        RM, RN = radii(self.lat)
        self.lat += dx[0] / (RM + self.h)
        self.lon += dx[1] / ((RN + self.h) * math.cos(self.lat))
        self.h -= dx[2]
        self.v += dx[IV:IV + 3]
        self.C = (np.eye(3) - skew(dx[IA:IA + 3])) @ self.C
        self.bg += dx[IBG:IBG + 3]
        self.ba += dx[IBA:IBA + 3]
        self.clk += dx[IC]
        self.dclk += dx[ID]
        self.hb += dx[IHB]
        self.x[NB:] += dx[NB:]

    def slot(self, sat, t, make):
        if sat in self.slot_sat:
            return self.slot_sat.index(sat)
        if not make:
            return -1
        for k in range(NSLOT):                            #forget the long-unheard
            if self.slot_sat[k] >= 0 and t - self.slot_heard[k] > self.a.drop_s:
                self.free(k)
        k = self.slot_sat.index(-1) if -1 in self.slot_sat else int(np.argmin(self.slot_heard))
        self.free(k)
        self.slot_sat[k] = sat
        self.slot_heard[k] = t
        o = NB + 2 * k
        self.P[o, o] = self.a.beta0 ** 2
        self.P[o + 1, o + 1] = self.a.tau0 ** 2
        return k

    def free(self, k):
        o = NB + 2 * k
        self.P[o:o + 2, :] = 0
        self.P[:, o:o + 2] = 0
        self.x[o:o + 2] = 0
        self.slot_sat[k] = -1

    def nis(self, H, y, R):
        self.cov_step()
        S = H @ self.P @ H.T + R
        return float(y @ np.linalg.solve(S, y)) if np.ndim(S) else float(y * y / S)

    def ecef(self):
        return T.geo_ecef(np.array([math.degrees(self.lat)]), np.array([math.degrees(self.lon)]),
                          np.array([self.h]))[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("npz"); ap.add_argument("tle"); ap.add_argument("bursts"); ap.add_argument("truth")
    ap.add_argument("--lag", type=float, default=-0.02)
    ap.add_argument("--alt-sigma", type=float, default=0.0)
    ap.add_argument("--alt-bias", type=float, default=0.0, help="m, added to the stand-in altitude")
    ap.add_argument("--cabin-baro", help="another flight's .npz to fit the cabin pressurisation schedule on")
    ap.add_argument("--cabin-min", type=float, default=600.0, help="m of cabin altitude below which it says nothing")
    ap.add_argument("--cabin-floor", action="store_true", help="below it, bound the aircraft between the ground and there")
    ap.add_argument("--cab-bias-sigma", type=float, default=700.0, help="the schedule's height offset, m (a state)")
    ap.add_argument("--cabin-rate", type=float, default=0.0, help="m/s of cabin descent below which its rate says nothing (0 = off)")
    ap.add_argument("--cab-bias-tau", type=float, default=1800.0)
    ap.add_argument("--cab-white", type=float, default=100.0, help="m, the reading's own noise")
    ap.add_argument("--vd-bound", type=float, default=0.0, help="m/s: an airliner's vertical speed, as a weak constraint (0 = off)")
    ap.add_argument("--zupt", action="store_true", help="zero velocity whenever the IMU is still (after landing)")
    ap.add_argument("--zupt-speed", type=float, default=15.0, help="m/s: only while the INS's own speed is under this")
    ap.add_argument("--adapt", action="store_true", help="innovation-based adaptive motion noise (use with --widen 1)")
    ap.add_argument("--adapt-n", type=int, default=60, help="bursts the NIS is averaged over")
    ap.add_argument("--adapt-target", type=float, default=1.0)
    ap.add_argument("--adapt-max", type=float, default=100.0)
    ap.add_argument("--no-doppler", action="store_true")
    ap.add_argument("--sig", type=float, default=20.0, help="Doppler noise floor, Hz")
    ap.add_argument("--gate", type=float, default=5.0)
    ap.add_argument("--amb", type=float, default=4.0)
    ap.add_argument("--minutes", type=float, default=0.0, help="stop this long after the loss")
    ap.add_argument("--out")
    ap.add_argument("--vrw", type=float, default=0.02)       #m/s/sqrt(s)
    ap.add_argument("--arw", type=float, default=3e-4)       #rad/sqrt(s)
    ap.add_argument("--bg-rw", type=float, default=2e-5)     #rad/s/sqrt(s)
    ap.add_argument("--ba-rw", type=float, default=2e-4)     #m/s^2/sqrt(s)
    ap.add_argument("--bg0", type=float, default=0.004)
    ap.add_argument("--gyro-sf", type=float, default=0.02, help="attitude error per radian turned")
    ap.add_argument("--gyro-temp", action="store_true", help="feed the gyro bias forward with temperature")
    ap.add_argument("--widen", type=float, default=2.0, help="covariance factor when most bursts fail the gate (1 = off)")
    ap.add_argument("--chans", default="-1,0,1", help="channels matched, offsets from Ring Alert")
    ap.add_argument("--both-imus", action="store_true", help="average IMU 0 and IMU 1")
    ap.add_argument("--cap-pos", type=float, default=1e12, help="widening stops at this sigma, m (off: a 5 km cap kept the outbound descent from recovering)")
    ap.add_argument("--cap-vel", type=float, default=1e12, help="m/s")
    ap.add_argument("--cap-att", type=float, default=1e12, help="rad")
    ap.add_argument("--acc-sf", type=float, default=0.05)
    ap.add_argument("--ba0", type=float, default=0.15)
    ap.add_argument("--clkd-sigma", type=float, default=1.0)
    ap.add_argument("--clkd-tau", type=float, default=600.0)
    ap.add_argument("--beta0", type=float, default=25.0, help="satellite transmit offset prior, Hz")
    ap.add_argument("--tau0", type=float, default=0.6, help="along-track timing prior, s (~4 km)")
    ap.add_argument("--beta-rw", type=float, default=0.02)
    ap.add_argument("--tau-rw", type=float, default=0.0005)
    ap.add_argument("--drop-s", type=float, default=600.0)
    ap.add_argument("--clk-temp", type=float, default=25.6,
                    help="TCXO Hz/degC (Castle Mound 2026-11-17): a weak clock pseudo-measurement from the"
                         " IMU temperature after the loss, anchored on the clock at the loss; 0 = off")
    ap.add_argument("--clk-temp-sigma", type=float, default=60.0)
    a = ap.parse_args()

    D = np.load(a.npz)
    imu, gps = D["imu"], D["gps"]
    if a.both_imus and "imu1" in D:
        #the ICM-42605 beside the ICM-45686, on the same axes: their mean halves the noise they
        #do not share (biases they share, like a boot calibration done moving, stay)
        b = D["imu1"]
        imu = imu.copy()
        for j in range(1, 8):
            imu[:, j] = 0.5 * (imu[:, j] + np.interp(imu[:, 0], b[:, 0], b[:, j]))
    to_utc, _, _ = T.time_map(gps, a.lag)
    good = (gps[:, 1] >= 3) & (gps[:, 2] >= 6) & (gps[:, 9] < 5.0)
    G = gps[good]
    te = G[:, 0] - a.lag
    spd = G[:, 6]
    #the take-off roll: the straight run up to 70 m/s, and wheels-off 30 m above the runway
    i70 = int(np.argmax(spd > 70))
    rwy = G[i70, 7]
    i0 = i70
    while i0 > 0 and abs(((G[i0 - 1, 7] - rwy + 180) % 360) - 180) < 4 and G[i0 - 1, 6] > 1.0:
        i0 -= 1
    ioff = i70 + int(np.argmax(G[i70:, 5] > G[i0, 5] + 30))
    t0, t_off = te[i0], te[ioff]
    #start still: the last 5 s before the roll with the u-blox under 0.3 m/s and the gyro quiet -
    #roll and pitch from its specific force, the gyro bias from its mean - then carry that
    #attitude on the gyro (heading unknown) to the roll, where the levelled specific force
    #against the u-blox acceleration gives the heading; the INS starts at the still moment
    best, ts = None, None
    for s_ in np.arange(t0 - 5, t0 - 1800, -1.0):
        kk = (imu[:, 0] > s_ - 5) & (imu[:, 0] <= s_)
        kg = (te > s_ - 5) & (te <= s_)
        if kk.sum() < 400 or not kg.any() or G[kg, 6].max() > 0.3:
            continue
        sd = imu[kk, 1:4].std(0).max()
        if sd < 0.01:
            best, ts = kk, s_
            break
    if ts is None:
        raise SystemExit("no still moment before the roll")
    fm = imu[best, 4:7].mean(0)
    bg = imu[best, 1:4].mean(0)
    #the gyro bias against temperature, from every still 20 s before the start (GPS still
    #there): MEMS bias walks ~0.002-0.006 deg/s/degC, and the board warms 20-30 degC in a bag
    kT = np.zeros(3)
    T0 = float(imu[best, 7].mean())
    if a.gyro_temp:
        rows_ = []
        for s_ in np.arange(imu[0, 0] + 30, ts, 20.0):
            kk_ = (imu[:, 0] > s_) & (imu[:, 0] <= s_ + 20)
            kg_ = (te > s_) & (te <= s_ + 20)
            if kk_.sum() < 1000 or not kg_.any() or G[kg_, 6].max() > 0.3:
                continue
            if imu[kk_, 1:4].std(0).max() < 0.01:
                rows_.append((*imu[kk_, 1:4].mean(0), imu[kk_, 7].mean()))
        rows_ = np.array(rows_)
        if len(rows_) >= 6 and np.ptp(rows_[:, 3]) > 3.0:
            A_ = np.column_stack([rows_[:, 3] - T0, np.ones(len(rows_))])
            for ax in range(3):
                kT[ax] = np.linalg.lstsq(A_, rows_[:, ax], rcond=None)[0][0]
        print(f"gyro bias temperature slope {np.round(np.degrees(kT), 4)} deg/s/degC from {len(rows_)} still "
              f"windows over {np.ptp(rows_[:, 3]) if len(rows_) else 0:.1f} degC")
    roll = math.atan2(-fm[1], -fm[2])
    pitch = math.atan2(fm[0], math.hypot(fm[1], fm[2]))
    C = euler_to_C(roll, pitch, 0.0)
    kk = (imu[:, 0] > ts) & (imu[:, 0] <= te[i70])
    seg = imu[kk]
    f_acc, n_acc = np.zeros(3), 0
    for j in range(1, len(seg)):
        dtj = seg[j, 0] - seg[j - 1, 0]
        if 0 < dtj < 0.5:
            C = C @ rotvec((seg[j, 1:4] - bg) * dtj)
        if seg[j, 0] >= t0:
            f_acc += C @ seg[j, 4:7]; n_acc += 1
    f_lv = (f_acc / max(n_acc, 1))[:2]
    vn = G[:, 6] * np.cos(np.radians(G[:, 7])); ve = G[:, 6] * np.sin(np.radians(G[:, 7]))
    a_ned = np.array([(vn[i70] - vn[i0]) / (te[i70] - te[i0]), (ve[i70] - ve[i0]) / (te[i70] - te[i0])])
    yaw = math.atan2(a_ned[1], a_ned[0]) - math.atan2(f_lv[1], f_lv[0])
    print(f"still at {ts:.1f} ({t0 - ts:.0f} s before the roll); roll from {te[i0]:.1f} (runway {rwy:.0f}), "
          f"wheels off {t_off:.1f}; gyro bias {np.round(bg, 4)}; attitude r {math.degrees(roll):.0f} "
          f"p {math.degrees(pitch):.0f}, heading offset {math.degrees(yaw):.0f} deg; roll accel "
          f"{np.linalg.norm(a_ned):.2f} m/s2, levelled f {np.linalg.norm(f_lv):.2f}")
    #the cabin barometer, through the pressurisation schedule: fitted on ANOTHER flight (its
    #u-blox height against its cabin), so nothing of this flight's truth goes in. Above ~600 m
    #cabin the schedule is near linear (2026-11-18/19: 737 and A320 alike); below it the cabin
    #sits at the field while the aircraft is still kilometres up, so it says nothing there
    cab_map = None
    if a.cabin_baro:
        Dx = np.load(a.cabin_baro)
        bx, gx = Dx["baro"], Dx["gps"]
        okx = gx[:, 1] >= 3
        hx = np.interp(bx[:, 0], gx[okx, 0], gx[okx, 5])
        cx = 44330.8 * (1 - (bx[:, 2] / 101325.0) ** 0.190263)
        kx = (cx > a.cabin_min) & (bx[:, 0] > gx[okx, 0][0]) & (bx[:, 0] < gx[okx, 0][-1])
        A_ = np.column_stack([cx[kx], np.ones(kx.sum())])
        coef = np.linalg.lstsq(A_, hx[kx], rcond=None)[0]
        sd = float(np.std(hx[kx] - A_ @ coef))
        cab_map = (coef[0], coef[1], max(sd * 1.5, 300.0))
        print(f"cabin baro: aircraft height = {coef[0]:.2f} x cabin + {coef[1]:.0f} m, fitted on another "
              f"flight (sd {sd:.0f} m), used at sigma {cab_map[2]:.0f} m above {a.cabin_min:.0f} m cabin")
    baro = D["baro"]
    cab_alt = 44330.8 * (1 - (baro[:, 2] / 101325.0) ** 0.190263)

    TR = np.load(a.truth)
    ck = TR["clock_track"]
    okc = ck[:, 2] >= 2 * np.maximum(ck[:, 3], 1)
    clk0 = float(np.interp(ts, ck[okc, 0], ck[okc, 1]))

    iS = int(np.argmin(np.abs(te - ts)))
    ins = INS(math.radians(G[iS, 3]), math.radians(G[iS, 4]), G[iS, 5] + 45.0, np.zeros(3),
              euler_to_C(0, 0, yaw) @ euler_to_C(roll, pitch, 0.0), bg, clk0, a)
    t0, i0 = ts, iS
    bg_ff = lambda temp: kT * (temp - T0)          #added to the measured rate's bias

    #bursts after the roll starts, with every satellite's state at each
    rows = list(csv.DictReader(open(a.bursts)))
    B = np.array([[float(r["t"]), float(r["f_hz"]), float(r["f_sigma_hz"])] for r in rows])
    B = B[np.argsort(B[:, 0])]
    t_end = imu[-1, 0] if a.minutes <= 0 else t_off + 60 * a.minutes
    B = B[(B[:, 0] > t0) & (B[:, 0] < t_end)]
    sats, norad = T.load_tles(a.tle)
    SR, SV = T.sat_ecef(sats, to_utc(B[:, 0]))
    SR2, SV2 = T.sat_ecef(sats, to_utc(B[:, 0]) + 0.5)
    #each satellite's Doppler rate comes from the filter's own position, burst by burst
    chans = tuple(int(c) for c in a.chans.split(","))

    #events: GPS (1 Hz, until wheels-off), height (1 Hz), bursts
    ev = []
    for i in range(i0, len(G)):
        if te[i] >= t_end:
            break
        if i % 5 == 0 and te[i] <= t_off:
            ev.append((te[i], 0, i))
        elif i % 5 == 0 and a.alt_sigma > 0:
            ev.append((te[i], 1, i))
        elif i % 5 == 0 and cab_map is not None:
            ev.append((te[i], 3, i))
        if i % 5 == 0 and te[i] > t_off and a.vd_bound > 0:
            ev.append((te[i] + 1e-3, 4, i))
    for j in range(len(B)):
        ev.append((B[j, 0], 2, j))
    ev.sort()
    ims = imu[(imu[:, 0] > t0) & (imu[:, 0] <= t_end)]
    out, ei, n_f, n_amb, n_none, last_t = [], 0, 0, 0, 0, t0
    next_fix = t_off
    stats = dict(fused=0, gated=0, amb=0)
    anchor, next_tc = None, t_off
    wbuf, fbuf, last_z = collections.deque(maxlen=400), collections.deque(maxlen=400), -1e9
    cab_high = [False]
    nis_hist, next_ad, qs_log = collections.deque(maxlen=a.adapt_n), t_off, []
    recent, last_widen = collections.deque(maxlen=60), -1e9
    assoc = []
    for ri, r in enumerate(ims):
        dt = r[0] - last_t
        last_t = r[0]
        if dt <= 0 or dt > 0.5:
            continue
        ins.mech(r[1:4] - bg_ff(r[7]), r[4:7], dt)
        #the IMU still (the board's own detector, AP_SoOP.cpp: gyro and |f| steady): the vehicle
        #is parked - after landing, at the gate - and its velocity is zero
        if a.zupt and r[0] > t_off:
            wbuf.append(float(np.linalg.norm(r[1:4]))); fbuf.append(float(np.linalg.norm(r[4:7])))
            #an IMU alone cannot tell a parked aircraft from smooth cruise (3,086 false zero-
            #velocity updates at 240 m/s): only where the INS itself says it is slow
            if len(wbuf) == wbuf.maxlen and r[0] - last_z > 0.5 and max(wbuf) < 0.03 \
                    and math.hypot(ins.v[0], ins.v[1]) < a.zupt_speed \
                    and abs(np.mean(fbuf) - 9.80665) < 0.3 and np.std(fbuf) < 0.05:
                H = np.zeros((3, NX)); H[:, IV:IV + 3] = np.eye(3)
                ins.update(H, -ins.v.copy(), np.eye(3) * 0.05 ** 2)
                last_z = r[0]
                stats["zupt"] = stats.get("zupt", 0) + 1
        while ei < len(ev) and ev[ei][0] <= r[0]:
            t, kind, i = ev[ei]; ei += 1
            if kind == 0:                                 #u-blox position + velocity
                RM, RN = radii(ins.lat)
                dp = np.array([(math.radians(G[i, 3]) - ins.lat) * (RM + ins.h),
                               (math.radians(G[i, 4]) - ins.lon) * (RN + ins.h) * math.cos(ins.lat),
                               -(G[i, 5] + 45.0 - ins.h)])
                dv = np.array([vn[i], ve[i], G[i, 8]]) - ins.v
                H = np.zeros((6, NX)); H[0:3, IP:IP + 3] = np.eye(3); H[3:6, IV:IV + 3] = np.eye(3)
                ins.update(H, np.r_[dp, dv], np.diag([3.0 ** 2] * 3 + [0.2 ** 2] * 3))
            elif kind == 3:                              #cabin barometer, through the schedule
                c_ = float(np.interp(t, baro[:, 0], cab_alt))
                H = np.zeros((1, NX)); H[0, IP + 2] = 1.0
                if c_ > a.cabin_min:
                    cab_high[0] = True
                    #the schedule's error is one offset for minutes on end (a 737 is not an
                    #A320), so it is a state - as white noise, a minute of it claimed +-90 m
                    #while 2-4 km out: z = h + bias + noise
                    H[0, IHB] = -1.0
                    y = np.array([-(cab_map[0] * c_ + cab_map[1] + 45.0 - ins.h) + ins.hb])
                    ins.update(H, y, np.array([[a.cab_white ** 2]]))
                elif a.cabin_rate > 0 and cab_high[0]:
                    #only on the way down: just after take-off the cabin dips as pressurisation
                    #starts (A320, 2026-11-19), which would read as the aircraft descending
                    #below the schedule's linear part the cabin may still be descending with the
                    #aircraft (737: 650 -> 210 m while it came down 2.7 km) or sit at the field
                    #(A320): its rate, when it moves, gives the vertical speed at ~1/slope
                    kk_ = (baro[:, 0] > t - 20) & (baro[:, 0] <= t)
                    if kk_.sum() > 20:
                        rc = np.polyfit(baro[kk_, 0] - t, cab_alt[kk_], 1)[0]   #m/s, + up
                        if rc < -a.cabin_rate:
                            Hv = np.zeros((1, NX)); Hv[0, IV + 2] = 1.0
                            vd_meas = -cab_map[0] * rc                    #down positive
                            ins.update(Hv, np.array([vd_meas - ins.v[2]]),
                                       np.array([[(0.5 * vd_meas) ** 2 + 2.0 ** 2]]))
                elif a.cabin_floor:
                    #the cabin down at the field: the aircraft is somewhere below where the
                    #schedule leaves it, no more - a bound, which an unstable vertical channel needs
                    top = cab_map[0] * a.cabin_min + cab_map[1]
                    y = np.array([-(0.5 * top + 45.0 - ins.h)])
                    ins.update(H, y, np.array([[(0.5 * top) ** 2]]))
            elif kind == 4:                              #the airliner envelope: |vd| well under 15 m/s
                H = np.zeros((1, NX)); H[0, IV + 2] = 1.0
                ins.update(H, np.array([-ins.v[2]]), np.array([[a.vd_bound ** 2]]))
            elif kind == 1:
                H = np.zeros((1, NX)); H[0, IP + 2] = 1.0
                #the u-blox height standing in for air data: noise, and the bias a pressure
                #altitude carries (the weather: QNH against the ISA, up to ~200 m)
                y = np.array([-(G[i, 5] + 45.0 + a.alt_bias + np.random.default_rng(i).normal(0, a.alt_sigma) - ins.h)])
                ins.update(H, y, np.array([[a.alt_sigma ** 2]]))
            elif not a.no_doppler:
                rr = ins.ecef()
                Cn = C_en(ins.lat, ins.lon)
                vr = Cn @ ins.v
                d = SR[:, i] - rr
                rho = np.linalg.norm(d, axis=1)
                u = d / rho[:, None]
                up = rr / np.linalg.norm(rr)
                el = np.degrees(np.arcsin(u @ up))
                dvv = SV[:, i] - vr
                rdot = np.sum(dvv * u, 1)
                R = np.array([[max(B[i, 2], 1.0) ** 2 + a.sig ** 2]])
                d2 = SR2[:, i] - rr
                u2 = d2 / np.linalg.norm(d2, axis=1)[:, None]
                rate_i = -T.F_RA / T.C * (np.sum((SV2[:, i] - vr) * u2, 1) - rdot) / 0.5
                cands = []
                for s in np.where((el > -3) & np.isfinite(rho))[0]:
                    gr = (dvv[s] - rdot[s] * u[s]) / rho[s]
                    k = ins.slot(s, B[i, 0], False)
                    for ch in chans:
                        fch = T.f_ch(ch)
                        pred = ch * T.SPACING - T.LO_OFFSET - fch / T.C * rdot[s] + ins.clk
                        H = np.zeros((1, NX))
                        H[0, IP:IP + 3] = (fch / T.C) * (gr @ Cn)
                        H[0, IV:IV + 3] = (fch / T.C) * (u[s] @ Cn)
                        H[0, IC] = 1.0
                        Rc = R.copy()
                        if k >= 0:
                            o = NB + 2 * k
                            pred += ins.x[o] + rate_i[s] * ins.x[o + 1]
                            H[0, o], H[0, o + 1] = 1.0, rate_i[s]
                        else:                             #not yet tracked: its priors as noise
                            Rc = Rc + a.beta0 ** 2 + (rate_i[s] * a.tau0) ** 2
                        yv = B[i, 1] - pred
                        cands.append((ins.nis(H, np.array([yv]), Rc), H, yv, s, k, Rc, ch))
                if not cands:
                    continue
                cands.sort(key=lambda c: c[0])
                nis_hist.append(min(cands[0][0], a.gate ** 2))
                if cands[0][0] > a.gate ** 2:
                    stats["gated"] += 1
                    recent.append(0)
                    #most bursts failing the gate means the filter is wrong about itself, not the
                    #sky: widen it so the bursts that would correct it get through again
                    if len(recent) >= 30 and sum(list(recent)[-30:]) < 9 and B[i, 0] - last_widen > 10:
                        #by a factor, but never past a ceiling: compounded, widening alone
                        #once let the outbound filter believe itself anywhere within 1,500 km
                        caps = ((slice(IP, IP + 3), a.cap_pos), (slice(IV, IV + 3), a.cap_vel),
                                (slice(IA, IA + 3), a.cap_att))
                        for blk, cap in caps:
                            sd = math.sqrt(max(np.diag(ins.P)[blk].max(), 1e-12))
                            f = min(a.widen, cap / sd) if sd < cap else 1.0
                            if f > 1.0:
                                g_ = math.sqrt(f)
                                ins.P[blk, :] *= g_; ins.P[:, blk] *= g_
                        last_widen = B[i, 0]
                        stats["widened"] = stats.get("widened", 0) + 1
                    continue
                if len(cands) > 1 and cands[1][0] < a.amb * max(cands[0][0], 1.0):
                    stats["amb"] += 1
                    continue
                _, H, yv, s_, k, Rc, ch_ = cands[0]
                if k < 0:                                 #a new satellite: give it its states
                    k = ins.slot(s_, B[i, 0], True)
                    o = NB + 2 * k
                    H[0, o], H[0, o + 1] = 1.0, rate_i[s_]
                    Rc = R
                ins.slot_heard[k] = B[i, 0]
                ins.update(H, np.array([yv]), Rc)
                stats["fused"] += 1
                recent.append(1)
                assoc.append((B[i, 0], int(norad[s_]), ch_))
        if r[0] >= t_off and anchor is None:
            anchor = (ins.clk, float(r[7]))
        if anchor is not None and a.clk_temp > 0 and r[0] >= next_tc:
            next_tc += 1.0
            H = np.zeros((1, NX)); H[0, IC] = 1.0
            Tsm = float(np.mean(ims[max(0, ri - 200):ri + 1, 7]))          #the last 2 s
            pred = anchor[0] + a.clk_temp * (Tsm - anchor[1])
            ins.update(H, np.array([pred - ins.clk]), np.array([[a.clk_temp_sigma ** 2]]))
        if a.adapt and r[0] >= next_ad and len(nis_hist) >= 20:
            #NIS averages 1 when the filter's covariance is right; run consistently above it
            #and the motion model is too sure of itself, so its noise rises in proportion
            next_ad = r[0] + 1.0
            ins.qscale = float(np.clip(np.mean(nis_hist) / a.adapt_target, 1.0, a.adapt_max))
            qs_log.append(ins.qscale)
        if r[0] >= next_fix:
            next_fix += 1.0
            P = ins.P
            out.append((r[0], math.degrees(ins.lat), math.degrees(ins.lon), ins.h,
                        math.sqrt(P[0, 0] + P[1, 1]), ins.clk, ins.v[0], ins.v[1], ins.v[2],
                        stats["fused"], float(np.linalg.norm(r[1:4])), math.sqrt(P[2, 2])))
    print("bursts after wheels-off: fused %d, gated %d, ambiguous %d, widened %d times, zero-velocity %d" % (
        stats["fused"], stats["gated"], stats["amb"], stats.get("widened", 0), stats.get("zupt", 0)))
    if qs_log:
        q_ = np.array(qs_log)
        print("adaptive motion noise: median x%.1f, 95%% x%.1f, max x%.1f" % (np.median(q_), np.percentile(q_, 95), q_.max()))
    o = np.array(out)
    rr, vr, lat, lon, alt, sp = T.truth_at(gps, a.lag, o[:, 0])
    ok = np.isfinite(rr[:, 0])
    dn = np.radians(o[:, 1] - lat) * 6371000.0
    de = np.radians(o[:, 2] - lon) * 6371000.0 * np.cos(np.radians(lat))
    err = np.hypot(dn, de)
    mins = (o[:, 0] - t_off) / 60.0
    for lo, hi in ((0, 5), (5, 15), (15, 30), (30, 45), (45, 60), (60, 90), (90, 200)):
        m = ok & (mins >= lo) & (mins < hi)
        if m.sum() > 10:
            print("  %3d-%3d min after the loss: median %8.0f m  p95 %8.0f m  stated 1-sigma %7.0f m"
                  "  (speed %3.0f m/s)" % (lo, hi, np.median(err[m]), np.percentile(err[m], 95),
                                           np.median(o[m, 4]), np.median(sp[m])))
    if a.out:
        np.save(a.out.rsplit(".", 1)[0] + "_assoc.npy", np.array(assoc, float))
        with open(a.out, "w") as f:
            f.write("t,min_after_loss,lat,lon,err,hacc,clk,vn,ve,vd,fused,gyro,h,h_err,vacc\n")
            for j, (row, e, mm, k_) in enumerate(zip(o, err, mins, ok)):
                f.write("%.2f,%.3f,%.7f,%.7f,%s,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%.4f,%.1f,%s,%.1f\n" % (
                        row[0], mm, row[1], row[2], "%.1f" % e if k_ else "", row[4], row[5],
                        row[6], row[7], row[8], row[9], row[10], row[3],
                        "%.1f" % (row[3] - alt[j] - 45.0) if k_ else "", row[11]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
