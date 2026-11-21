#!/usr/bin/env python3
"""A flight replayed on a map: where the u-blox says the board was, against where the board's
own sensors (soop_ins.py: IMU + Iridium Doppler + cabin baro, no GPS after wheels-off) put it.

Usage:
  soop_flightmap.py --flight "LABEL|LOG.npz|EST.csv[|IMU_ONLY.csv]" [--flight ...]
                    --basemap DIR OUT.html

LOG.npz is soop_extract.py's (gps), EST.csv soop_ins.py's --out, IMU_ONLY.csv the same run with
--no-doppler (optional, drawn as the IMU alone). DIR holds Natural Earth's
ne_10m_coastline.geojson and ne_10m_admin_0_boundary_lines_land.geojson (public domain,
github.com/nvkelso/natural-earth-vector); only the lines near the flights are embedded, so the
page needs nothing from the network. One self-contained page, in soop_live.py's colours.
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

PLACES = [  #(name, lat, lon, kind)
    ("STN", 51.885, 0.235, "apt"), ("LGW", 51.148, -0.190, "apt"), ("BRE", 53.0475, 8.787, "apt"),
    ("HAM", 53.630, 9.988, "apt"), ("London", 51.507, -0.128, "city"), ("Amsterdam", 52.370, 4.900, "city"),
    ("Rotterdam", 51.920, 4.480, "city"),
    ("Groningen", 53.220, 6.570, "city"), ("Brussels", 50.850, 4.350, "city"), ("Cambridge", 52.205, 0.119, "city"),
]


def rows(path):
    return [r for r in csv.DictReader(open(path))]


def flight(spec):
    parts = spec.split("|")
    label, npz, est = parts[:3]
    imu = parts[3] if len(parts) > 3 and parts[3] else None
    D = np.load(npz)
    g = D["gps"]
    g = g[(g[:, 1] >= 3) & (np.abs(g[:, 3]) > 1e-6)]
    E = [r for r in rows(est) if r["lat"] and r["err"]]
    t_off = float(E[0]["t"])
    t_end = float(E[-1]["t"])
    #the truth from a few minutes before wheels-off to a little after the estimate ends
    k = (g[:, 0] > t_off - 420) & (g[:, 0] < t_end + 300)
    gk = g[k]
    keep = np.r_[True, np.diff(np.floor(gk[:, 0] / 5.0)) > 0]
    truth = [[round(float(r[0] - t_off), 1), round(float(r[3]), 5), round(float(r[4]), 5), round(float(r[5])), round(float(r[6]), 1)]
             for r in gk[keep]]
    est_pts = [[round(float(r["t"]) - t_off, 1), round(float(r["lat"]), 5), round(float(r["lon"]), 5),
                round(float(r["hacc"])), round(float(r["err"])), int(float(r.get("fused", 0) or 0)),
                round(float(r["h"])) if r.get("h") else None]
               for r in E[::5]]
    imu_pts = []
    if imu:
        I = [r for r in rows(imu) if r["lat"] and r["err"]]
        imu_pts = [[round(float(r["t"]) - t_off, 1), round(float(r["lat"]), 5), round(float(r["lon"]), 5),
                    round(float(r["err"]))] for r in I[::5]]
    return {"label": label, "truth": truth, "est": est_pts, "imu": imu_pts}


def clip_lines(path, box, step_km=0.8):
    la0, la1, lo0, lo1 = box
    out = []
    for f in json.load(open(path))["features"]:
        geom = f["geometry"]
        lines = geom["coordinates"] if geom["type"] == "MultiLineString" else [geom["coordinates"]]
        for line in lines:
            cur, last = [], None
            for lon, lat in line:
                inside = la0 <= lat <= la1 and lo0 <= lon <= lo1
                if inside:
                    if last is None or math.hypot((lat - last[0]) * 111.0, (lon - last[1]) * 111.0 * math.cos(math.radians(lat))) > step_km:
                        cur.append([round(lon, 3), round(lat, 3)])
                        last = (lat, lon)
                elif cur:
                    cur.append([round(lon, 3), round(lat, 3)])
                    if len(cur) > 1:
                        out.append(cur)
                    cur, last = [], None
            if len(cur) > 1:
                out.append(cur)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--flight", action="append", required=True)
    ap.add_argument("--basemap", required=True)
    ap.add_argument("--note", default="")
    ap.add_argument("out")
    a = ap.parse_args()
    F = [flight(s) for s in a.flight]
    lat = [p[1] for f in F for p in f["truth"] + f["est"]]
    lon = [p[2] for f in F for p in f["truth"] + f["est"]]
    box = (min(lat) - 1.5, max(lat) + 1.5, min(lon) - 2.5, max(lon) + 2.5)
    coast = clip_lines(os.path.join(a.basemap, "ne_10m_coastline.geojson"), box)
    border = clip_lines(os.path.join(a.basemap, "ne_10m_admin_0_boundary_lines_land.geojson"), box)
    data = {"flights": F, "coast": coast, "border": border, "note": a.note,
            "places": [p for p in PLACES if box[0] <= p[1] <= box[1] and box[2] <= p[2] <= box[3]]}
    html = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "soop_flightmap.html")).read()
    html = html.replace("/*DATA*/null", json.dumps(data, separators=(",", ":")))
    open(a.out, "w").write(html)
    print(f"{a.out}: {len(F)} flights, {sum(len(c) for c in coast)} coast points, {len(html) / 1e3:.0f} kB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
