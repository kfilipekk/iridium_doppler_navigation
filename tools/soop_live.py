#!/usr/bin/env python3
"""A live view of a soop_session.py recording, in the browser: Iridium against the u-blox.

Usage:
  soop_live.py SESSION.csv [--ref LAT LON] [--port 8765]

then open http://localhost:8765. It reads the session file the recorder is writing, so it
never touches the board's serial port and can run beside a test.
"""
import argparse
import http.server
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import soop_watch as w

KEEP_S = 3600          #seconds of track the page gets
F_RA = 1626270833.0    #Ring Alert, Hz
TLE_DEFAULT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "sitl", "tle", "iridium-next.tle")


class Sky:
    """Every Iridium satellite as seen from the reference: now, and the passes ahead.
    Needs sgp4 and numpy; without them the page simply has no sky."""

    def __init__(self, tle_path):
        from sgp4.api import Satrec, SatrecArray
        import numpy as np
        self.np = np
        lines = open(tle_path).read().splitlines()
        self.names, sats = [], []
        for i, ln in enumerate(lines):
            if ln.startswith("1 ") and i + 1 < len(lines) and lines[i + 1].startswith("2 "):
                self.names.append(lines[i - 1].strip().replace("IRIDIUM ", ""))
                sats.append(Satrec.twoline2rv(ln, lines[i + 1]))
        self.arr = SatrecArray(sats)
        self._passes, self._passes_at, self._ref = [], -1e9, None

    def look(self, ref, utc_s):
        """az, el (deg) and Doppler at Ring Alert (Hz) for each satellite at each UTC second
        of today; arrays shaped (sats, times)."""
        np = self.np
        from sgp4.api import jday
        import datetime
        day = datetime.datetime.now(datetime.timezone.utc).replace(hour=0, minute=0, second=0,
                                                                   microsecond=0)
        jd0, fr0 = jday(day.year, day.month, day.day, 0, 0, 0)
        t = np.atleast_1d(np.asarray(utc_s, float))
        jd, fr = np.full(t.size, jd0), fr0 + t / 86400.0
        e, r, v = self.arr.sgp4(jd, fr)
        T = jd + fr - 2451545.0
        g = np.radians((280.46061837 + 360.98564736629 * T) % 360)
        c, s, wE = np.cos(g), np.sin(g), 7.292115e-5
        x = c * r[..., 0] + s * r[..., 1]
        y = -s * r[..., 0] + c * r[..., 1]
        z = r[..., 2]
        vx = c * v[..., 0] + s * v[..., 1] + wE * y
        vy = -s * v[..., 0] + c * v[..., 1] - wE * x
        la, lo = np.radians(ref[0]), np.radians(ref[1])
        a, e2 = 6378.137, 6.69437999014e-3
        N = a / np.sqrt(1 - e2 * np.sin(la) ** 2)
        site = np.array([N * np.cos(la) * np.cos(lo), N * np.cos(la) * np.sin(lo),
                         N * (1 - e2) * np.sin(la)])
        R = np.stack([x, y, z], -1) - site
        V = np.stack([vx, vy, v[..., 2]], -1)
        east = np.array([-np.sin(lo), np.cos(lo), 0.0])
        north = np.array([-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)])
        up = np.array([np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)])
        rng = np.linalg.norm(R, axis=-1)
        el = np.degrees(np.arcsin((R @ up) / rng))
        az = np.degrees(np.arctan2(R @ east, R @ north)) % 360
        dop = -np.sum(R * V, -1) / rng / 299792.458 * F_RA
        el[e != 0] = -90.0
        return az, el, dop

    def now(self, ref, utc_s):
        #one second apart, for the Doppler rate
        az, el, dop = self.look(ref, [utc_s, utc_s + 1.0])
        out = []
        for i, n in enumerate(self.names):
            if el[i, 0] > 0:
                h = 780.0                                    #km, Iridium's altitude
                re = 6371.0
                e = math.radians(el[i, 0])
                rng = -re * math.sin(e) + math.sqrt((re * math.sin(e)) ** 2 + h * h + 2 * re * h)
                out.append(dict(name=n, az=round(float(az[i, 0]), 1), el=round(float(el[i, 0]), 1),
                                dop=round(float(dop[i, 0])),
                                rate=round(float(dop[i, 1] - dop[i, 0]), 1), rng=round(rng)))
        return out

    def doppler(self, ref, utc_s, span=600.0, step=15.0):
        """Predicted Doppler of each satellite up within +-span of now: [{name, pts:[[t, Hz]]}]."""
        np = self.np
        t = np.arange(utc_s - span, utc_s + span + step, step)
        _, el, dop = self.look(ref, t)
        out = []
        for i, n in enumerate(self.names):
            up = el[i] > 0
            if up.any():
                out.append(dict(name=n, pts=[[float(t[k]), round(float(dop[i, k]))]
                                             for k in range(t.size) if up[k]],
                                hi=bool((el[i] >= 10).any())))
        return out

    def passes(self, ref, utc_s, ahead_s=5400, step=20.0):
        """Passes above 10 deg in the next ahead_s, recomputed once a minute."""
        if abs(utc_s - self._passes_at) < 60 and ref == self._ref:
            return self._passes
        np = self.np
        t = np.arange(utc_s - 600, utc_s + ahead_s, step)
        _, el, _ = self.look(ref, t)
        out = []
        for i, n in enumerate(self.names):
            up = el[i] >= 10
            d = np.diff(np.r_[0, up.astype(int), 0])
            for s0, s1 in zip(np.where(d == 1)[0], np.where(d == -1)[0]):
                k = s0 + int(np.argmax(el[i, s0:s1]))
                if t[s1 - 1] >= utc_s:
                    out.append(dict(name=n, rise=float(t[s0]), set=float(t[s1 - 1]),
                                    peak=float(t[k]), max_el=round(float(el[i, k]), 1)))
        out.sort(key=lambda p: p["rise"])
        self._passes, self._passes_at, self._ref = out, utc_s, ref
        return out


def ne_m(lat, lon, ref):
    return (math.radians(lat - ref[0]) * 6371000.0,
            math.radians(lon - ref[1]) * 6371000.0 * math.cos(math.radians(ref[0])))


def burst_waterfall(paths, sky, ref):
    """Every burst in the board's dataflash logs as (UTC s, offset from Ring Alert Hz, C/N0,
    result), with the Doppler predicted over the same span to lay over them."""
    import soop_fieldlog
    import numpy as np
    pts, t_lo, t_hi = [], 1e9, -1e9
    for p in paths:
        log = soop_fieldlog.board_log(p)
        if not log["utc_known"]:
            continue
        for b in log["sob"]:
            #the DSP reports baseband; Ring Alert sits at -LO_OFFSET (62.5 kHz) below the LO
            pts.append([b["t"], round(b["f"] + 62500.0), round(b["cn0"], 1), b["res"]])
            t_lo, t_hi = min(t_lo, b["t"]), max(t_hi, b["t"])
    curves = []
    if sky and pts:
        t = np.arange(t_lo - 60, t_hi + 60, 5.0)
        _, el, dop = sky.look(ref, t)
        for i, n in enumerate(sky.names):
            up = el[i] > 0
            if up.any():
                curves.append(dict(name=n, pts=[[float(t[k]), round(float(dop[i, k]))]
                                                for k in range(t.size) if up[k]]))
    return dict(bursts=pts, curves=curves, logs=[os.path.basename(p) for p in paths])


def snapshot(path, ref_arg, sky=None):
    marks, g1, g2 = w.read(path)
    if not g2:
        return dict(ok=False, why="waiting for the recorder")
    end_t = w.secs(g2[-1][0])
    win = w.windows(marks, end_t)
    first_den = next((s for s, e, d in win if d), end_t)
    if ref_arg:
        ref, how = ref_arg, "given"
    else:
        good = [r for r in g1 if r[1] >= 3 and w.secs(r[0]) < first_den]
        if not good:
            return dict(ok=False, why="no anchored u-blox fix yet")
        ref = (sum(r[2] for r in good) / len(good), sum(r[3] for r in good) / len(good))
        how = f"mean of {len(good)} u-blox fixes before the first denial"
    denied = lambda t: any(d and s <= t <= e for s, e, d in win)

    track = []
    for r in g2:
        t = w.secs(r[0])
        if t < end_t - KEEP_S or r[2] == 0.0:
            continue
        n, e = ne_m(r[2], r[3], ref)
        track.append([round(t, 1), round(n, 1), round(e, 1), r[4], r[5], r[1], denied(t)])
    ublox = []
    for r in g1:
        t = w.secs(r[0])
        if t >= end_t - KEEP_S and r[1] >= 3:
            n, e = ne_m(r[2], r[3], ref)
            ublox.append([round(t, 1), round(n, 2), round(e, 2)])
    stats = []
    for s, e, d in win:
        rows = [x for x in track if s <= x[0] <= e]
        errs = [math.hypot(x[1], x[2]) for x in rows]
        if errs:
            stats.append(dict(start=s, end=e, denied=d, median=w.pct(errs, .5),
                              p95=w.pct(errs, .95), max=max(errs),
                              inside=sum(1 for x, er in zip(rows, errs) if er <= 2 * x[3]) / len(rows)))
    log = [[t, m] for t, m in marks if "could not get home" not in m][-40:]
    l1 = g1[-1] if g1 else None
    return dict(ok=True, ref=ref, ref_how=how, now=end_t, file=os.path.basename(path),
                denied=bool(win and win[-1][2]), phase_start=win[-1][0] if win else end_t,
                ublox_now=dict(fix=l1[1], sats=l1[5], hacc=l1[4]) if l1 else None,
                track=track, ublox=ublox[-600:], windows=stats, log=log,
                sky=sky.now(tuple(ref), end_t) if sky else None,
                passes=sky.passes(tuple(ref), end_t) if sky else None,
                doppler=sky.doppler(tuple(ref), end_t) if sky else None)


PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SoOP Live</title>
<style>
:root{--bg:#1e1e1e;--side:#252526;--panel:#1e1e1e;--line:#3c3c3c;--fg:#cccccc;--dim:#858585;
--blue:#007acc;--acc:#4fc1ff;--ok:#89d185;--warn:#cca700;--bad:#f48771;--irid:#c586c0;--gps:#4ec9b0}
*{box-sizing:border-box}html,body{height:100%;margin:0}[hidden]{display:none!important}
body{background:var(--bg);color:var(--fg);font:13px/1.4 "Segoe UI",system-ui,-apple-system,sans-serif;
display:grid;grid-template-rows:30px 1fr 22px;overflow:hidden}
.mono{font-family:Consolas,"Cascadia Code",Menlo,"DejaVu Sans Mono",monospace;font-variant-numeric:tabular-nums}
header{background:#323233;display:flex;align-items:center;gap:12px;padding:0 12px;font-size:12px;color:var(--dim);border-bottom:1px solid #2b2b2b}
header b{color:var(--fg);font-weight:600}
main{display:grid;grid-template-columns:280px 1fr;min-height:0}
aside{background:var(--side);border-right:1px solid var(--line);overflow:auto;padding:0 0 12px}
.sec{padding:10px 14px 4px;font-size:11px;letter-spacing:.06em;text-transform:uppercase;color:var(--dim);font-weight:600}
.kv{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;padding:2px 14px 8px}
.kv span:nth-child(odd){color:var(--dim)}.kv span:nth-child(even){text-align:right}
.big{font-size:28px;color:var(--fg);padding:0 14px;line-height:1.1}
.big small{font-size:12px;color:var(--dim)}
.pill{display:inline-block;padding:1px 8px;border-radius:9px;font-size:11px;font-weight:600}
.pill.on{background:#16825d;color:#fff}.pill.den{background:#a1260d;color:#fff}
.content{display:grid;grid-template-rows:minmax(0,1fr) minmax(0,1.2fr) 190px;min-height:0;min-width:0}
.tab{flex:none;white-space:nowrap;overflow:hidden}
.pane{border-bottom:1px solid var(--line);display:flex;flex-direction:column;min-height:0;min-width:0}
.tab{height:28px;display:flex;align-items:center;gap:14px;padding:0 12px;background:var(--side);font-size:12px;color:var(--dim);border-bottom:1px solid var(--line)}
.tab b{color:var(--fg);font-weight:400;border-bottom:1px solid var(--blue);padding-bottom:4px;margin-top:5px}
.key{display:inline-flex;align-items:center;gap:5px}.key i{width:10px;height:2px;display:inline-block}
.split{display:grid;grid-template-columns:1fr 1fr;min-height:0;flex:1}
canvas{width:100%;height:100%;display:block}
.cv{position:relative;min-height:0;min-width:0;flex:1}
.cv+.cv{border-left:1px solid var(--line)}
.term{overflow:auto;padding:6px 12px;font-size:12px;flex:1}
.term div{white-space:pre}.term .t{color:var(--dim)}.term .d{color:var(--bad)}.term .e{color:var(--ok)}
table{border-collapse:collapse;width:100%;font-size:12px}
td,th{padding:3px 12px;text-align:right;white-space:nowrap}th{color:var(--dim);font-weight:400}td:first-child,th:first-child{text-align:left}
footer{background:var(--blue);color:#fff;display:flex;align-items:center;gap:16px;padding:0 10px;font-size:12px}
footer.den{background:#a1260d}footer .r{margin-left:auto}
@media (max-width:760px){main{grid-template-columns:1fr}aside{display:none}.split{grid-template-columns:1fr}}
</style></head><body>
<header><b>NAVCORE SoOP</b><span>live session</span><span id="file" class="mono"></span></header>
<main>
<aside>
  <div class="sec">Iridium error now</div>
  <div class="big mono" id="err">—</div>
  <div class="kv mono">
    <span>fix</span><span id="ifix">—</span>
    <span>satellites</span><span id="isats">—</span>
    <span>reported hacc</span><span id="ihacc">—</span>
    <span>north / east</span><span id="ine">—</span>
    <span>in view &gt;10°</span><span id="inview">—</span>
  </div>
  <div style="height:54px;padding:0 14px"><canvas id="sats"></canvas></div>
  <div class="sec">u-blox</div>
  <div class="kv mono">
    <span>state</span><span id="gstate">—</span>
    <span>satellites</span><span id="gsats">—</span>
    <span>hacc</span><span id="ghacc">—</span>
  </div>
  <div class="sec">Windows</div>
  <table class="mono"><thead><tr><th></th><th>median</th><th>p95</th><th>in 2σ</th></tr></thead><tbody id="wins"></tbody></table>
  <div class="sec">Reference</div>
  <div class="kv mono"><span>lat</span><span id="rlat">—</span><span>lon</span><span id="rlon">—</span></div>
  <div style="padding:0 14px;color:var(--dim);font-size:11px" id="rhow"></div>
</aside>
<div class="content">
  <div class="pane"><div class="tab"><b class="tb" data-v="tl">error.timeline</b><span class="tb" data-v="dp">doppler.predicted</span><span class="tb" data-v="wf">bursts.waterfall</span>
    <span class="key k-wf" hidden><i style="background:#89d185;height:6px;width:6px;border-radius:3px"></i>fused</span>
    <span class="key k-wf" hidden><i style="background:#cca700;height:6px;width:6px;border-radius:3px"></i>other</span>
    <span class="key k-wf" hidden>size = C/N0 · lines = predicted Doppler</span>
    <span class="key k-wf" hidden id="wfinfo"></span>
    <span class="key k-tl"><i style="background:var(--irid)"></i>Iridium error</span>
    <span class="key k-tl"><i style="background:#c586c055;height:8px"></i>reported hacc</span>
    <span class="key k-tl"><i style="background:#a1260d66;height:8px"></i>GPS denied</span>
    <span class="key k-dp" hidden><i style="background:#4fc1ff22;height:8px"></i>DSP search ±50 kHz</span>
    <span class="key k-dp" hidden><i style="background:var(--fg);width:2px;height:10px"></i>now</span></div>
    <div class="split" style="grid-template-columns:1fr auto"><div class="cv"><canvas id="tl"></canvas><canvas id="dp" hidden></canvas><canvas id="wf" hidden></canvas></div>
    <div class="term mono k-dp" hidden style="padding:0;border-left:1px solid var(--line);min-width:330px"><table>
      <thead><tr><th>sat</th><th>el</th><th>az</th><th>range</th><th>doppler</th><th>rate</th></tr></thead><tbody id="look"></tbody></table></div></div></div>
  <div class="pane"><div class="tab"><b>position.map</b>
    <span class="key"><i style="background:var(--irid);height:6px;width:6px;border-radius:3px"></i>Iridium</span>
    <span class="key"><i style="background:var(--gps);height:6px;width:6px;border-radius:3px"></i>u-blox</span>
    <span class="key">+ surveyed spot</span>
    <b style="margin-left:auto">iridium.sky</b>
    <span class="key"><i style="background:var(--irid);height:6px;width:6px;border-radius:3px"></i>above 10°</span>
    <span class="key"><i style="background:var(--dim);height:6px;width:6px;border-radius:3px"></i>low</span></div>
    <div class="split"><div class="cv"><canvas id="map"></canvas></div><div class="cv"><canvas id="sky"></canvas></div></div></div>
  <div class="pane" style="border:0"><div class="split" style="min-height:0">
    <div style="display:flex;flex-direction:column;min-height:0"><div class="tab"><b>session.log</b></div><div class="term mono" id="log"></div></div>
    <div style="display:flex;flex-direction:column;min-height:0;border-left:1px solid var(--line)"><div class="tab"><b>passes.next</b><span>above 10°, today's TLEs</span></div>
      <div class="term mono" style="padding:0"><table><thead><tr><th>sat</th><th>rise</th><th>peak</th><th>max el</th><th>in</th></tr></thead><tbody id="passes"></tbody></table></div></div>
  </div></div>
</div>
</main>
<footer id="sb"><span id="sbstate">connecting…</span><span id="sbphase" class="mono"></span><span class="r mono" id="sbtime"></span></footer>
<script>
const $=id=>document.getElementById(id), css=v=>getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const hms=s=>{s=((s%86400)+86400)%86400;const h=Math.floor(s/3600),m=Math.floor(s%3600/60),x=Math.floor(s%60);return [h,m,x].map(v=>String(v).padStart(2,'0')).join(':')};
const fm=v=>v==null?'—':(v<10?v.toFixed(1):Math.round(v))+' m';
function setup(c){const r=c.getBoundingClientRect(),d=devicePixelRatio||1;c.width=r.width*d;c.height=r.height*d;const g=c.getContext('2d');g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,r.width,r.height);g.font='11px Consolas,Menlo,monospace';return [g,r.width,r.height]}
function nice(v){const p=Math.pow(10,Math.floor(Math.log10(v))),f=v/p;return (f<=1?1:f<=2?2:f<=5?5:10)*p}
function timeline(d){
  const [g,W,H]=setup($('tl'));const L=48,R=12,T=10,B=22,tr=d.track;if(!tr.length)return;
  const t0=tr[0][0],t1=d.now,err=tr.map(p=>Math.hypot(p[1],p[2]));
  const ymax=nice(Math.max(10,...err.slice(-1800))*1.15);
  const x=t=>L+(t-t0)/(Math.max(60,t1-t0))*(W-L-R),y=v=>T+(1-Math.min(v,ymax)/ymax)*(H-T-B);
  g.fillStyle='#a1260d33';let s=null;tr.forEach((p,i)=>{if(p[6]&&s===null)s=p[0];if((!p[6]||i==tr.length-1)&&s!==null){g.fillRect(x(s),T,x(p[0])-x(s),H-T-B);s=null}});
  g.strokeStyle=css('--line');g.fillStyle=css('--dim');g.lineWidth=1;
  for(let k=0;k<=4;k++){const v=ymax*k/4,yy=y(v);g.beginPath();g.moveTo(L,yy+.5);g.lineTo(W-R,yy+.5);g.stroke();g.textAlign='right';g.fillText(v+' m',L-6,yy+4)}
  const span=t1-t0,step=span>2400?600:span>900?300:span>300?60:30;g.textAlign='center';
  for(let t=Math.ceil(t0/step)*step;t<=t1;t+=step)g.fillText(hms(t).slice(0,5),x(t),H-6);
  g.fillStyle='#c586c033';g.beginPath();tr.forEach((p,i)=>{const yy=y(Math.min(p[3],ymax));i?g.lineTo(x(p[0]),yy):g.moveTo(x(p[0]),yy)});
  g.lineTo(x(tr[tr.length-1][0]),y(0));g.lineTo(x(tr[0][0]),y(0));g.fill();
  g.strokeStyle=css('--irid');g.lineWidth=1.5;g.beginPath();tr.forEach((p,i)=>{i?g.lineTo(x(p[0]),y(err[i])):g.moveTo(x(p[0]),y(err[i]))});g.stroke();
  const l=tr.length-1;g.fillStyle=css('--irid');g.beginPath();g.arc(x(tr[l][0]),y(err[l]),3,0,7);g.fill();
}
function map(d){
  const [g,W,H]=setup($('map'));const tr=d.track.slice(-1800),u=d.ublox;
  const ext=nice(Math.max(20,...tr.map(p=>Math.max(Math.abs(p[1]),Math.abs(p[2]))),...u.map(p=>Math.max(Math.abs(p[1]),Math.abs(p[2]))))*1.1);
  const cx=W/2,cy=H/2,sc=(Math.min(W,H)/2-18)/ext,X=e=>cx+e*sc,Y=n=>cy-n*sc;
  g.strokeStyle=css('--line');g.fillStyle=css('--dim');
  [ext/2,ext].forEach(r=>{g.beginPath();g.arc(cx,cy,r*sc,0,7);g.stroke();g.fillText(r+' m',cx+r*sc*0.71+3,cy-r*sc*0.71)});
  g.beginPath();g.moveTo(cx,10);g.lineTo(cx,H-10);g.moveTo(10,cy);g.lineTo(W-10,cy);g.stroke();g.fillText('N',cx+4,14);
  u.forEach(p=>{g.fillStyle='#4ec9b055';g.fillRect(X(p[2])-1,Y(p[1])-1,2,2)});
  tr.forEach((p,i)=>{g.fillStyle=p[6]?'#c586c0':'#c586c066';const r=i==tr.length-1?4:1.6;g.beginPath();g.arc(X(p[2]),Y(p[1]),r,0,7);g.fill()});
  g.strokeStyle=css('--fg');g.lineWidth=1.5;g.beginPath();g.moveTo(cx-6,cy);g.lineTo(cx+6,cy);g.moveTo(cx,cy-6);g.lineTo(cx,cy+6);g.stroke();
}
function sats(d){
  const [g,W,H]=setup($('sats'));const tr=d.track,T=12,B=2;if(!tr.length)return;
  const t0=tr[0][0],t1=d.now,x=t=>(t-t0)/Math.max(60,t1-t0)*W,m=Math.max(4,...tr.map(p=>p[4])),y=v=>T+(1-v/m)*(H-T-B);
  g.fillStyle=css('--dim');g.fillText('satellites tracked, last hour',0,9);
  g.strokeStyle=css('--acc');g.lineWidth=1.2;g.beginPath();tr.forEach((p,i)=>{i?g.lineTo(x(p[0]),y(p[4])):g.moveTo(x(p[0]),y(p[4]))});g.stroke();
}
function sky(d){
  const [g,W,H]=setup($('sky'));if(!d.sky){g.fillStyle=css('--dim');g.fillText('no sky view: run with sgp4 + numpy',12,20);return}
  const cx=W/2,cy=H/2,R=Math.min(W,H)/2-16,P=(az,el)=>{const r=R*(90-el)/90,a=az*Math.PI/180;return [cx+r*Math.sin(a),cy-r*Math.cos(a)]};
  g.strokeStyle=css('--line');g.fillStyle=css('--dim');g.textAlign='center';
  [0,30,60].forEach(el=>{g.beginPath();g.arc(cx,cy,R*(90-el)/90,0,7);g.stroke();if(el)g.fillText(el+'°',cx+3,cy-R*(90-el)/90+11)});
  g.beginPath();g.moveTo(cx,cy-R);g.lineTo(cx,cy+R);g.moveTo(cx-R,cy);g.lineTo(cx+R,cy);g.stroke();
  [['N',0],['E',90],['S',180],['W',270]].forEach(([s,a])=>{const [x,y]=P(a,-9);g.fillText(s,x,y+4)});
  g.textAlign='left';const boxes=[];
  d.sky.forEach(s=>{const [x,y]=P(s.az,s.el),hi=s.el>=10;g.fillStyle=hi?css('--irid'):css('--dim');g.beginPath();g.arc(x,y,hi?4:2.5,0,7);g.fill();
    if(!hi)return;let ly=y-2;while(boxes.some(b=>Math.abs(b[0]-(x+6))<70&&Math.abs(b[1]-ly)<24))ly+=24;boxes.push([x+6,ly]);
    if(ly!==y-2){g.strokeStyle=css('--dim');g.beginPath();g.moveTo(x,y);g.lineTo(x+5,ly-4);g.stroke()}
    g.fillStyle=css('--fg');g.fillText(s.name,x+6,ly);g.fillStyle=css('--dim');g.fillText((s.dop>0?'+':'')+(s.dop/1000).toFixed(1)+' kHz',x+6,ly+12)});
  $('inview').textContent=d.sky.filter(s=>s.el>=10).length+' of '+d.sky.length+' up';
}
let view='tl';
document.querySelectorAll('.tb').forEach(el=>{el.style.cursor='pointer';el.onclick=()=>{view=el.dataset.v;
  document.querySelectorAll('.tb').forEach(t=>{const on=t.dataset.v==view;t.style.color=on?css('--fg'):'';t.style.borderBottom=on?'1px solid var(--blue)':'0';t.style.paddingBottom=on?'4px':'0';t.style.marginTop=on?'5px':'0'});
  ['tl','dp','wf'].forEach(v=>{$(v).hidden=view!=v;document.querySelectorAll('.k-'+v).forEach(k=>k.hidden=view!=v)});tick()}});
let WF=null;fetch('/bursts').then(r=>r.json()).then(j=>{WF=j}).catch(()=>{});
function waterfall(){
  if(view!='wf')return;const [g,W,H]=setup($('wf'));
  if(!WF||!WF.bursts.length){g.fillStyle=css('--dim');g.fillText('no burst log loaded: run with --logs SD/*.BIN',12,20);return}
  const b=WF.bursts,L=56,R=12,T=10,B=22,t0=Math.min(...b.map(p=>p[0]))-30,t1=Math.max(...b.map(p=>p[0]))+30;
  const x=t=>L+(t-t0)/(t1-t0)*(W-L-R),y=f=>T+(1-(f+60000)/120000)*(H-T-B);
  g.strokeStyle=css('--line');g.fillStyle=css('--dim');g.textAlign='right';
  for(let f=-60000;f<=60000;f+=20000){const yy=y(f);g.beginPath();g.moveTo(L,yy+.5);g.lineTo(W-R,yy+.5);g.stroke();g.fillText((f>0?'+':'')+f/1000+' kHz',L-6,yy+4)}
  g.textAlign='center';const st=(t1-t0)>1800?600:(t1-t0)>600?120:60;for(let t=Math.ceil(t0/st)*st;t<=t1;t+=st)g.fillText(hms(t).slice(0,5),x(t),H-6);
  WF.curves.forEach((c,i)=>{g.strokeStyle=PAL[i%PAL.length]+'99';g.lineWidth=1;g.beginPath();let on=false;
    c.pts.forEach(p=>{if(p[0]<t0||p[0]>t1){on=false;return}on?g.lineTo(x(p[0]),y(p[1])):g.moveTo(x(p[0]),y(p[1]));on=true});g.stroke()});
  b.forEach(p=>{g.fillStyle=p[3]==0?'#89d185':'#cca70099';g.beginPath();g.arc(x(p[0]),y(p[1]),Math.max(1.2,(p[2]-40)/5),0,7);g.fill()});
  $('wfinfo').textContent=b.length+' bursts · '+b.filter(p=>p[3]==0).length+' fused · '+WF.logs.join(', ');
}
const PAL=['#c586c0','#4fc1ff','#4ec9b0','#dcdcaa','#ce9178','#9cdcfe','#d7ba7d','#b5cea8'];
function doppler(d){
  if(view!='dp')return;const [g,W,H]=setup($('dp'));if(!d.doppler){g.fillStyle=css('--dim');g.fillText('no sky view',12,20);return}
  const L=56,R=70,T=10,B=22,t0=d.now-600,t1=d.now+600,x=t=>L+(t-t0)/(t1-t0)*(W-L-R),y=f=>T+(1-(f+40000)/80000)*(H-T-B);
  g.fillStyle='#4fc1ff14';g.fillRect(L,y(40000),W-L-R,y(-40000)-y(40000));
  g.strokeStyle=css('--line');g.fillStyle=css('--dim');g.lineWidth=1;g.textAlign='right';
  for(let f=-40000;f<=40000;f+=10000){const yy=y(f);g.beginPath();g.moveTo(L,yy+.5);g.lineTo(W-R,yy+.5);g.stroke();g.fillText((f>0?'+':'')+f/1000+' kHz',L-6,yy+4)}
  g.textAlign='center';for(let t=Math.ceil(t0/300)*300;t<=t1;t+=300)g.fillText(hms(t).slice(0,5),x(t),H-6);
  g.textAlign='left';
  d.doppler.forEach((s,i)=>{const c=s.hi?PAL[i%PAL.length]:'#5a5a5a';g.strokeStyle=c;g.lineWidth=s.hi?1.6:1;g.beginPath();
    s.pts.forEach((p,k)=>{k?g.lineTo(x(p[0]),y(p[1])):g.moveTo(x(p[0]),y(p[1]))});g.stroke();
    const last=s.pts[s.pts.length-1];if(s.hi){g.fillStyle=c;g.fillText(s.name,Math.min(x(last[0])+4,W-R+4),y(last[1])+4)}});
  g.strokeStyle=css('--fg');g.beginPath();g.moveTo(x(d.now)+.5,T);g.lineTo(x(d.now)+.5,H-B);g.stroke();
  (d.sky||[]).forEach(s=>{g.fillStyle=s.el>=10?css('--fg'):css('--dim');g.beginPath();g.arc(x(d.now),y(s.dop),3,0,7);g.fill()});
  $('look').replaceChildren(...(d.sky||[]).sort((a,b)=>b.el-a.el).map(s=>{const tr=document.createElement('tr');
    [s.name,s.el.toFixed(0)+'°',s.az.toFixed(0)+'°',s.rng+' km',(s.dop>0?'+':'')+(s.dop/1000).toFixed(2)+' kHz',(s.rate>0?'+':'')+s.rate.toFixed(0)+' Hz/s']
      .forEach(v=>{const td=document.createElement('td');td.textContent=v;if(s.el<10)td.style.color=css('--dim');tr.append(td)});return tr}));
}
function passes(d){
  if(!d.passes)return;const now=d.now;
  $('passes').replaceChildren(...d.passes.filter(p=>p.set>=now).slice(0,30).map(p=>{const tr=document.createElement('tr'),live=p.rise<=now;
    const inn=live?'now':Math.round((p.rise-now)/60)+' min';
    [p.name,hms(p.rise).slice(0,5),hms(p.peak).slice(0,5),p.max_el.toFixed(0)+'°',inn].forEach((v,i)=>{const td=document.createElement('td');td.textContent=v;
      if(i==3)td.style.color=p.max_el>=60?css('--ok'):p.max_el>=30?css('--fg'):css('--dim');if(i==4&&live)td.style.color=css('--irid');tr.append(td)});return tr}));
}
async function tick(){
  let d;try{d=await (await fetch('/api',{cache:'no-store'})).json()}catch(e){$('sbstate').textContent='server not reachable';return}
  if(!d.ok){$('sbstate').textContent=d.why;return}
  $('file').textContent=d.file;
  const p=d.track.length?d.track[d.track.length-1]:null;
  if(p){const e=Math.hypot(p[1],p[2]);
    $('err').innerHTML=fm(e).replace(' m','<small> m</small>');
    $('ifix').textContent=['none','no fix','2D','3D'][p[5]]||p[5];$('isats').textContent=p[4];$('ihacc').textContent=fm(p[3]);
    $('ine').textContent=p[1].toFixed(0)+' / '+p[2].toFixed(0)+' m';}
  else{$('err').innerHTML='<small>waiting for the first Iridium fix</small>';['ifix','isats','ihacc','ine'].forEach(k=>$(k).textContent='—')}
  const u=d.ublox_now;$('gstate').innerHTML=d.denied?'<span class="pill den">DENIED</span>':'<span class="pill on">IN USE</span>';
  $('gsats').textContent=u?u.sats:'—';$('ghacc').textContent=u?fm(u.hacc):'—';
  $('rlat').textContent=d.ref[0].toFixed(7);$('rlon').textContent=d.ref[1].toFixed(7);$('rhow').textContent=d.ref_how;
  $('wins').replaceChildren(...d.windows.filter(w=>w.denied).map(w=>{const tr=document.createElement('tr');
    [hms(w.start).slice(0,5),fm(w.median),fm(w.p95),Math.round(w.inside*100)+'%'].forEach(v=>{const td=document.createElement('td');td.textContent=v;tr.append(td)});return tr}));
  const lg=$('log'),atEnd=lg.scrollTop+lg.clientHeight>=lg.scrollHeight-4;
  lg.replaceChildren(...d.log.map(([t,m])=>{const div=document.createElement('div'),a=document.createElement('span'),b=document.createElement('span');
    a.className='t';a.textContent=t.slice(0,8)+'  ';b.textContent=m;b.className=m.includes('DISABLED')?'d':m.includes('ENABLED')?'e':'';div.append(a,b);return div}));
  if(atEnd)lg.scrollTop=lg.scrollHeight;
  $('sb').className=d.denied?'den':'';$('sbstate').textContent=d.denied?'⛔ GPS DENIED — Iridium only':'✓ GPS in use — Iridium anchored';
  $('sbphase').textContent='for '+((d.now-d.phase_start)/60).toFixed(1)+' min';$('sbtime').textContent=hms(d.now)+' UTC';
  if(view=='tl')timeline(d);doppler(d);waterfall();map(d);sats(d);sky(d);passes(d);
}
tick();setInterval(tick,1000);addEventListener('resize',tick);
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("session")
    ap.add_argument("--ref", type=float, nargs=2, metavar=("LAT", "LON"))
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--tle", default=TLE_DEFAULT, help="Iridium TLEs for the sky view")
    ap.add_argument("--logs", nargs="*", default=[], help="dataflash BINs for the burst view")
    a = ap.parse_args()
    try:
        sky = Sky(a.tle)
    except (ImportError, OSError) as e:
        print(f"no sky view ({e}); pip install sgp4 numpy for it")
        sky = None
    wf = None
    if a.logs:
        snap = snapshot(a.session, a.ref)
        ref = tuple(snap["ref"]) if snap.get("ok") else tuple(a.ref or (0.0, 0.0))
        wf = json.dumps(burst_waterfall(a.logs, sky, ref)).encode()
        print(f"burst view from {len(a.logs)} logs")

    class H(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.startswith("/bursts"):
                body, ctype = wf or b'{"bursts":[],"curves":[],"logs":[]}', "application/json"
            elif self.path.startswith("/api"):
                body = json.dumps(snapshot(a.session, a.ref, sky)).encode()
                ctype = "application/json"
            else:
                body, ctype = PAGE.encode(), "text/html; charset=utf-8"
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    print(f"http://localhost:{a.port}  ({a.session})")
    http.server.ThreadingHTTPServer(("127.0.0.1", a.port), H).serve_forever()


if __name__ == "__main__":
    sys.exit(main())
