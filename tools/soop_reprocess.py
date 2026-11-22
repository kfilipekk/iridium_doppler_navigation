#!/usr/bin/env python3
"""Run the board's DSP again over its own I/Q recordings, on the host, the right way up.

Usage:
  soop_reprocess.py LOG.npz IQ_DIR OUT.csv [--swap] [--jobs N] [--files A-B]

LOG.npz is soop_extract.py's: SOI says where each iqNNNN file starts (capture sample and board
time), SOD which blocks the recorder dropped and SOG where the capture itself skipped. Each file
(.iq4, as recorded to 2026-11-20) is laid back on the capture's sample timeline - dropped and skipped blocks as zeros - so every
burst dsp_host finds gets its board time exactly: SOI.T + sample / SOOP_FS. SOD is written as a
streaming (droppable) log message and the logger drops some under load - 2026-11-18: 13406 of
60982 on the return flight, 110 s in all - so the recorder's own cumulative counter (SOQ.D, 1 Hz)
says how many really went in each second; the unlogged ones are put beside that second's logged
ones (drops come in runs, while the card stalls), or at its start. --swap exchanges I
and Q first: Rev E's spectrum comes out mirrored about the LO (2026-11-20, every session so far),
which hides Ring Alert from the DSP. Block-numbered files (.iq4b/.i16b, firmware since
8c0988e) carry their own timeline and are already the right way up unless SOW says otherwise:
leave --swap off for them. The csv adds t (board s) and the file number to dsp_host's.
"""
import argparse
import csv
import os
import subprocess
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DSP_HOST = os.path.join(os.path.dirname(HERE), "firmware", "soop", "dsp_host")
FS = 500000.0
BLK = 4096


def blocks_of(soi, sod, sog, k):
    """The capture block starts file k holds, in order, and the span it covers."""
    s0 = int(soi[k, 2])
    s1 = int(soi[k + 1, 2]) if k + 1 < len(soi) else None
    dropped = set(int(x) for x in sod[:, 1]) if len(sod) else set()
    gaps = [(int(a), int(b)) for a, b in sog[:, 1:3]] if len(sog) else []
    return s0, s1, dropped, gaps


def unpack4(raw):
    n = len(raw) // (4 + BLK)
    raw = raw[:n * (4 + BLK)].reshape(n, 4 + BLK)
    step = raw[:, :4].copy().view("<f4")[:, 0]
    b = raw[:, 4:]
    hi = (b >> 4).astype(np.int8)
    lo = (b & 0xF).astype(np.int8)
    hi[hi >= 8] -= 16
    lo[lo >= 8] -= 16
    i = np.clip(np.rint((hi + 0.5) * step[:, None]), -32768, 32767).astype(np.int16)
    q = np.clip(np.rint((lo + 0.5) * step[:, None]), -32768, 32767).astype(np.int16)
    return i, q


def unlogged_drops(soq, sod, t_of):
    """Per SOQ interval (t0, t1]: drops the counter saw that no SOD record covers."""
    if soq is None or len(soq) < 2:
        return np.array([]), np.array([])
    t_sod = t_of(sod[:, 1]) if len(sod) else np.array([])
    edges = soq[:, 0]
    logged = np.histogram(t_sod, edges)[0] if len(t_sod) else np.zeros(len(edges) - 1, int)
    counted = np.diff(soq[:, 2])
    return edges, np.maximum(counted - logged, 0).astype(int)


def numbered_file(path, t_s0, s0, swap, out_path):
    """A .iq4b/.i16b file: every record carries its capture block, so the timeline is the file's
    own - no SOD or SOQ needed. Missing blocks become zeros."""
    sys.path.insert(0, HERE)
    import soop_iq
    blocks, iq = soop_iq.read(path)
    if swap:
        iq = iq[..., ::-1]
    b0 = int(s0 // BLK)
    full = np.zeros((int(blocks[-1]) - b0 + 1, BLK, 2), np.int16)
    keep = blocks >= b0
    full[blocks[keep] - b0] = iq[keep]
    full.tofile(out_path)
    return f"{len(blocks)} recorded, {len(full) - keep.sum()} missing (block numbers)"


def one_file(args):
    k, soi, sod, sog, soq, iq_dir, swap, tmpdir = args
    fno, t_s0 = int(soi[k, 1]), float(soi[k, 3])
    for ext in ("iq4b", "i16b"):
        path = os.path.join(iq_dir, "iq%04d.%s" % (fno, ext))
        if os.path.exists(path):
            out_path = os.path.join(tmpdir, "f%04d.bin" % fno)
            note = numbered_file(path, t_s0, int(soi[k, 2]), swap, out_path)
            return k, run_dsp(out_path, t_s0, fno), note
    path = os.path.join(iq_dir, "iq%04d.iq4" % fno)
    if not os.path.exists(path):
        return k, [], "missing"
    s0, s1, dropped, gaps = blocks_of(soi, sod, sog, k)
    i, q = unpack4(np.fromfile(path, dtype=np.uint8))
    if swap:
        i, q = q, i
    n_rec = len(i)
    #walk the capture timeline from s0: each block start is recorded, dropped, or never captured
    out_path = os.path.join(tmpdir, "f%04d.bin" % fno)
    zero = np.zeros(2 * BLK, np.int16)
    blk = np.empty(2 * BLK, np.int16)
    gi = 0
    gaps = sorted(g for g in gaps if g[1] > s0)
    s, r, n_zero = s0, 0, 0
    #capture sample -> board time, along this file's own timeline
    t_of = lambda smp: t_s0 + (np.asarray(smp, float) - s0) / FS
    edges, missing = unlogged_drops(soq, sod, t_of)
    missing = missing.copy()
    #seconds with logged drops get theirs beside those runs; the rest at the second's start
    t_drop = t_of(np.array(sorted(dropped), float)) if dropped else np.array([])
    has_logged = (np.histogram(t_drop, edges)[0] > 0) if len(edges) and len(t_drop) else \
        np.zeros(max(len(edges) - 1, 0), bool)
    n_late, in_run, j_prev = 0, False, -1
    with open(out_path, "wb") as f:
        while r < n_rec and (s1 is None or s < s1):
            while gi < len(gaps) and gaps[gi][1] <= s:
                gi += 1
            j = int(np.searchsorted(edges, t_of(s))) - 1 if len(edges) else -1
            if j != j_prev and 0 <= j_prev < len(missing) and 0 <= j < len(missing):
                missing[j] += missing[j_prev]             #what a second could not place
                missing[j_prev] = 0
            j_prev = j
            spare = 0 <= j < len(missing) and missing[j] > 0
            if gi < len(gaps) and gaps[gi][0] <= s < gaps[gi][1]:
                f.write(zero.tobytes()); n_zero += 1          #the capture skipped this block
                in_run = False
            elif s in dropped:
                f.write(zero.tobytes()); n_zero += 1          #captured, never recorded
                in_run = True
            elif spare and (in_run or not has_logged[j]):
                f.write(zero.tobytes()); n_zero += 1; n_late += 1   #dropped, SOD lost
                missing[j] -= 1
            else:
                blk[0::2], blk[1::2] = i[r], q[r]
                f.write(blk.tobytes()); r += 1
                in_run = False
            s += BLK
    note = f"{n_rec} recorded, {n_zero} zero-filled ({n_late} from the counter), {n_rec - r} left over"
    return k, run_dsp(out_path, t_s0, fno), note


def run_dsp(out_path, t_s0, fno):
    #SOOP_DSP_LO (Hz) centres the DSP elsewhere than the Ring Alert, e.g. -83333 for the two
    #simplex channels 125-167 kHz above it; the workers are separate processes, so not an option
    lo = os.environ.get("SOOP_DSP_LO")
    res = subprocess.run([DSP_HOST, out_path] + (["--lo", lo] if lo else []), capture_output=True, text=True)
    os.remove(out_path)
    rows = list(csv.DictReader(res.stdout.splitlines()))
    for row in rows:
        row["t"] = "%.6f" % (t_s0 + float(row["t_sample"]) / FS)
        row["file"] = str(fno)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("npz")
    ap.add_argument("iq_dir")
    ap.add_argument("out")
    ap.add_argument("--swap", action="store_true")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--files", help="first-last file numbers to do, e.g. 64-70")
    ap.add_argument("--soq", help="SOQ records (t, Q, D, W) as .npy, if not in the npz")
    a = ap.parse_args()
    subprocess.run(["make", "-s", "-C", os.path.dirname(DSP_HOST), "dsp_host"], check=True)
    D = np.load(a.npz)
    soi, sod, sog = D["soi"], D["sod"], D["sog"]
    soq = np.load(a.soq) if a.soq else (D["soq"] if "soq" in D else None)
    ks = range(len(soi))
    if a.files:
        lo, hi = map(int, a.files.split("-"))
        ks = [k for k in ks if lo <= soi[k, 1] <= hi]
    tmp = tempfile.mkdtemp(prefix="soop_rep_")
    rows_all = []
    with ProcessPoolExecutor(a.jobs) as ex:
        for k, rows, note in ex.map(one_file, [(k, soi, sod, sog, soq, a.iq_dir, a.swap, tmp) for k in ks]):
            print("iq%04d: %d bursts, %d Ring Alerts decoded (%s)" % (
                soi[k, 1], len(rows), sum(int(r["ira"]) for r in rows), note), flush=True)
            rows_all += rows
    os.rmdir(tmp)
    if rows_all:
        with open(a.out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["t", "file"] + [c for c in rows_all[0] if c not in ("t", "file")])
            w.writeheader()
            w.writerows(rows_all)
    print(f"{len(rows_all)} bursts -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
