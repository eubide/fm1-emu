#!/usr/bin/env python3
"""The AC79's FFT / vector unit (JL_FFT, crates/fm1-soc/src/hwdsp.rs) measured: a plan of jobs run by
fm1-dock's probes/hwdsp.c on an FM-1 and in fm1-emu, the results kept as the emulator's reference.

  tools/hwdsp.py run --on emu|fm1 [--plan fft|vec|all] [--only SUBSTR] [--dock DIR] [--port NAME]
      runs the plan's cases (one probe run each) and merges what came back into tests/hwdsp/ON.json
  tools/hwdsp.py compare [A B]      case by case, where two result files differ (default emu fm1)
  tools/hwdsp.py fit [ON]           each FFT case against the exact transform (numpy): the scale
                                    that fits best, the error left after it, saturation
  tools/hwdsp.py export [ON]        tests/hwdsp/ON.txt: a line a case, `id | args | ticks | con | out`
                                    (hex words): what crates/fm1-soc/tests/hwdsp_fm1.rs checks the model by

--dock: an fm1-dock checkout with probes/hwdsp.c and a build (tools/build.py; default ../fm1-harbor);
the probe is built there once (build/probes/hwdsp.dock). emu runs fm1-live ($FM1_LIVE, default this
checkout's target/release/fm1-live): build it first. fm1: fm1-dock on an FM-1 over USB-MIDI (the probe
runs from RAM, nothing is written to flash; python-rtmidi: uv run --with python-rtmidi ...).

A case's input is made from a seed by the probe; gen() here makes the same words (probes/hwdsp.c's
fill(), keep them in step)."""
import argparse
import json
import os
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "tests" / "hwdsp"
WORDS = 2048 + 8

# the SDK's FFT configurations (fftvec_math_drv.h, its #defines evaluated)
FFT = dict([
    ("COMPLEX_FFT10", 0x200a2100), ("COMPLEX_IFFT10", 0x200a2104), ("COMPLEX_FFT20", 0x20142200),
    ("COMPLEX_IFFT20", 0x30142204), ("COMPLEX_FFT15", 0x200f3000), ("COMPLEX_IFFT15", 0x200f3004),
    ("COMPLEX_FFT30", 0x201e3100), ("COMPLEX_IFFT30", 0x301e3104), ("COMPLEX_FFT32", 0x20200510),
    ("COMPLEX_IFFT32", 0x40200514), ("COMPLEX_FFT40", 0x20282300), ("COMPLEX_IFFT40", 0x40282304),
    ("COMPLEX_FFT60", 0x203c3200), ("COMPLEX_IFFT60", 0x203c3204), ("COMPLEX_FFT64", 0x20400620),
    ("COMPLEX_IFFT64", 0x30400604), ("COMPLEX_FFT80", 0x20502410), ("COMPLEX_IFFT80", 0x30502404),
    ("COMPLEX_FFT120", 0x20783300), ("COMPLEX_IFFT120", 0x30783304), ("COMPLEX_FFT128", 0x20800720),
    ("COMPLEX_IFFT128", 0x40800714), ("COMPLEX_FFT160", 0x20a02510), ("COMPLEX_IFFT160", 0x40a02504),
    ("COMPLEX_FFT240", 0x20f03410), ("COMPLEX_IFFT240", 0x40f03414), ("COMPLEX_FFT256", 0x21000830),
    ("COMPLEX_IFFT256", 0x31000804), ("COMPLEX_FFT320", 0x21402620), ("COMPLEX_IFFT320", 0x31402604),
    ("COMPLEX_FFT480", 0x21e03520), ("COMPLEX_IFFT480", 0x31e03504), ("COMPLEX_FFT512", 0x22000930),
    ("COMPLEX_IFFT512", 0x42000914), ("COMPLEX_FFT960", 0x23c03620), ("COMPLEX_IFFT960", 0x43c03614),
    ("COMPLEX_FFT1024", 0x24000a40), ("COMPLEX_IFFT1024", 0x34000a04), ("COMPLEX_FFT2048", 0x28000b40),
    ("COMPLEX_IFFT2048", 0x48000b14), ("REAL_FFT20", 0x30142102), ("REAL_IFFT20", 0x30142106),
    ("REAL_FFT30", 0x301e3002), ("REAL_IFFT30", 0x301e3006), ("REAL_FFT40", 0x30282202),
    ("REAL_IFFT40", 0x40282206), ("REAL_FFT60", 0x303c3102), ("REAL_IFFT60", 0x403c3106),
    ("REAL_FFT64", 0x30400512), ("REAL_IFFT64", 0x30400506), ("REAL_FFT80", 0x30502312),
    ("REAL_IFFT80", 0x30502306), ("REAL_FFT120", 0x30783202), ("REAL_IFFT120", 0x30783206),
    ("REAL_FFT128", 0x30800622), ("REAL_IFFT128", 0x40800606), ("REAL_FFT160", 0x30a02412),
    ("REAL_IFFT160", 0x40a02406), ("REAL_FFT240", 0x30f03312), ("REAL_IFFT240", 0x40f03316),
    ("REAL_FFT256", 0x31000732), ("REAL_IFFT256", 0x31000706), ("REAL_FFT320", 0x31402522),
    ("REAL_IFFT320", 0x31402506), ("REAL_FFT480", 0x31e03422), ("REAL_IFFT480", 0x31e03406),
    ("REAL_FFT512", 0x32000832), ("REAL_IFFT512", 0x42000806), ("REAL_FFT960", 0x33c03522),
    ("REAL_IFFT960", 0x43c03506), ("REAL_FFT1024", 0x34000942), ("REAL_IFFT1024", 0x34000906),
    ("REAL_FFT2048", 0x38000a42), ("REAL_IFFT2048", 0x48000a06),
])

# the vector unit's control bits (fftvec_math_drv.h)
X32, Y32, Z32, Z64 = 2 << 6, 2 << 8, 2 << 10, 3 << 10
X16, Y16, Z16, X8, Y8, Z8 = 1 << 6, 1 << 8, 1 << 10, 0, 0, 0
MIN, MAX, CPX_QDT, CONST_MUL, NEG_ZERO, MAC32, QDT = 1 << 20, 1 << 19, 1 << 18, 1 << 17, 1 << 16, 1 << 15, 1 << 14
YCONJ, XCONJ, ACC, SUB = 1 << 13, 1 << 12, 1 << 5, 1 << 4
ADDSUB, MUL, CPX_MUL, QDT_MODE = 0, 1, 2, 3

# the input generators (probes/hwdsp.c's fill)
ZEROS, IMPULSE, DC, RANDOM, RAMP, RAW = range(6)


def gen(g, amp, p, n=WORDS):
    out, x = [], (p & 0xFFFFFFFF) or 1
    amp = amp - (1 << 32) if amp & 0x80000000 else amp
    for i in range(n):
        v = 0
        if g == IMPULSE:
            v = amp if i == p else 0
        elif g == DC:
            v = amp
        elif g in (RANDOM, RAW):
            x ^= (x << 13) & 0xFFFFFFFF
            x ^= x >> 17
            x ^= (x << 5) & 0xFFFFFFFF
            if g == RAW:
                v = x - (1 << 32) if x & 0x80000000 else x
            else:
                v = ((x >> 1) % (2 * amp + 1)) - amp if amp else 0
        elif g == RAMP:
            v = ((i * amp + p + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)
        out.append(v)
    return out


def fft_words(con):
    n, real, inv = (con >> 16) & 0xFFF, bool(con & 2), bool(con & 4)
    if not real:
        return 2 * n, 2 * n
    return (n + 2, n) if inv else (n, n + 2)


def plan_fft(full=False):
    """(id, args) for the FFT: every size of the SDK's table in full, else a spread; inputs an impulse
    at 0 and at 1, small and large random words, and full-scale ones (does it saturate, or wrap)"""
    pick = FFT if full else {k: v for k, v in FFT.items()
                             if int(''.join(c for c in k if c.isdigit())) in (64, 128, 256, 512, 1024, 120, 240, 20)}
    inputs = [("imp0", IMPULSE, 1 << 20, 0), ("imp1", IMPULSE, 1 << 20, 1), ("rnd14", RANDOM, 1 << 14, 7),
              ("rnd24", RANDOM, 1 << 24, 11), ("raw", RAW, 0, 13)]
    out = []
    for name, con in pick.items():
        nin, nout = fft_words(con)
        for iname, g, amp, p in inputs:
            for off in range(0, nout, 1900):
                cid = f"fft/{name}/{iname}" + (f"@{off}" if off else "")
                out.append((cid, [1, con, g, amp, p, off, min(1900, nout - off)]))
    return out


def vjob(con, nlen, q=0, nloop=1, inc=(0, 0, 0), const=0, x=(RANDOM, 1000, 3), y=(RANDOM, 1000, 5),
         z=(RAMP, 1, 0x100), n_out=None, config2=0):
    c0 = ((q & 0xFF) << 24) | (nloop << 12) | nlen
    c1 = (inc[2] << 20) | (inc[1] << 10) | inc[0]
    return [2, con, c0, c1, config2, const & 0xFFFFFFFF, *x, *y, *z, 0, n_out or min(1900, max(8, 2 * nlen * nloop))]


def plan_vec():
    """(id, args) for the vector unit: each of the SDK's operations as its driver configures it, at 32
    bits and narrower, with q's edges, saturation, and the loop / increment fields"""
    w32 = X32 | Y32 | Z32
    big = (RANDOM, 0x7FFFFFFF, 9)
    c = []
    for q in (0, 1, 5, 15, 31, -1, -3):
        c.append((f"vec/real_mul/q{q}", vjob(w32 | MUL, 64, q, x=(RANDOM, 1 << 20, 3), y=(RANDOM, 1 << 20, 5))))
    c += [
        ("vec/real_add", vjob(w32 | ADDSUB, 64)),
        ("vec/real_sub", vjob(w32 | ADDSUB | SUB, 64)),
        ("vec/real_add_sat", vjob(w32 | ADDSUB, 64, x=big, y=(RANDOM, 0x7FFFFFFF, 21))),
        ("vec/real_mul_sat", vjob(w32 | MUL, 64, 0, x=big, y=(RANDOM, 0x7FFFFFFF, 21))),
        ("vec/real_mul_round", vjob(w32 | MUL, 64, 4, x=(RANDOM, 100, 3), y=(RANDOM, 100, 5))),
        ("vec/real_mac32", vjob(w32 | MAC32 | MUL, 64, 6, z=(RANDOM, 1 << 20, 17))),
        ("vec/real_msc32", vjob(w32 | MAC32 | SUB | MUL, 64, 6, z=(RANDOM, 1 << 20, 17))),
        ("vec/real_dot", vjob(w32 | ACC | MUL, 64, 3)),
        ("vec/real_add_dot", vjob(w32 | ACC | ADDSUB, 64)),
        ("vec/real_sum", vjob(w32 | ACC | QDT_MODE, 64, 2, z=(ZEROS, 0, 0))),
        ("vec/real_qdt", vjob(w32 | QDT | QDT_MODE, 64, 4)),
        ("vec/real_qdtsum", vjob(w32 | QDT | ACC | QDT_MODE, 64, 4, z=(ZEROS, 0, 0))),
        ("vec/real_qdtsum_z64", vjob(X32 | Z64 | QDT | ACC | QDT_MODE, 64, 0, x=big, z=(ZEROS, 0, 0))),
        ("vec/mul_const", vjob(w32 | MUL | CONST_MUL, 64, 21, const=1677722, x=(RANDOM, 1 << 24, 3))),
        ("vec/max_const", vjob(w32 | QDT_MODE | MAX, 64, 0, const=100)),
        ("vec/min_const", vjob(w32 | QDT_MODE | MIN, 64, 0, const=-100)),
        ("vec/neg_zero", vjob(w32 | ADDSUB | SUB | NEG_ZERO, 64)),
        ("vec/cpx_add", vjob(w32 | ADDSUB, 2 * 32)),
        ("vec/cpx_mul", vjob(w32 | CPX_MUL, 32, 8)),
        ("vec/cpx_mul_xconj", vjob(w32 | CPX_MUL | XCONJ, 32, 8)),
        ("vec/cpx_mul_yconj", vjob(w32 | CPX_MUL | YCONJ, 32, 8)),
        ("vec/cpx_mac32", vjob(w32 | CPX_MUL | MAC32, 32, 8, z=(RANDOM, 1 << 20, 17))),
        ("vec/cpx_dot", vjob(w32 | CPX_MUL | ACC, 64, 8, z=(ZEROS, 0, 0))),
        ("vec/cpx_qdt", vjob(w32 | CPX_QDT | QDT | QDT_MODE, 64, 4, z=(ZEROS, 0, 0))),
        ("vec/cpx_qdtsum", vjob(w32 | QDT | ACC | QDT_MODE, 64, 4, z=(ZEROS, 0, 0))),
        ("vec/x16_y16_z16_add", vjob(X16 | Y16 | Z16 | ADDSUB, 64, x=(RAW, 0, 3), y=(RAW, 0, 5))),
        ("vec/x16_y32_z32_mul", vjob(X16 | Y32 | Z32 | MUL, 64, 2, x=(RAW, 0, 3))),
        ("vec/x8_y8_z8_add", vjob(X8 | Y8 | Z8 | ADDSUB, 64, x=(RAW, 0, 3), y=(RAW, 0, 5))),
        ("vec/x8_y16_z16_mul", vjob(X8 | Y16 | Z16 | MUL, 64, 0, x=(RAW, 0, 3), y=(RAW, 0, 5))),
        ("vec/loop4_inc", vjob(w32 | MUL, 16, 2, nloop=4, inc=(16, 16, 16))),
        ("vec/loop4_inc0x", vjob(w32 | MUL, 16, 2, nloop=4, inc=(0, 16, 16))),
        ("vec/loop4_dot", vjob(w32 | ACC | MUL, 16, 2, nloop=4, inc=(16, 16, 1), z=(ZEROS, 0, 0))),
        ("vec/sdk_mul_const", vjob(w32 | CONST_MUL | QDT | QDT_MODE, 64, 21, const=1677722, x=(RANDOM, 1 << 24, 3))),
        ("vec/sdk_mul_const_neg", vjob(w32 | CONST_MUL | QDT | QDT_MODE, 64, 4, const=-77777, x=(RANDOM, 1 << 20, 3))),
        ("vec/sdk_mac_const", vjob(w32 | CONST_MUL | MUL, 64, 10, const=-300000, x=(RANDOM, 1 << 28, 3))),
        ("vec/sdk_msc_const_sat", vjob(w32 | CONST_MUL | SUB | MUL, 64, 0, const=0x7FFFFFFF, x=big, y=(RANDOM, 1 << 20, 5))),
        ("vec/sdk_max", vjob(w32 | MAX | CONST_MUL | QDT | QDT_MODE, 64, 3, const=-5000, x=(RANDOM, 1 << 16, 3))),
        ("vec/sdk_min", vjob(w32 | MIN | CONST_MUL | QDT | QDT_MODE, 64, 3, const=5000, x=(RANDOM, 1 << 16, 3))),
        ("vec/sdk_max_q0", vjob(w32 | MAX | CONST_MUL | QDT | QDT_MODE, 64, 0, const=0, x=(RANDOM, 1 << 16, 3))),
        ("vec/qdtsum_z64_small", vjob(X32 | Z64 | QDT | ACC | QDT_MODE, 64, 0, x=(RANDOM, 1 << 28, 9), z=(ZEROS, 0, 0))),
        ("vec/qdtsum_z64_q8", vjob(X32 | Z64 | QDT | ACC | QDT_MODE, 64, 8, x=(RANDOM, 1 << 30, 9), z=(ZEROS, 0, 0))),
        ("vec/cpx_qdt_z64", vjob(X32 | Z64 | CPX_QDT | QDT | QDT_MODE, 64, 0, x=(RANDOM, 1 << 30, 9), z=(ZEROS, 0, 0))),
        ("vec/add_dot_q4", vjob(w32 | ACC | ADDSUB, 64, 4)),
        ("vec/dot_sat", vjob(w32 | ACC | MUL, 64, 0, x=big, y=(RANDOM, 0x7FFFFFFF, 21))),
        ("vec/sub_wrap", vjob(w32 | ADDSUB | SUB, 64, x=big, y=(RANDOM, 0x7FFFFFFF, 21))),
        ("vec/x16_sub_sat", vjob(X16 | Y16 | Z16 | ADDSUB | SUB, 64, x=(RAW, 0, 3), y=(RAW, 0, 5))),
        ("vec/x8_x16_z32_mul_q3", vjob(X8 | Y16 | Z32 | MUL, 64, 3, x=(RAW, 0, 3), y=(RAW, 0, 5))),
        ("vec/x16_mul_z16_sat", vjob(X16 | Y16 | Z16 | MUL, 64, 0, x=(RAW, 0, 3), y=(RAW, 0, 5))),
        ("vec/x32_z16_mul", vjob(X32 | Y32 | Z16 | MUL, 64, 8, x=(RANDOM, 1 << 16, 3), y=(RANDOM, 1 << 16, 5))),
        ("vec/mul_neg_q_sat", vjob(w32 | MUL, 64, -8, x=(RANDOM, 1 << 16, 3), y=(RANDOM, 1 << 16, 5))),
        ("vec/round_neg_half", vjob(w32 | MUL, 64, 1, x=(RANDOM, 3, 3), y=(RANDOM, 3, 5))),
        ("vec/loop3_inc_bytes", vjob(w32 | MUL, 8, 1, nloop=3, inc=(12, 20, 40))),
        ("vec/loop3_x16", vjob(X16 | Y16 | Z32 | MUL, 8, 0, nloop=3, inc=(6, 10, 32), x=(RAW, 0, 3), y=(RAW, 0, 5))),
        *[(f"vec/add_dot_ramp_q{q}", vjob(w32 | ACC | ADDSUB, 16, q, x=(RAMP, 1, 1), y=(RAMP, 2, 3), z=(ZEROS, 0, 0)))
          for q in (0, 1, 2, 4, 8, -2)],
        *[(f"vec/add_dot_sub_q{q}", vjob(w32 | ACC | ADDSUB | SUB, 16, q, x=(RAMP, 1, 1), y=(RAMP, 2, 3), z=(ZEROS, 0, 0)))
          for q in (0, 2)],
        *[(f"vec/qdtsum_z64_ramp_q{q}", vjob(X32 | Z64 | QDT | ACC | QDT_MODE, 16, q, x=(RAMP, 1, 1), z=(ZEROS, 0, 0)))
          for q in (0, 2)],
        ("vec/qdtsum_z32_ramp", vjob(X32 | Z32 | QDT | ACC | QDT_MODE, 16, 0, x=(RAMP, 1, 1), z=(ZEROS, 0, 0))),
        ("vec/cpx_qdt_z64_ramp", vjob(X32 | Z64 | CPX_QDT | QDT | QDT_MODE, 16, 0, x=(RAMP, 1, 1), z=(ZEROS, 0, 0))),
        ("vec/cpx_qdtsum_z64_ramp", vjob(X32 | Z64 | QDT | ACC | QDT_MODE | CPX_QDT, 16, 0, x=(RAMP, 1, 1), z=(ZEROS, 0, 0))),
        ("vec/dot_ramp_q2", vjob(w32 | ACC | MUL, 16, 2, x=(RAMP, 1, 1), y=(RAMP, 2, 3), z=(ZEROS, 0, 0))),
        ("vec/len1", vjob(w32 | MUL, 1, 0)),
        ("vec/len0", vjob(w32 | MUL, 0, 0)),
    ]
    return c


def plan(name):
    if name == "fft":
        return plan_fft()
    if name == "fftfull":
        return plan_fft(True)
    if name == "vec":
        return plan_vec()
    return [("regs", [0])] + plan_fft() + plan_vec()


def load(on):
    p = OUT / f"{on}.json"
    return json.loads(p.read_text()) if p.exists() else {}


def cmd_run(a):
    dock = Path(a.dock).resolve()
    sys.path.insert(0, str(dock / "tools"))
    os.environ.setdefault("FM1_LIVE", str(ROOT / "target" / "release" / "fm1-live"))
    import probe  # fm1-dock's tools/probe.py
    blob_p = dock / "build" / "probes" / "hwdsp.dock"
    for _ in range(5):                          # (the Docker build is flaky on macOS: try again)
        try:
            blob = probe.build(dock / "probes" / "hwdsp.c")
            break
        except SystemExit as e:
            print(f"build: {e}; again", file=sys.stderr)
    else:
        blob = blob_p.read_bytes()
    OUT.mkdir(parents=True, exist_ok=True)
    res = load(a.on)
    cases = [c for c in plan(a.plan) if a.only in c[0]]
    for k, (cid, args) in enumerate(cases):
        if cid in res and not a.again:
            continue
        arg = b"".join(struct.pack("<I", int(x) & 0xFFFFFFFF) for x in args)
        r = probe.run_emu(blob, arg, a.ms) if a.on == "emu" else probe.run_fm1(blob, arg, a.ms, a.port)
        r = {"args": args, **{k2: v for k2, v in r.items() if k2 in ("state", "rc", "result", "crash_pc")}}
        res[cid] = r
        out = r.get("result", {})
        print(f"[{k + 1}/{len(cases)}] {cid}: {r.get('state')} polls {out.get('polls')} ticks {out.get('ticks')}",
              file=sys.stderr)
        (OUT / f"{a.on}.json").write_text(json.dumps(res, indent=0, sort_keys=True))


def words(r, key="out"):
    return [int(v, 16) - ((int(v, 16) & 0x80000000) << 1) for v in r.get("result", {}).get(key, [])]


def cmd_compare(a):
    A, B = load(a.a), load(a.b)
    same = differ = 0
    for cid in sorted(A.keys() & B.keys()):
        ra, rb = A[cid].get("result", {}), B[cid].get("result", {})
        wa, wb = words(A[cid]), words(B[cid])
        bad = [i for i, (x, y) in enumerate(zip(wa, wb)) if x != y]
        # (con0: what the job before left; the emulator boots for every case, the FM-1 does not)
        regs = [k for k in ("con_done", "con", "con_cleared", "x_head", "y_head") if ra.get(k) != rb.get(k)]
        if bad or regs or len(wa) != len(wb):
            differ += 1
            d = max((abs(wa[i] - wb[i]) for i in bad), default=0)
            print(f"{cid}: {len(bad)} of {len(wa)} words differ (largest {d}), first at {bad[:1]}"
                  + (f"; {', '.join(f'{k} {ra.get(k)} / {rb.get(k)}' for k in regs)}" if regs else ""))
        else:
            same += 1
        ta, tb = ra.get("ticks"), rb.get("ticks")
        if ta and tb and a.time:
            print(f"   {cid}: ticks {ta} / {tb}")
    print(f"{same} the same, {differ} differ; only in {a.a}: {len(A.keys() - B.keys())}, "
          f"only in {a.b}: {len(B.keys() - A.keys())}")


def cmd_fit(a):
    import numpy as np
    R = load(a.on)
    for cid in sorted(k for k in R if k.startswith("fft/")):
        r = R[cid]
        args = r["args"]
        con, g, amp, p, off, n = args[1:7]
        nin, nout = fft_words(con)
        x = np.array(gen(g, amp, p, nin), dtype=float)
        N, real, inv = (con >> 16) & 0xFFF, bool(con & 2), bool(con & 4)
        if not real:
            z = x[0::2] + 1j * x[1::2]
            y = np.fft.ifft(z) * N if inv else np.fft.fft(z)
            ideal = np.column_stack([y.real, y.imag]).ravel()
        elif not inv:
            y = np.fft.rfft(x)
            ideal = np.column_stack([y.real, y.imag]).ravel()
        else:
            z = x[0::2] + 1j * x[1::2]
            ideal = np.fft.irfft(z, N) * N
        ideal = ideal[off:off + n]
        hw = np.array(words(r), dtype=float)
        if len(hw) != len(ideal) or not len(hw):
            print(f"{cid}: {len(hw)} words, expected {len(ideal)}")
            continue
        k = float(hw @ ideal / (ideal @ ideal)) if ideal @ ideal else float("nan")
        res = hw - k * ideal
        sat = int(np.sum(np.abs(hw) >= 2 ** 31 - 1))
        print(f"{cid:34s} scale {k:12.6g} (1/{1 / k if k else float('inf'):9.4g})  "
              f"rms err {np.sqrt(np.mean(res ** 2)):10.4g}  max {np.max(np.abs(res)):10.4g}  "
              f"ticks {r['result'].get('ticks')}  sat {sat}")


def cmd_export(a):
    R = load(a.on)
    lines = ["# tools/hwdsp.py export: id | args | ticks | CON when done | out (hex words)"]
    for cid in sorted(R):
        r = R[cid]
        res = r.get("result", {})
        out = " ".join(f"{int(v, 16):08x}" for v in res.get("out", []))
        lines.append(f"{cid} | {','.join(str(x) for x in r['args'])} | {res.get('ticks', '')} | "
                     f"{res.get('con_done', '')} | {out}")
    (OUT / f"{a.on}.txt").write_text("\n".join(lines) + "\n")
    print(f"{OUT / (a.on + '.txt')}: {len(R)} cases")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--on", default="emu", choices=("emu", "fm1"))
    r.add_argument("--plan", default="all", choices=("all", "fft", "fftfull", "vec"))
    r.add_argument("--only", default="")
    r.add_argument("--again", action="store_true", help="run cases already in the file again")
    r.add_argument("--dock", default=str(ROOT.parent / "fm1-harbor"))
    r.add_argument("--port")
    r.add_argument("--ms", type=int, default=3000)
    c = sub.add_parser("compare")
    c.add_argument("a", nargs="?", default="emu")
    c.add_argument("b", nargs="?", default="fm1")
    c.add_argument("--time", action="store_true")
    f = sub.add_parser("fit")
    f.add_argument("on", nargs="?", default="fm1")
    e = sub.add_parser("export")
    e.add_argument("on", nargs="?", default="fm1")
    a = ap.parse_args()
    {"run": cmd_run, "compare": cmd_compare, "fit": cmd_fit, "export": cmd_export}[a.cmd](a)


if __name__ == "__main__":
    main()
