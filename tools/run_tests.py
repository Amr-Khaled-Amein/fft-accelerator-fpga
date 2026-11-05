"""
regression: python model -> test vectors -> iverilog -> compare

    python run_tests.py                 # everything
    python run_tests.py --random 500    # more random vectors (default 100)
    python run_tests.py --no-top        # skip the slow uart test
    python run_tests.py --no-mutants    # skip the broken design check
    python run_tests.py --seed 7

steps:
  1. twiddle generator: every entry within 1/2 lsb of the real value (or saturated)
  2. the model itself against numpy (so matching it actually means something)
  3. rtl, n=256 16 bit: directed signals + random vectors, bit exact vs the model
  4. rtl at other sizes and widths, each with its own generated rom
  5. whole design over uart (tb_top)
  6. mutants: break the rtl on purpose, the tests above have to catch every one

at the end it rewrites fft/src/twiddle_rom.v for n=256 / 16 bit so the gowin
project always has the right one.
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
SRC = os.path.join(ROOT, "fft", "src")
SIM = os.path.join(ROOT, "sim")
BUILD = os.path.join(ROOT, "build")
sys.path.insert(0, HERE)

import fft_model    # noqa: E402
import fft_uart     # noqa: E402
import gen_twiddle  # noqa: E402
import gen_vectors  # noqa: E402

CORE = ["fft_core.v", "sample_ram.v"]
TOP = CORE + ["top.v", "uart_tx.v", "uart_rx.v"]

# (n, data width, twiddle width) for the size / width sweep
SWEEP = [(4, 16, 16), (8, 16, 16), (16, 12, 12), (32, 16, 10), (64, 18, 18),
         (128, 10, 16), (512, 24, 20), (1024, 16, 16), (2048, 14, 14)]


def sh(cmd, cwd):
    r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout)
        print(r.stderr)
        raise RuntimeError(f"command failed: {' '.join(cmd)}")
    return r.stdout + r.stderr


def build_dir(name):
    d = os.path.join(BUILD, name)
    os.makedirs(d, exist_ok=True)
    return d


def run_fft_sim(d, vectors, n=256, w=16, tw=16, src=SRC):
    """vectors -> files -> tb_fft. returns (ok, rtl output text, list of result rows)"""
    model = gen_vectors.write_set(d, vectors, w, tw)
    gen_twiddle.write_rom(os.path.join(d, "twiddle_rom.v"), n, tw)
    files = [os.path.join(src, f) for f in CORE] + [os.path.join(d, "twiddle_rom.v"), os.path.join(SIM, "tb_fft.sv")]
    sh(["iverilog", "-g2012", "-Wall", "-Wno-timescale", "-o", "sim.vvp",
        f"-Ptb_fft.N={n}", f"-Ptb_fft.W={w}", f"-Ptb_fft.TW={tw}"] + files, d)
    out = sh(["vvp", "-n", "sim.vvp"], d)
    rows = []
    res = os.path.join(d, "results.txt")
    if os.path.exists(res):
        with open(res) as f:
            for line, m in zip(f, model):
                name, nbad, cyc, sat = line.split()
                rows.append({"name": name, "bad": int(nbad), "cycles": int(cyc), "sat": int(sat),
                             "snr": fft_model.snr_db(m[3], m[4], m[1], m[2], w),
                             "maxerr": fft_model.max_err_lsb(m[3], m[4], m[1], m[2], w),
                             "model_sat": m[5]})
    ok = "PASS:" in out and len(rows) == len(vectors) and not any(r["bad"] for r in rows)
    return ok, out, rows


# ------------------------------------------------------------------ 1 + 2: python side

def check_twiddles():
    bad = 0
    for n in (4, 8, 16, 64, 256, 1024, 4096):
        for w in (8, 10, 12, 16, 18, 24):
            hi = (1 << (w - 1)) - 1
            for k, (c, s) in enumerate(gen_twiddle.twiddles(n, w)):
                for q, v in ((c, math.cos(2 * math.pi * k / n)), (s, math.sin(2 * math.pi * k / n))):
                    ideal = v * (1 << (w - 1))
                    if not (q == hi and ideal >= hi) and abs(q - ideal) > 0.5:
                        bad += 1
                    if not -(1 << (w - 1)) <= q <= hi:
                        bad += 1
    return bad == 0


def check_schedule():
    """fft_core.v runs stages back to back when N >= 32 (OVERLAP), else it drains 4 clocks"""
    errs = []
    for n in (4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096):
        bubbles = 0 if n >= 32 else 4
        h = fft_model.hazards(n, bubbles)
        if h:
            errs.append(f"n={n}: {h} stale reads with {bubbles} bubbles between stages")
    # and the drain really is needed below 32, otherwise the rule is wrong the other way
    if not fft_model.hazards(16, 0) or not fft_model.hazards(8, 2):
        errs.append("expected hazards for small n without a full drain")
    return errs


def check_model():
    """the model has to be close to numpy whenever nothing clips"""
    errs = []
    for n, w in ((16, 16), (256, 16), (256, 12), (1024, 18)):
        for name, re, im in gen_vectors.directed(n, w) + gen_vectors.random_vectors(n, w, 30, 5):
            o_re, o_im, nsat = fft_model.fft_fixed(re, im, w, w)
            if nsat == 0:
                e = fft_model.max_err_lsb(o_re, o_im, re, im, w)
                if e > 3.0:
                    errs.append(f"n={n} w={w} {name}: model is {e:.2f} lsb off numpy")
    return errs


# ------------------------------------------------------------------ 5: uart

def run_top_sim(src=SRC):
    d = build_dir("top")
    v = {name: (name, re, im) for name, re, im in gen_vectors.directed(256, 16)}
    vec = [v["tone_off_bin"], v["fs_square"], v["noise"]]
    model = gen_vectors.write_set(d, vec, 16, 16)
    gen_twiddle.write_rom(os.path.join(d, "twiddle_rom.v"), 256, 16)
    files = [os.path.join(src, f) for f in TOP] + [os.path.join(d, "twiddle_rom.v"), os.path.join(SIM, "tb_top.sv")]
    sh(["iverilog", "-g2012", "-Wall", "-Wno-timescale", "-o", "sim.vvp"] + files, d)
    t0 = time.time()
    out = sh(["vvp", "-n", "sim.vvp"], d)
    dt = time.time() - t0

    # parse what came back with the same code the board script uses
    errs = []
    if "PASS:" not in out:
        errs.append("tb_top says FAIL")
        errs += [l.strip() for l in out.splitlines() if "FAIL" in l or "    " in l][:6]
    with open(os.path.join(d, "uart.out")) as f:
        data = bytes(int(x, 16) for x in f.read().split())
    L = fft_uart.REPLY_LEN
    if len(data) != L * len(vec):
        errs.append(f"got {len(data)} bytes, want {L * len(vec)}")
    else:
        for i, m in enumerate(model):
            try:
                rep = fft_uart.parse_reply(data[i * L:(i + 1) * L])
            except ValueError as e:
                errs.append(f"{m[0]}: {e}")
                continue
            if rep["re"] != m[3] or rep["im"] != m[4] or rep["clip"] != (m[5] > 0):
                errs.append(f"{m[0]}: fft_uart.parse_reply doesn't match the model")
    return errs, dt


# ------------------------------------------------------------------ 6: mutants

# (what it breaks, file, old text, new text). old text has to be in the file
MUTANTS = [
    ("truncate instead of round", "fft_core.v", "+ half;", "+ 0;"),
    ("wrong sign in the twiddle multiply", "fft_core.v", "ti1  <= p_ic - p_rs;", "ti1  <= p_ic + p_rs;"),
    ("no 1/2 scale on one output", "fft_core.v", "y0r  <= clip(s0r >>> TW);", "y0r  <= clip(s0r >>> (TW-1));"),
    ("no saturation, wraps around", "fft_core.v",
     "if (v > $signed({1'b0, {(W-1){1'b1}}}))\n            clip = {1'b0, {(W-1){1'b1}}};",
     "if (1'b0)\n            clip = 0;"),
    ("clip flag never set", "fft_core.v", "sat <= 1'b1;", "sat <= 1'b0;"),
    ("no bit reversal on load", "fft_core.v", "wire [LOGN-1:0] in_br = bitrev(in_addr);",
     "wire [LOGN-1:0] in_br = in_addr;"),
    ("a and b swapped on read", "fft_core.v", "wire signed [W-1:0] ar = sw0 ?", "wire signed [W-1:0] ar = !sw0 ?"),
    ("write mux uses the wrong pipeline stage", "fft_core.v",
     "wd0 = sw3 ? {y1r, y1i} : {y0r, y0i};", "wd0 = sw2 ? {y1r, y1i} : {y0r, y0i};"),
    ("bank address delayed one stage too few", "fft_core.v", "wa0 = a0_3;", "wa0 = a0_2;"),
    ("drain between stages too short", "fft_core.v", "end else if (!(v0 | v1 | v2)) begin", "end else if (!v0) begin"),
    ("stages overlap for small n too", "fft_core.v", "OVERLAP = (N >= 32);", "OVERLAP = (N >= 4);"),
    ("one stage missing", "fft_core.v", "if (stage == LOGN - 1) begin", "if (stage == LOGN - 2) begin"),
    ("twiddle index off by one", "fft_core.v", "tw_k    = (bf", "tw_k    = 1'b1 + (bf"),
    ("one twiddle entry 1 lsb off", "rom", None, None),
]


def run_mutants(rand_vec):
    """every mutant has to fail at least one of: n=256 directed+random, n=16 / n=8 directed"""
    suites = [(256, gen_vectors.directed(256, 16) + rand_vec[:20]),
              (16, gen_vectors.directed(16, 16)), (8, gen_vectors.directed(8, 16))]
    results = []
    for name, fname, old, new in MUTANTS:
        d = build_dir("mutant")
        msrc = os.path.join(d, "src")
        os.makedirs(msrc, exist_ok=True)
        for f in CORE:
            shutil.copy(os.path.join(SRC, f), msrc)
        if fname == "rom":
            caught = False
            for n, vec in suites:
                gen_vectors.write_set(d, vec, 16, 16)
                text = gen_twiddle.rom_verilog(n, 16)
                # bump the cos of entry 3 by one lsb
                k = 3
                c, s = gen_twiddle.twiddles(n, 16)[k]
                aw = max(1, (n // 2 - 1).bit_length())
                a = f"{aw}'d{k}: d <= {{16'h{c & 0xffff:04x}, 16'h{s & 0xffff:04x}}};"
                b = f"{aw}'d{k}: d <= {{16'h{(c - 1) & 0xffff:04x}, 16'h{s & 0xffff:04x}}};"
                assert a in text
                with open(os.path.join(d, "twiddle_rom.v"), "w") as f:
                    f.write(text.replace(a, b))
                files = [os.path.join(msrc, f) for f in CORE] + [os.path.join(d, "twiddle_rom.v"),
                                                                 os.path.join(SIM, "tb_fft.sv")]
                sh(["iverilog", "-g2012", "-Wno-timescale", "-o", "sim.vvp", f"-Ptb_fft.N={n}"] + files, d)
                out = sh(["vvp", "-n", "sim.vvp"], d)
                if "PASS:" not in out:
                    caught = True
                    break
            results.append((name, caught))
            continue

        p = os.path.join(msrc, fname)
        with open(p) as f:
            text = f.read()
        if text.count(old) < 1:
            raise RuntimeError(f"mutant '{name}': text not found in {fname}")
        with open(p, "w") as f:
            f.write(text.replace(old, new))
        caught = False
        for n, vec in suites:
            # a mutant that doesn't compile is a broken mutant, not a catch -> sh() raises
            ok, _, _ = run_fft_sim(d, vec, n, 16, 16, src=msrc)
            if not ok:
                caught = True
                break
        results.append((name, caught))
    return results


# ------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--random", type=int, default=100)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--no-top", action="store_true")
    ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--no-mutants", action="store_true")
    a = ap.parse_args()

    if not shutil.which("iverilog"):
        print("iverilog not on PATH")
        sys.exit(1)

    fails = 0
    summary = {}
    t_all = time.time()

    ok = check_twiddles()
    print("twiddle generator (n 4..4096, width 8..24) ...", "ok" if ok else "FAIL")
    fails += not ok
    errs = check_schedule()
    print("pipeline schedule, no stale reads for any n ...", "ok" if not errs else "FAIL")
    for e in errs[:5]:
        print("     ", e)
    fails += bool(errs)
    errs = check_model()
    print("model vs numpy, nothing clipped -> max error <= 3 lsb ...", "ok" if not errs else "FAIL")
    for e in errs[:5]:
        print("     ", e)
    fails += bool(errs)

    print("\nrtl n=256, 16 bit, directed signals:")
    print(f"  {'signal':18s} {'result':6s} {'cycles':>6s} {'snr dB':>7s} {'max err':>8s}  clip")
    ok, out, rows = run_fft_sim(build_dir("directed"), gen_vectors.directed(256, 16))
    for r in rows:
        snr = "inf" if math.isinf(r["snr"]) else f"{r['snr']:.1f}"
        print(f"  {r['name']:18s} {'ok' if not r['bad'] else 'FAIL':6s} {r['cycles']:6d} {snr:>7s} "
              f"{r['maxerr']:8.2f}  {'yes' if r['sat'] else '-'}")
    print("  " + [l for l in out.splitlines() if l.startswith(("PASS", "FAIL"))][-1])
    summary["directed"] = rows
    fails += not ok

    rand_vec = []
    if a.random:
        rand_vec = gen_vectors.random_vectors(256, 16, a.random, a.seed)
        ok, out, rows = run_fft_sim(build_dir("random"), rand_vec)
        print(f"\nrtl n=256, {a.random} random vectors (seed {a.seed}):")
        for r in rows:
            if r["bad"]:
                print(f"  FAIL {r['name']} ({r['bad']} bad bins)")
        nclip = sum(1 for r in rows if r["sat"])
        print("  " + [l for l in out.splitlines() if l.startswith(("PASS", "FAIL"))][-1] +
              f"  ({nclip} of them clipped)")
        summary["random"] = {"count": a.random, "ok": ok}
        fails += not ok

    if not a.no_sweep:
        print("\nother sizes / widths (each with its own generated rom):")
        summary["sweep"] = []
        for n, w, tw in SWEEP:
            vec = gen_vectors.directed(n, w) + gen_vectors.random_vectors(n, w, 20, a.seed + n)
            ok, out, rows = run_fft_sim(build_dir(f"n{n}_w{w}_tw{tw}"), vec, n, w, tw)
            cyc = rows[0]["cycles"] if rows else 0
            print(f"  n={n:<5d} w={w:<3d} tw={tw:<3d} {'PASS' if ok else 'FAIL'}  "
                  f"{len(vec)} vectors  {cyc} cycles")
            summary["sweep"].append({"n": n, "w": w, "tw": tw, "ok": ok, "vectors": len(vec), "cycles": cyc})
            fails += not ok

    if not a.no_top:
        print("\nwhole design over uart (3 frames, fast baud) ...")
        errs, dt = run_top_sim()
        print(f"  {'PASS' if not errs else 'FAIL'}  ({dt:.1f}s)")
        for e in errs:
            print("     ", e)
        fails += bool(errs)

    if not a.no_mutants:
        print("\nmutants (broken on purpose, each one has to be caught):")
        res = run_mutants(rand_vec or gen_vectors.random_vectors(256, 16, 20, a.seed))
        for name, caught in res:
            print(f"  {'caught' if caught else 'MISSED':7s} {name}")
        missed = sum(1 for _, c in res if not c)
        print(f"  {len(res) - missed}/{len(res)} caught")
        summary["mutants"] = {"total": len(res), "caught": len(res) - missed}
        fails += missed

    gen_twiddle.write_rom(os.path.join(SRC, "twiddle_rom.v"), 256, 16)
    print("\nfft/src/twiddle_rom.v <- n=256, 16 bit")
    with open(os.path.join(BUILD, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1, default=lambda x: None if isinstance(x, float) and math.isinf(x) else x)

    print(f"\nALL PASS ({time.time() - t_all:.0f}s)" if not fails else f"\n{fails} FAILED")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
