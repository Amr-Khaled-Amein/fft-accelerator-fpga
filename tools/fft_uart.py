"""
run ffts on the tang nano over uart and check them

    python fft_uart.py --list                  # show com ports
    python fft_uart.py COM4                    # all test signals, compare against numpy + the model
    python fft_uart.py COM4 --random 100       # plus 100 random vectors
    python fft_uart.py COM4 --plot ../docs/spectrum.png
    python fft_uart.py COM4 --signal two_tones --show
    python fft_uart.py --replot ../docs/board_capture.json --plot out.png   # no board needed

for every signal it prints the snr against numpy.fft, the max error, how many bins
are bit exact vs fft_model.py, and the cycle count the board measured.
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import fft_model    # noqa: E402
import gen_vectors  # noqa: E402

N = 256
W = 16
CLK_HZ = 27e6
REPLY_LEN = 4 * N + 8


def s16(v):
    return v - 0x10000 if v & 0x8000 else v


def make_frame(re, im):
    b = bytearray(b"F")
    for a, c in zip(re, im):
        b += (int(a) & 0xffff).to_bytes(2, "big") + (int(c) & 0xffff).to_bytes(2, "big")
    return bytes(b)


def parse_reply(b):
    """reply bytes -> dict, raises ValueError if it's broken"""
    if len(b) != REPLY_LEN:
        raise ValueError(f"reply is {len(b)} bytes, want {REPLY_LEN}")
    if b[0] != 0xA5 or b[1] != 0x5A:
        raise ValueError(f"bad header {b[0]:02x} {b[1]:02x}")
    cs = 0
    for x in b[2:-1]:
        cs ^= x
    if cs != b[-1]:
        raise ValueError("bad checksum")
    re = [s16(int.from_bytes(b[2 + 4 * k:4 + 4 * k], "big")) for k in range(N)]
    im = [s16(int.from_bytes(b[4 + 4 * k:6 + 4 * k], "big")) for k in range(N)]
    return {
        "re": re,
        "im": im,
        "cycles": int.from_bytes(b[4 * N + 2:4 * N + 6], "big"),
        "clip": bool(b[4 * N + 6] & 1),
    }


def run_one(ser, re, im, timeout=3.0):
    ser.reset_input_buffer()
    ser.write(make_frame(re, im))
    buf = bytearray()
    t0 = time.time()
    while len(buf) < REPLY_LEN and time.time() - t0 < timeout:
        buf += ser.read(REPLY_LEN - len(buf))
    return parse_reply(bytes(buf))


def check(name, re, im, rep):
    m_re, m_im, nsat = fft_model.fft_fixed(re, im, W, 16)
    exact = sum(1 for k in range(N) if rep["re"][k] == m_re[k] and rep["im"][k] == m_im[k])
    return {
        "name": name,
        "exact": exact,
        "snr": fft_model.snr_db(rep["re"], rep["im"], re, im, W),
        "maxerr": fft_model.max_err_lsb(rep["re"], rep["im"], re, im, W),
        "cycles": rep["cycles"],
        "clip": rep["clip"],
        "clip_model": nsat > 0,
    }


def spectrum_db(re, im):
    """board output (= fft / N) back to the real fft size, in dB full scale"""
    x = fft_model.to_complex(re, im, W) * N
    return 20 * np.log10(np.maximum(np.abs(x), 1e-9) / N)


def plot(caps, path, show):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    bg, ink, ink2, grid = "#fcfcfb", "#1f1f1e", "#6b6a64", "#e6e5e0"
    c_np, c_hw = "#eb6834", "#2a78d6"
    plt.rcParams.update({"font.size": 9, "text.color": ink, "axes.labelcolor": ink2,
                         "xtick.color": ink2, "ytick.color": ink2, "axes.edgecolor": grid})
    fig, axes = plt.subplots(len(caps), 1, figsize=(9, 2.7 * len(caps)), squeeze=False, facecolor=bg)
    k = np.arange(N)
    for ax, c in zip(axes[:, 0], caps):
        ax.set_facecolor(bg)
        ref = np.fft.fft(fft_model.to_complex(c["in_re"], c["in_im"], W)) / N
        ref_db = 20 * np.log10(np.maximum(np.abs(ref), 1e-9))
        ax.plot(k, ref_db, color=c_np, lw=2, label="numpy.fft (float)", zorder=2)
        ax.plot(k, spectrum_db(c["out_re"], c["out_im"]), "o", color=c_hw, ms=3.2, mew=0.6,
                mec=bg, label="fpga (16 bit fixed point)", zorder=3)
        ax.set_xlim(0, N - 1)
        ax.set_ylim(-115, 5)
        ax.set_ylabel("dBFS")
        ax.set_title(f"{c['name']}:  snr {c['snr']:.1f} dB vs numpy,  {c['exact']}/{N} bins "
                     f"bit exact vs model,  {c['cycles']} cycles", fontsize=9.5, loc="left", color=ink)
        ax.grid(color=grid, lw=0.6)
        ax.set_axisbelow(True)
        ax.spines[["top", "right"]].set_visible(False)
    axes[0, 0].legend(loc="upper right", frameon=False, fontsize=8.5)
    axes[-1, 0].set_xlabel("bin")
    fig.tight_layout()
    if path:
        fig.savefig(path, dpi=130, facecolor=bg)
        print(f"plot -> {path}")
    if show:
        plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--signal", default="all", help="one of the test signal names, or all")
    ap.add_argument("--random", type=int, default=0, help="also run this many random vectors")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--plot", help="save the spectrum plot here")
    ap.add_argument("--plot-signals", default="tone_off_bin,two_tones,noise")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--save", help="save everything (inputs + board outputs) as json")
    ap.add_argument("--replot", help="no board: redraw the plot from a json made with --save")
    a = ap.parse_args()

    if a.replot:
        with open(a.replot) as f:
            caps = json.load(f)
        want = a.plot_signals.split(",")
        plot([c for c in caps if c["name"] in want], a.plot, a.show or not a.plot)
        return

    import serial
    if a.list or not a.port:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(p.device, "-", p.description)
        return

    vec = gen_vectors.directed(N, W)
    if a.signal != "all":
        vec = [v for v in vec if v[0] == a.signal]
        if not vec:
            sys.exit(f"no signal called {a.signal}")
    if a.random:
        vec += gen_vectors.random_vectors(N, W, a.random, a.seed)

    ser = serial.Serial(a.port, 115200, timeout=0.5)
    time.sleep(0.1)
    ser.reset_input_buffer()

    rows, caps = [], []
    t_start = time.time()
    print(f"{'signal':18s} {'snr dB':>7s} {'max err':>8s} {'bit exact':>10s} {'cycles':>7s}  clip")
    for name, re, im in vec:
        try:
            rep = run_one(ser, re, im)
        except ValueError as e:
            print(f"{name:18s} NO REPLY ({e})")
            rows.append({"name": name, "exact": 0, "snr": float("nan"), "maxerr": float("nan"),
                         "cycles": 0, "clip": None, "clip_model": None})
            continue
        r = check(name, re, im, rep)
        rows.append(r)
        caps.append({"name": name, "in_re": [int(x) for x in re], "in_im": [int(x) for x in im],
                     "out_re": rep["re"], "out_im": rep["im"], "cycles": rep["cycles"],
                     "clip": rep["clip"], "snr": r["snr"], "exact": r["exact"]})
        if not name.startswith("r") or a.signal != "all" or r["exact"] != N:
            clip = "yes" if r["clip"] else "-"
            if r["clip"] != r["clip_model"]:
                clip += " (model disagrees!)"
            snr = "inf" if math.isinf(r["snr"]) else f"{r['snr']:.1f}"
            print(f"{name:18s} {snr:>7s} {r['maxerr']:8.2f} {r['exact']:6d}/{N} {r['cycles']:7d}  {clip}")
    dt = time.time() - t_start
    ser.close()

    good = [r for r in rows if r["exact"] == N and r["clip"] == r["clip_model"]]
    cyc = sorted({r["cycles"] for r in rows if r["cycles"]})
    print(f"\n{len(good)}/{len(rows)} vectors bit exact vs fft_model "
          f"({sum(r['exact'] for r in rows)}/{len(rows) * N} bins), {dt:.1f} s")
    if cyc:
        print(f"cycles per fft (measured on the board): {', '.join(map(str, cyc))}"
              f"  = {cyc[0] / CLK_HZ * 1e6:.2f} us at 27 MHz")

    if a.save:
        with open(a.save, "w") as f:
            json.dump(caps, f)
        print(f"saved {len(caps)} captures -> {a.save}")
    if a.plot or a.show:
        want = a.plot_signals.split(",")
        sel = [c for c in caps if c["name"] in want] or caps[:3]
        plot(sel, a.plot, a.show)
    sys.exit(0 if len(good) == len(rows) else 1)


if __name__ == "__main__":
    main()
