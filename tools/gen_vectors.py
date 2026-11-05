"""
test signals for the fft + the expected output from the bit accurate model

    python gen_vectors.py --out ../build/vec            # directed set, n=256 w=16
    python gen_vectors.py --random 50 --seed 3 --out d  # random set

writes (one line per sample, hex, "re im"):
    in.hex     input samples, all vectors back to back
    exp.hex    expected output bins from fft_model
    sat.hex    1 if the model clipped somewhere in that vector
    names.txt  vector names
"""

import argparse
import os

import numpy as np

import fft_model


def quantize(x, w):
    """complex float (full scale = 1.0) -> two int arrays"""
    lo, hi = -(1 << (w - 1)), (1 << (w - 1)) - 1
    s = 1 << (w - 1)
    re = np.clip(np.round(np.real(x) * s), lo, hi).astype(int)
    im = np.clip(np.round(np.imag(x) * s), lo, hi).astype(int)
    return list(re), list(im)


def tone(n, k, amp, phase=0.0):
    t = np.arange(n)
    return amp * np.exp(1j * (2 * np.pi * k * t / n + phase))


def directed(n, w, seed=1):
    """list of (name, re, im). bins are picked for n=256 and scaled for other sizes"""
    rng = np.random.default_rng(seed)
    t = np.arange(n)
    kb = lambda k: max(1, k * n // 256)
    fs = 1.0 - 1.0 / (1 << (w - 1))     # largest positive value
    lo = -1.0
    v = []

    x = np.zeros(n, complex)
    x[0] = fs
    v.append(("impulse", x))

    x = np.zeros(n, complex)
    x[n // 2 + 1] = 0.7 - 0.3j
    v.append(("impulse_shifted", x))

    v.append(("dc", np.full(n, 0.9 + 0j)))
    v.append(("tone_on_bin", tone(n, kb(10), 0.9)))
    v.append(("tone_real", 0.9 * np.cos(2 * np.pi * kb(37) * t / n) + 0j))
    v.append(("tone_off_bin", tone(n, kb(20) + 0.5, 0.9)))
    v.append(("two_tones", tone(n, kb(17), 0.5) + tone(n, -kb(60), 0.4, 1.0)))
    v.append(("noise", rng.uniform(-0.5, 0.5, n) + 1j * rng.uniform(-0.5, 0.5, n)))

    # full scale. a tone with |x| = 1 sits right at the top, rounding can push it to +1.0 -> clips by 1 lsb
    v.append(("fs_tone", tone(n, kb(3), 1.0)))
    v.append(("fs_nyquist", np.where(t % 2, fs, lo) + 0j))
    v.append(("fs_dc_corner", np.full(n, lo + 1j * lo)))
    # corners of the square (|x| = 1.41): partial sums can go past full scale, these have to clip, not wrap
    v.append(("fs_noise", rng.choice([lo, fs], n) + 1j * rng.choice([lo, fs], n)))
    sq =np.where(np.cos(2 * np.pi * t / n) >= 0, fs, lo) + 1j * np.where(np.sin(2 * np.pi * t / n) >= 0, fs, lo)
    v.append(("fs_square", sq))

    return [(name, *quantize(x, w)) for name, x in v]


def random_vectors(n, w, count, seed):
    """random mixes: noise at random levels, random tones, sparse impulses, full scale corners"""
    rng = np.random.default_rng(seed)
    out = []
    for i in range(count):
        kind = rng.integers(5)
        if kind == 0:
            a = 10 ** rng.uniform(-3, 0)
            x = rng.uniform(-a, a, n) + 1j * rng.uniform(-a, a, n)
            name = "noise"
        elif kind == 1:
            m = rng.integers(1, 5)
            amps = rng.dirichlet(np.ones(m)) * rng.uniform(0.05, 1.0)
            x = sum(tone(n, rng.uniform(0, n), amps[j], rng.uniform(0, 2 * np.pi)) for j in range(m))
            name = f"{m}tones"
        elif kind == 2:
            x = np.zeros(n, complex)
            for _ in range(rng.integers(1, 6)):
                x[rng.integers(n)] = rng.uniform(-1, 1) + 1j * rng.uniform(-1, 1)
            name = "impulses"
        elif kind == 3:
            x = rng.choice([-1.0, 1.0], n) + 1j * rng.choice([-1.0, 1.0], n)
            name = "corners"
        else:
            # raw random ints over the whole range
            lo, hi = -(1 << (w - 1)), (1 << (w - 1))
            re = list(rng.integers(lo, hi, n))
            im = list(rng.integers(lo, hi, n))
            out.append((f"r{i}_fullrange", re, im))
            continue
        out.append((f"r{i}_{name}", *quantize(x, w)))
    return out


def write_set(d, vectors, w, tw):
    """writes the files the testbench reads. returns [(name, re, im, out_re, out_im, nsat)]"""
    os.makedirs(d, exist_ok=True)
    n = len(vectors[0][1])
    rom = fft_model.twiddles(n, tw)
    mask = (1 << w) - 1
    hx = (w + 3) // 4
    res = []
    with open(os.path.join(d, "in.hex"), "w") as fi, \
         open(os.path.join(d, "exp.hex"), "w") as fe, \
         open(os.path.join(d, "sat.hex"), "w") as fs, \
         open(os.path.join(d, "names.txt"), "w") as fn:
        for name, re, im in vectors:
            o_re, o_im, nsat = fft_model.fft_fixed(re, im, w, tw, rom)
            for a, b in zip(re, im):
                fi.write(f"{int(a) & mask:0{hx}x} {int(b) & mask:0{hx}x}\n")
            for a, b in zip(o_re, o_im):
                fe.write(f"{a & mask:0{hx}x} {b & mask:0{hx}x}\n")
            fs.write(f"{int(nsat > 0)}\n")
            fn.write(name + "\n")
            res.append((name, re, im, o_re, o_im, nsat))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=256)
    ap.add_argument("--w", type=int, default=16)
    ap.add_argument("--tw", type=int, default=16)
    ap.add_argument("--random", type=int, default=0, help="random vectors instead of the directed set")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    if a.random:
        vec = random_vectors(a.n, a.w, a.random, a.seed)
    else:
        vec = directed(a.n, a.w, a.seed)
    res = write_set(a.out, vec, a.w, a.tw)
    for name, re, im, o_re, o_im, nsat in res:
        print(f"{name:16s} snr {fft_model.snr_db(o_re, o_im, re, im, a.w):6.1f} dB  sat {nsat}")


if __name__ == "__main__":
    main()
