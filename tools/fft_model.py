"""
bit accurate model of the fft in the rtl, plus helpers to compare against numpy

same algorithm as fft_core.v: radix-2 dit, input stored bit reversed, then
log2(n) stages of n/2 butterflies, in place. every butterfly does

    t  = b * (c - j*s)                      full precision
    y0 = sat((a * 2^(tw-1) + t + 2^(tw-1)) >> tw)
    y1 = sat((a * 2^(tw-1) - t + 2^(tw-1)) >> tw)

so it scales by 1/2 per stage, rounds to nearest, and clips. the output is
fft(x) / n.

    python fft_model.py          # quick self check against numpy
"""

import math
import numpy as np

from gen_twiddle import twiddles


def bitrev(x, bits):
    r = 0
    for _ in range(bits):
        r = (r << 1) | (x & 1)
        x >>= 1
    return r


def sat(v, w):
    lo, hi = -(1 << (w - 1)), (1 << (w - 1)) - 1
    if v < lo:
        return lo, True
    if v > hi:
        return hi, True
    return v, False


def butterfly(ar, ai, br, bi, c, s, w, tw):
    """one butterfly, returns (y0r, y0i, y1r, y1i, saturated)"""
    tr = br * c + bi * s
    ti = bi * c - br * s
    half = 1 << (tw - 1)
    out = []
    clipped = False
    for v in ((ar << (tw - 1)) + tr, (ai << (tw - 1)) + ti,
              (ar << (tw - 1)) - tr, (ai << (tw - 1)) - ti):
        q, hit = sat((v + half) >> tw, w)
        out.append(q)
        clipped |= hit
    return out[0], out[1], out[2], out[3], clipped


def butterfly_order(n):
    """(stage, a, b, k) for every butterfly in the order the rtl issues them"""
    logn = n.bit_length() - 1
    for st in range(logn):
        span = 1 << st
        for bf in range(n // 2):
            pos = bf & (span - 1)
            a = ((bf >> st) << (st + 1)) | pos
            yield st, a, a + span, pos << (logn - 1 - st)


def hazards(n, bubbles, latency=4):
    """
    how many butterfly reads would see a stale value if every stage boundary
    had 'bubbles' idle clocks. a result written for a butterfly issued at clock t
    is only readable by butterflies issued after t + latency.
    """
    t, cur, last_w, bad = 0, -1, {}, 0
    for st, a, b, _ in butterfly_order(n):
        if st != cur:
            if cur >= 0:
                t += bubbles
            cur = st
        for x in (a, b):
            if x in last_w and not last_w[x] + latency < t:
                bad += 1
        last_w[a] = last_w[b] = t
        t += 1
    return bad


def fft_fixed(re, im, w=16, tw=16, rom=None):
    """
    re, im: lists of signed ints (w bits). returns (out_re, out_im, n_saturations)
    out is in natural order, = fft / n
    """
    n = len(re)
    logn = n.bit_length() - 1
    assert 1 << logn == n and len(im) == n
    if rom is None:
        rom = twiddles(n, tw)
    xr = [0] * n
    xi = [0] * n
    for k in range(n):
        xr[bitrev(k, logn)] = int(re[k])
        xi[bitrev(k, logn)] = int(im[k])
    nsat = 0
    for st, a, b, k in butterfly_order(n):
        c, s = rom[k]
        y0r, y0i, y1r, y1i, hit = butterfly(xr[a], xi[a], xr[b], xi[b], c, s, w, tw)
        xr[a], xi[a], xr[b], xi[b] = y0r, y0i, y1r, y1i
        nsat += hit
    return xr, xi, nsat


# ------------------------------------------------------------ float side

def to_complex(re, im, w=16):
    return (np.asarray(re, float) + 1j * np.asarray(im, float)) / (1 << (w - 1))


def numpy_ref(re, im, w=16):
    """numpy fft of the same (already quantized) input, scaled by 1/n like the hw"""
    x = to_complex(re, im, w)
    return np.fft.fft(x) / len(x)


def snr_db(out_re, out_im, re, im, w=16):
    """snr of the fixed point output against numpy, in dB"""
    ref = numpy_ref(re, im, w)
    got = to_complex(out_re, out_im, w)
    err = np.sum(np.abs(got - ref) ** 2)
    sig = np.sum(np.abs(ref) ** 2)
    if err == 0:
        return math.inf
    return 10 * math.log10(sig / err)


def max_err_lsb(out_re, out_im, re, im, w=16):
    """largest error of any bin (re or im) vs numpy, in output lsbs"""
    ref = numpy_ref(re, im, w) * (1 << (w - 1))
    e = np.maximum(np.abs(np.asarray(out_re) - ref.real), np.abs(np.asarray(out_im) - ref.imag))
    return float(np.max(e))


if __name__ == "__main__":
    rng = np.random.default_rng(1)
    for n in (8, 64, 256, 1024):
        re = rng.integers(-16000, 16000, n)
        im = rng.integers(-16000, 16000, n)
        o_re, o_im, ns = fft_fixed(re, im)
        print(f"n={n:5d}  random noise  snr {snr_db(o_re, o_im, re, im):6.1f} dB  "
              f"max err {max_err_lsb(o_re, o_im, re, im):.2f} lsb  sat {ns}")
    t = np.arange(256)
    re = np.round(30000 * np.cos(2 * np.pi * 10 * t / 256)).astype(int)
    im = np.round(30000 * np.sin(2 * np.pi * 10 * t / 256)).astype(int)
    o_re, o_im, ns = fft_fixed(re, im)
    print(f"tone in bin 10: out[10] = {o_re[10]} {o_im[10]}j (expect ~30000), "
          f"snr {snr_db(o_re, o_im, re, im):.1f} dB")
