"""
writes the twiddle rom as plain verilog (a registered case statement)

    python gen_twiddle.py --n 256 --width 16                 # -> ../fft/src/twiddle_rom.v
    python gen_twiddle.py --n 1024 --width 18 -o rom.v

entry k (0 .. n/2-1) holds cos and sin of 2*pi*k/n in signed fixed point with
width-1 fraction bits. values are rounded to nearest and saturated, so cos(0) = 1.0
becomes 0x7fff for 16 bits. the fft uses W = cos - j*sin.
"""

import argparse
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.normpath(os.path.join(HERE, "..", "fft", "src", "twiddle_rom.v"))


def quant(v, width):
    """round to nearest (half goes up) and clip to the signed range"""
    q = math.floor(v * (1 << (width - 1)) + 0.5)
    lo, hi = -(1 << (width - 1)), (1 << (width - 1)) - 1
    return max(lo, min(hi, q))


def twiddles(n, width):
    """list of (cos, sin) ints for k = 0 .. n/2-1"""
    if n < 4 or n & (n - 1):
        raise ValueError("n has to be a power of 2, at least 4")
    if width < 4:
        raise ValueError("width too small")
    return [(quant(math.cos(2 * math.pi * k / n), width),
             quant(math.sin(2 * math.pi * k / n), width)) for k in range(n // 2)]


def rom_verilog(n, width):
    tw = twiddles(n, width)
    aw = max(1, (n // 2 - 1).bit_length())
    mask = (1 << width) - 1
    hx = (width + 3) // 4
    lines = [
        f"// twiddle rom for n={n}, width={width}. made by tools/gen_twiddle.py, don't edit",
        f"// entry k = {{cos, sin}} of 2*pi*k/{n}, signed with {width - 1} fraction bits",
        "",
        "module twiddle_rom #(",
        f"    parameter N  = {n},     // only here so the testbench can check it, don't override",
        f"    parameter TW = {width}",
        ")(",
        "    input  wire                 clk,",
        f"    input  wire [{aw - 1}:0]          addr,",
        "    output wire signed [TW-1:0] cos_o,",
        "    output wire signed [TW-1:0] sin_o",
        ");",
        "    reg [2*TW-1:0] d;",
        "    assign cos_o = d[2*TW-1:TW];",
        "    assign sin_o = d[TW-1:0];",
        "",
        "    always @(posedge clk) begin",
        "        case (addr)",
    ]
    for k, (c, s) in enumerate(tw):
        lines.append(f"            {aw}'d{k}: d <= {{{width}'h{c & mask:0{hx}x}, {width}'h{s & mask:0{hx}x}}};")
    if (1 << aw) != n // 2:
        lines.append(f"            default: d <= {2 * width}'d0;")
    lines += [
        "        endcase",
        "    end",
        "endmodule",
        "",
    ]
    return "\n".join(lines)


def write_rom(path, n, width):
    with open(path, "w", newline="\n") as f:
        f.write(rom_verilog(n, width))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=256)
    ap.add_argument("--width", type=int, default=16)
    ap.add_argument("-o", "--out", default=DEFAULT_OUT)
    a = ap.parse_args()
    write_rom(a.out, a.n, a.width)
    print(f"wrote {a.out}  ({a.n // 2} entries, {a.width} bit)")


if __name__ == "__main__":
    main()
