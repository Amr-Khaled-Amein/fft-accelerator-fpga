// in-place radix-2 dit fft, one butterfly unit reused for every stage
//
// samples go in through the load port in natural order and get stored at the
// bit reversed address. start runs log2(N) stages of N/2 butterflies, then the
// bins can be read back in natural order. output = fft(x) / N (1/2 per stage).
//
// memory is split in 2 banks by the parity of the address bits. the two points
// of a butterfly always differ in exactly one bit, so they're always in different
// banks -> both can be read and written every clock with simple dual port rams.
//
// butterfly pipeline (one butterfly per clock):
//   rd   address out to the rams and twiddle rom (both sync read)
//   s1   t = b * (c - js)                   complex multiply, full precision
//   s2   a*2^(TW-1) + t, a*2^(TW-1) - t     add / sub
//   s3   round, >> TW, saturate             -> written back in the next clock
// a result is written 4 clocks after its butterfly was issued. for N >= 32 the
// first butterflies of a stage never touch what the last ones of the previous
// stage are still working on, so the next stage starts right away. for small N
// the pipeline drains between stages instead. (fft_model.hazards() checks this)

module fft_core #(
    parameter N    = 256,
    parameter W    = 16,          // data width of re and im
    parameter TW   = 16,          // twiddle width, has to match twiddle_rom.v
    parameter LOGN = $clog2(N)
)(
    input  wire            clk,
    input  wire            rst,

    // load, only while idle
    input  wire            in_we,
    input  wire [LOGN-1:0] in_addr,
    input  wire [W-1:0]    in_re,
    input  wire [W-1:0]    in_im,

    // read back, data is there 1 clock after out_addr
    input  wire [LOGN-1:0] out_addr,
    output wire [W-1:0]    out_re,
    output wire [W-1:0]    out_im,

    input  wire            start,
    output wire            busy,
    output reg             done,      // 1 clock pulse at the end
    output reg             sat        // something got clipped in the last run
);
    localparam AW = LOGN - 1;         // bank address width
    localparam SW = W + TW + 2;       // width of the sums before rounding
    localparam SB = $clog2(LOGN);     // stage counter width
    localparam OVERLAP = (N >= 32);   // stages can run back to back, see above

    function [LOGN-1:0] bitrev;
        input [LOGN-1:0] x;
        integer i;
        for (i = 0; i < LOGN; i = i + 1)
            bitrev[i] = x[LOGN-1-i];
    endfunction

    // clip a rounded sum down to W bits
    function [W-1:0] clip;
        input signed [SW-TW-1:0] v;
        if (v > $signed({1'b0, {(W-1){1'b1}}}))
            clip = {1'b0, {(W-1){1'b1}}};
        else if (v < $signed({1'b1, {(W-1){1'b0}}}))
            clip = {1'b1, {(W-1){1'b0}}};
        else
            clip = v[W-1:0];
    endfunction

    function clipped;
        input signed [SW-TW-1:0] v;
        clipped = (v > $signed({1'b0, {(W-1){1'b1}}})) || (v < $signed({1'b1, {(W-1){1'b0}}}));
    endfunction

    // ------------------------------------------------------------ control

    localparam IDLE = 2'd0, RUN = 2'd1, DRAIN = 2'd2;
    reg [1:0]    state;
    reg [SB-1:0] stage;
    reg [AW-1:0] bf;                  // butterfly number inside the stage

    reg v0, v1, v2, v3;               // valid bits down the pipeline

    assign busy = (state != IDLE);

    always @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin
            state <= IDLE;
            stage <= 0;
            bf    <= 0;
        end else begin
            case (state)
                IDLE: if (start) begin
                    state <= RUN;
                    stage <= 0;
                    bf    <= 0;
                end
                RUN: begin
                    bf <= bf + 1'b1;
                    if (bf == {AW{1'b1}}) begin
                        if (OVERLAP && stage != LOGN - 1)
                            stage <= stage + 1'b1;     // bf wraps to 0, keep going
                        else
                            state <= DRAIN;
                    end
                end
                DRAIN: begin
                    if (stage == LOGN - 1) begin
                        // last stage, wait until the last write is done too
                        if (!(v0 | v1 | v2 | v3)) begin
                            state <= IDLE;
                            done  <= 1'b1;
                        end
                    end else if (!(v0 | v1 | v2)) begin
                        // v3 writes this clock, the next read is a clock later so it's safe
                        state <= RUN;
                        stage <= stage + 1'b1;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end

    // butterfly addresses: put a 0 at bit 'stage' of bf to get a, b = a with that bit set
    wire [LOGN-1:0] bfx     = {1'b0, bf};
    wire [LOGN-1:0] lowmask = ({{(LOGN-1){1'b0}}, 1'b1} << stage) - 1'b1;
    wire [LOGN-1:0] a_idx   = ((bfx & ~lowmask) << 1) | (bfx & lowmask);
    wire [LOGN-1:0] b_idx   = a_idx | (lowmask + 1'b1);
    wire [AW-1:0]   tw_k    = (bf & lowmask[AW-1:0]) << (LOGN - 1 - stage);
    wire            a_bank  = ^a_idx;         // b is always in the other bank

    // ------------------------------------------------------------ memory

    wire [2*W-1:0] rd0, rd1;
    reg            we0, we1;
    reg  [AW-1:0]  wa0, wa1, ra0, ra1;
    reg  [2*W-1:0] wd0, wd1;

    sample_ram #(.DW(2*W), .AW(AW)) u_bank0 (
        .clk(clk), .we(we0), .waddr(wa0), .wdata(wd0), .raddr(ra0), .rdata(rd0));
    sample_ram #(.DW(2*W), .AW(AW)) u_bank1 (
        .clk(clk), .we(we1), .waddr(wa1), .wdata(wd1), .raddr(ra1), .rdata(rd1));

    wire signed [TW-1:0] tw_c, tw_s;
    twiddle_rom u_rom (.clk(clk), .addr(tw_k), .cos_o(tw_c), .sin_o(tw_s));

    // pipeline regs
    reg            sw0, sw1, sw2, sw3;    // 1 = a lives in bank 1
    reg [AW-1:0]   a0_0, a1_0, a0_1, a1_1, a0_2, a1_2, a0_3, a1_3;   // bank0 / bank1 address per stage
    reg signed [W-1:0]  ar1, ai1;
    reg signed [SW-1:0] tr1, ti1;
    reg signed [SW-1:0] s0r, s0i, s1r, s1i;
    reg [W-1:0]    y0r, y0i, y1r, y1i;

    wire [LOGN-1:0] in_br = bitrev(in_addr);
    reg             out_bank;

    // read side: butterfly addresses while running, read back port when idle
    always @(*) begin
        if (state == RUN) begin
            ra0 = a_bank ? b_idx[LOGN-1:1] : a_idx[LOGN-1:1];
            ra1 = a_bank ? a_idx[LOGN-1:1] : b_idx[LOGN-1:1];
        end else begin
            ra0 = out_addr[LOGN-1:1];
            ra1 = out_addr[LOGN-1:1];
        end
    end

    // write side: butterfly results from s3, or the load port when idle
    always @(*) begin
        if (state == IDLE && in_we) begin
            we0 = ~^in_br;
            we1 =  ^in_br;
            wa0 = in_br[LOGN-1:1];
            wa1 = in_br[LOGN-1:1];
            wd0 = {in_re, in_im};
            wd1 = {in_re, in_im};
        end else begin
            we0 = v3;
            we1 = v3;
            wa0 = a0_3;
            wa1 = a1_3;
            wd0 = sw3 ? {y1r, y1i} : {y0r, y0i};
            wd1 = sw3 ? {y0r, y0i} : {y1r, y1i};
        end
    end

    always @(posedge clk)
        out_bank <= ^out_addr;
    assign out_re = out_bank ? rd1[2*W-1:W] : rd0[2*W-1:W];
    assign out_im = out_bank ? rd1[W-1:0]   : rd0[W-1:0];

    // ------------------------------------------------------------ butterfly

    // a and b out of the two banks
    wire signed [W-1:0] ar = sw0 ? rd1[2*W-1:W] : rd0[2*W-1:W];
    wire signed [W-1:0] ai = sw0 ? rd1[W-1:0]   : rd0[W-1:0];
    wire signed [W-1:0] br = sw0 ? rd0[2*W-1:W] : rd1[2*W-1:W];
    wire signed [W-1:0] bi = sw0 ? rd0[W-1:0]   : rd1[W-1:0];

    // 4 plain W x TW products, these go to the dsp blocks
    wire signed [W+TW-1:0] p_rc = br * tw_c;
    wire signed [W+TW-1:0] p_is = bi * tw_s;
    wire signed [W+TW-1:0] p_ic = bi * tw_c;
    wire signed [W+TW-1:0] p_rs = br * tw_s;

    wire signed [SW-1:0] half = {{(SW-TW){1'b0}}, 1'b1, {(TW-1){1'b0}}};

    always @(posedge clk) begin
        if (rst) begin
            v0 <= 1'b0; v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0;
        end else begin
            v0 <= (state == RUN);
            v1 <= v0;
            v2 <= v1;
            v3 <= v2;
        end

        // rd -> s1: remember where a and b came from
        sw0  <= a_bank;
        a0_0 <= ra0;
        a1_0 <= ra1;

        // s1: complex multiply, b * (c - js)
        tr1  <= p_rc + p_is;
        ti1  <= p_ic - p_rs;
        ar1  <= ar;
        ai1  <= ai;
        sw1  <= sw0;
        a0_1 <= a0_0;
        a1_1 <= a1_0;

        // s2: add / sub, a gets lined up with the TW-1 fraction bits of t
        s0r  <= (ar1 <<< (TW-1)) + tr1 + half;
        s0i  <= (ai1 <<< (TW-1)) + ti1 + half;
        s1r  <= (ar1 <<< (TW-1)) - tr1 + half;
        s1i  <= (ai1 <<< (TW-1)) - ti1 + half;
        sw2  <= sw1;
        a0_2 <= a0_1;
        a1_2 <= a1_1;

        // s3: drop TW bits (that's the 1/2 scale too) and clip
        y0r  <= clip(s0r >>> TW);
        y0i  <= clip(s0i >>> TW);
        y1r  <= clip(s1r >>> TW);
        y1i  <= clip(s1i >>> TW);
        sw3  <= sw2;
        a0_3 <= a0_2;
        a1_3 <= a1_2;
    end

    // sticky clip flag for the whole run
    always @(posedge clk) begin
        if (rst || (state == IDLE && start))
            sat <= 1'b0;
        else if (v2 && (clipped(s0r >>> TW) || clipped(s0i >>> TW) ||
                        clipped(s1r >>> TW) || clipped(s1i >>> TW)))
            sat <= 1'b1;
    end
endmodule
