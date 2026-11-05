// tb_fft - self checking testbench for fft_core
//
// reads in.hex / exp.hex / sat.hex / names.txt from the current folder (made by
// tools/gen_vectors.py), runs every vector through the core and compares all
// bins and the clip flag against the python model. prints PASS or FAIL.
// also writes out.hex (what the rtl gave) and results.txt for run_tests.py
//
//   iverilog -g2012 -P tb_fft.N=256 -P tb_fft.W=16 -P tb_fft.TW=16 ...
//   +vcd    dump tb_fft.vcd for gtkwave

`timescale 1ns/1ps

module tb_fft;
    parameter N  = 256;
    parameter W  = 16;
    parameter TW = 16;
    localparam LOGN = $clog2(N);

    logic clk = 1'b0;
    logic rst = 1'b1;
    always #18.5 clk = ~clk;

    logic            in_we = 1'b0;
    logic [LOGN-1:0] in_addr = '0, out_addr = '0;
    logic [W-1:0]    in_re = '0, in_im = '0;
    logic            start = 1'b0;
    wire  [W-1:0]    out_re, out_im;
    wire             busy, done, sat;

    fft_core #(.N(N), .W(W), .TW(TW)) dut (
        .clk(clk), .rst(rst),
        .in_we(in_we), .in_addr(in_addr), .in_re(in_re), .in_im(in_im),
        .out_addr(out_addr), .out_re(out_re), .out_im(out_im),
        .start(start), .busy(busy), .done(done), .sat(sat)
    );

    logic [W-1:0] xr [0:N-1];
    logic [W-1:0] xi [0:N-1];
    logic [W-1:0] er [0:N-1];
    logic [W-1:0] ei [0:N-1];

    int cycles = 0;
    always @(posedge clk)
        if (busy) cycles++;

    int fin, fexp, fsat, fnam, fout, fres;
    int nvec = 0, badvec = 0, badbins = 0, nbins = 0;
    int k, r, nbad, exp_sat;
    reg [8*40-1:0] name;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_fft.vcd");
            $dumpvars(0, tb_fft);
        end

        // the rom is generated separately, make sure it's for the same n / width
        if (dut.u_rom.N != N || dut.u_rom.TW != TW) begin
            $error("twiddle_rom.v is for n=%0d tw=%0d, tb wants n=%0d tw=%0d",
                   dut.u_rom.N, dut.u_rom.TW, N, TW);
            $finish;
        end

        fin  = $fopen("in.hex", "r");
        fexp = $fopen("exp.hex", "r");
        fsat = $fopen("sat.hex", "r");
        fnam = $fopen("names.txt", "r");
        fout = $fopen("out.hex", "w");
        fres = $fopen("results.txt", "w");
        if (!fin || !fexp || !fsat || !fnam) begin
            $error("can't open the vector files");
            $finish;
        end

        repeat (3) @(posedge clk);
        @(negedge clk) rst = 1'b0;

        while ($fscanf(fnam, "%s", name) == 1) begin
            for (k = 0; k < N; k++) begin
                r = $fscanf(fin, "%h %h", xr[k], xi[k]);
                r = $fscanf(fexp, "%h %h", er[k], ei[k]);
            end
            r = $fscanf(fsat, "%d", exp_sat);

            // load, natural order (the core does the bit reversal)
            for (k = 0; k < N; k++) begin
                @(negedge clk);
                in_we   = 1'b1;
                in_addr = k;
                in_re   = xr[k];
                in_im   = xi[k];
            end
            @(negedge clk) in_we = 1'b0;

            // run
            cycles = 0;
            start = 1'b1;
            @(negedge clk) start = 1'b0;
            k = 0;
            while (!done && k < 40 * N * LOGN) begin
                @(posedge clk);
                k++;
            end
            if (!done) begin
                $error("%0s: timeout, done never came", name);
                $finish;
            end

            // read back, data shows up one clock after the address
            nbad = 0;
            @(negedge clk) out_addr = 0;
            for (k = 0; k < N; k++) begin
                @(negedge clk);
                $fdisplay(fout, "%h %h", out_re, out_im);
                if (out_re !== er[k] || out_im !== ei[k]) begin
                    if (nbad < 4)
                        $display("    %s bin %0d: rtl %h %h  expected %h %h",
                                 name, k, out_re, out_im, er[k], ei[k]);
                    nbad++;
                end
                out_addr = k + 1;
            end
            if (sat !== exp_sat[0]) begin
                $display("    %s: clip flag %b, model says %0d", name, sat, exp_sat);
                nbad++;
            end

            $display("  %-4s %-18s %0d cycles%s", nbad ? "FAIL" : "ok", name, cycles,
                     sat ? "  (clipped)" : "");
            $fdisplay(fres, "%s %0d %0d %0d", name, nbad, cycles, sat);
            nvec++;
            nbins += N;
            badbins += (nbad > N) ? N : nbad;
            if (nbad) badvec++;
        end

        $fclose(fout);
        $fclose(fres);
        if (nvec == 0)
            $display("FAIL: no vectors");
        else if (badvec == 0)
            $display("PASS: %0d/%0d vectors, %0d/%0d bins bit exact", nvec, nvec, nbins, nbins);
        else
            $display("FAIL: %0d/%0d vectors failed, %0d bad bins", badvec, nvec, badbins);
        $finish;
    end
endmodule
