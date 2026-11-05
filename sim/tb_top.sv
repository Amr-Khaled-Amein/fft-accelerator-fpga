// tb_top - the whole design (uart + control + fft) at a faster baud rate
//
// 1. a couple of junk bytes, they have to be ignored
// 2. half a frame, then silence -> has to time out and set the error led
// 3. every vector in in.hex as a full frame, the reply is checked byte by byte
//    (header, all bins, flags, checksum) and the error led has to clear
// all received bytes go to uart.out, run_tests.py also parses that with fft_uart.py

`timescale 1ns/1ps

module tb_top;
    localparam CPB     = 16;        // clocks per uart bit
    localparam N       = 256;
    localparam TIMEOUT = 20000;

    logic clk = 1'b0;
    always #18.5 clk = ~clk;

    logic rxd = 1'b1;               // pc -> fpga
    wire  txd;
    wire [5:0] led;

    top #(.CLKS_PER_BIT(CPB), .N(N), .TIMEOUT(TIMEOUT), .STRETCH(4)) dut (
        .clk(clk), .uart_rxd(rxd), .uart_txd(txd), .led(led));

    task automatic send_byte(input [7:0] b);
        int i;
        rxd = 1'b0;
        repeat (CPB) @(posedge clk);
        for (i = 0; i < 8; i++) begin
            rxd = b[i];
            repeat (CPB) @(posedge clk);
        end
        rxd = 1'b1;
        repeat (CPB) @(posedge clk);
    endtask

    // receiver, runs the whole time
    logic [7:0] rx [0:65535];
    int nrx = 0, framing = 0;
    initial begin : rx_loop
        logic [7:0] b;
        int i;
        forever begin
            @(negedge txd);
            repeat (CPB / 2) @(posedge clk);
            for (i = 0; i < 8; i++) begin
                repeat (CPB) @(posedge clk);
                b[i] = txd;
            end
            repeat (CPB) @(posedge clk);
            if (!txd) framing++;
            rx[nrx] = b;
            nrx++;
        end
    end

    logic [15:0] xr [0:N-1];
    logic [15:0] xi [0:N-1];
    logic [15:0] er [0:N-1];
    logic [15:0] ei [0:N-1];
    reg [8*40-1:0] name;
    int fin, fexp, fsat, fnam, fout;
    int k, r, t, base, nbad, exp_sat, nvec = 0, badvec = 0, errors = 0;
    logic [7:0] cs;
    logic [15:0] gr, gi;
    logic [31:0] cyc;

    initial begin
        fin  = $fopen("in.hex", "r");
        fexp = $fopen("exp.hex", "r");
        fsat = $fopen("sat.hex", "r");
        fnam = $fopen("names.txt", "r");
        if (!fin || !fexp || !fsat || !fnam) begin
            $error("can't open the vector files");
            $finish;
        end
        repeat (300) @(posedge clk);

        // junk, nothing should come back
        send_byte(8'h00);
        send_byte(8'h55);
        // half a frame then nothing
        send_byte(8'h46);
        for (k = 0; k < 37; k++)
            send_byte(k);
        repeat (TIMEOUT + 100) @(posedge clk);
        if (led[5] !== 1'b0) begin
            $display("  FAIL error led didn't come on after a timeout");
            errors++;
        end else
            $display("  ok   half frame timed out, error led on");
        if (nrx != 0) begin
            $display("  FAIL got %0d bytes back from junk", nrx);
            errors++;
        end

        while ($fscanf(fnam, "%s", name) == 1) begin
            for (k = 0; k < N; k++) begin
                r = $fscanf(fin, "%h %h", xr[k], xi[k]);
                r = $fscanf(fexp, "%h %h", er[k], ei[k]);
            end
            r = $fscanf(fsat, "%d", exp_sat);

            base = nrx;
            send_byte(8'h46);
            for (k = 0; k < N; k++) begin
                send_byte(xr[k][15:8]);
                send_byte(xr[k][7:0]);
                send_byte(xi[k][15:8]);
                send_byte(xi[k][7:0]);
            end

            t = 0;
            while (nrx < base + 4 * N + 8 && t < 200 * CPB * N) begin
                @(posedge clk);
                t++;
            end
            nbad = 0;
            if (nrx < base + 4 * N + 8) begin
                $display("  FAIL %0s: only %0d reply bytes", name, nrx - base);
                nbad++;
            end else begin
                if (rx[base] !== 8'hA5 || rx[base + 1] !== 8'h5A) nbad++;
                cs = 8'd0;
                for (k = 2; k < 4 * N + 7; k++)
                    cs ^= rx[base + k];
                if (cs !== rx[base + 4 * N + 7]) begin
                    $display("    %0s: bad checksum", name);
                    nbad++;
                end
                for (k = 0; k < N; k++) begin
                    gr = {rx[base + 2 + 4 * k], rx[base + 3 + 4 * k]};
                    gi = {rx[base + 4 + 4 * k], rx[base + 5 + 4 * k]};
                    if (gr !== er[k] || gi !== ei[k]) begin
                        if (nbad < 4)
                            $display("    %0s bin %0d: got %h %h expected %h %h", name, k, gr, gi, er[k], ei[k]);
                        nbad++;
                    end
                end
                cyc = {rx[base + 4 * N + 2], rx[base + 4 * N + 3], rx[base + 4 * N + 4], rx[base + 4 * N + 5]};
                if (rx[base + 4 * N + 6] !== {7'd0, exp_sat[0]}) begin
                    $display("    %0s: flags %h, model says clip=%0d", name, rx[base + 4 * N + 6], exp_sat);
                    nbad++;
                end
                if (led[5] !== 1'b1) begin
                    $display("    %0s: error led still on", name);
                    nbad++;
                end
            end
            $display("  %-4s %-14s %0d cycles (from the board counter)", nbad ? "FAIL" : "ok", name, cyc);
            nvec++;
            if (nbad) badvec++;
        end

        if (framing) begin
            $display("  FAIL %0d uart framing errors", framing);
            errors++;
        end
        fout = $fopen("uart.out", "w");
        for (k = 0; k < nrx; k++)
            $fdisplay(fout, "%h", rx[k]);
        $fclose(fout);

        if (errors == 0 && badvec == 0 && nvec > 0)
            $display("PASS: %0d/%0d frames over uart", nvec, nvec);
        else
            $display("FAIL: %0d/%0d frames bad, %0d other errors", badvec, nvec, errors);
        $finish;
    end
endmodule
