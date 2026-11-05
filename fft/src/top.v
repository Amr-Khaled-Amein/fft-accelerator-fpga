// top level for the Tang Nano 20K: uart in -> fft -> uart out
// talk to it with tools/fft_uart.py
//
// pc sends:   'F', then N samples of re_hi re_lo im_hi im_lo (16 bit signed, big endian)
// fpga sends: A5 5A, N bins (same format, natural order), cycles (4 bytes),
//             flags (bit 0 = something clipped), xor of everything after A5 5A
//
// leds (active low):
//   led[0]  receiving samples
//   led[1]  fft running (stretched to ~80 ms so you can see it)
//   led[2]  sending the result
//   led[3]  toggles after every finished fft
//   led[4]  last fft clipped
//   led[5]  error: a frame stopped halfway and timed out (cleared by the next good one)

module top #(
    parameter CLKS_PER_BIT = 234,       // 27 MHz / 115200. the sim overrides this to go faster
    parameter N            = 256,
    parameter TIMEOUT      = 2700000,   // ~100 ms without a byte in the middle of a frame
    parameter STRETCH      = 21         // led[1] on time = 2^21 clocks
)(
    input  wire       clk,
    input  wire       uart_rxd,
    output wire       uart_txd,
    output wire [5:0] led
);
    localparam W    = 16;               // the uart frame is 16 bit per value
    localparam LOGN = $clog2(N);
    localparam NB   = 4 * N + 8;        // bytes in a reply
    localparam BW   = LOGN + 3;

    // power on reset, the regs come up as 0 on the gowin so this just counts up once
    reg [7:0] por_cnt = 8'd0;
    wire      por = ~&por_cnt;
    always @(posedge clk)
        if (por) por_cnt <= por_cnt + 8'd1;

    // ------------------------------------------------------------ uart

    wire [7:0] rx_data;
    wire       rx_valid;
    reg  [7:0] tx_data;
    wire       tx_valid, tx_ready;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst(por), .rxd(uart_rxd), .data(rx_data), .valid(rx_valid));

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst(por), .data(tx_data), .valid(tx_valid), .ready(tx_ready), .txd(uart_txd));

    // ------------------------------------------------------------ fft

    reg             in_we;
    reg  [LOGN-1:0] in_addr;
    reg  [W-1:0]    in_re, in_im;
    wire [LOGN-1:0] out_addr;
    wire [W-1:0]    out_re, out_im;
    reg             start;
    wire            busy, done, sat;

    fft_core #(.N(N), .W(W), .TW(16)) u_fft (
        .clk(clk), .rst(por),
        .in_we(in_we), .in_addr(in_addr), .in_re(in_re), .in_im(in_im),
        .out_addr(out_addr), .out_re(out_re), .out_im(out_im),
        .start(start), .busy(busy), .done(done), .sat(sat)
    );

    // ------------------------------------------------------------ control

    localparam S_CMD = 3'd0, S_LOAD = 3'd1, S_START = 3'd2, S_RUN = 3'd3, S_SEND = 3'd4;
    reg [2:0]    state;
    reg [BW-1:0] bcnt;          // byte counter, load and send
    reg [23:0]   sh;            // first 3 bytes of a sample
    reg [31:0]   tmo;
    reg [31:0]   cycles;        // clocks the core was busy for
    reg [7:0]    cs;
    reg          err, toggle, clip;

    // which byte goes out next
    wire [BW-1:0] j = bcnt - 2'd2;
    assign out_addr = j[LOGN+1:2];
    always @(*) begin
        if (bcnt == 0)
            tx_data = 8'hA5;
        else if (bcnt == 1)
            tx_data = 8'h5A;
        else if (bcnt < 4 * N + 2)
            case (j[1:0])
                2'd0: tx_data = out_re[15:8];
                2'd1: tx_data = out_re[7:0];
                2'd2: tx_data = out_im[15:8];
                default: tx_data = out_im[7:0];
            endcase
        else if (bcnt < 4 * N + 6)
            case (bcnt[1:0])        // 4N+2 .. 4N+5, 4N is a multiple of 4
                2'd2: tx_data = cycles[31:24];
                2'd3: tx_data = cycles[23:16];
                2'd0: tx_data = cycles[15:8];
                default: tx_data = cycles[7:0];
            endcase
        else if (bcnt == 4 * N + 6)
            tx_data = {7'd0, clip};
        else
            tx_data = cs;
    end

    assign tx_valid = (state == S_SEND);

    always @(posedge clk) begin
        in_we <= 1'b0;
        start <= 1'b0;
        if (por) begin
            state  <= S_CMD;
            bcnt   <= 0;
            err    <= 1'b0;
            toggle <= 1'b0;
            clip   <= 1'b0;
            cycles <= 32'd0;
            tmo    <= 32'd0;
        end else begin
            case (state)
                S_CMD: if (rx_valid && rx_data == 8'h46) begin
                    state <= S_LOAD;
                    bcnt  <= 0;
                    tmo   <= 32'd0;
                end

                S_LOAD: begin
                    if (rx_valid) begin
                        tmo  <= 32'd0;
                        sh   <= {sh[15:0], rx_data};
                        bcnt <= bcnt + 1'b1;
                        if (bcnt[1:0] == 2'd3) begin
                            in_we   <= 1'b1;
                            in_addr <= bcnt[LOGN+1:2];
                            in_re   <= sh[23:8];
                            in_im   <= {sh[7:0], rx_data};
                        end
                        if (bcnt == 4 * N - 1)
                            state <= S_START;
                    end else if (tmo == TIMEOUT) begin
                        err   <= 1'b1;
                        state <= S_CMD;
                    end else
                        tmo <= tmo + 1'b1;
                end

                S_START: begin          // one clock so the last sample is written first
                    start  <= 1'b1;
                    cycles <= 32'd0;
                    state  <= S_RUN;
                end

                S_RUN: begin
                    if (busy)
                        cycles <= cycles + 1'b1;
                    if (done) begin
                        clip   <= sat;
                        err    <= 1'b0;
                        toggle <= ~toggle;
                        bcnt   <= 0;
                        cs     <= 8'd0;
                        state  <= S_SEND;
                    end
                end

                S_SEND: if (tx_ready) begin   // tx_valid is high the whole time we're here
                    if (bcnt >= 2)
                        cs <= cs ^ tx_data;
                    bcnt <= bcnt + 1'b1;
                    if (bcnt == NB - 1)
                        state <= S_CMD;
                end

                default: state <= S_CMD;
            endcase
        end
    end

    // stretch the busy pulse so it shows on the led
    reg [STRETCH:0] led_busy;
    always @(posedge clk)
        if (por)
            led_busy <= 0;
        else if (start)
            led_busy <= {1'b1, {STRETCH{1'b0}}};
        else if (led_busy != 0)
            led_busy <= led_busy - 1'b1;

    assign led = ~{err, clip, toggle, state == S_SEND, led_busy != 0, state == S_LOAD};
endmodule
