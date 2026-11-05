// 8N1 uart receiver, samples in the middle of each bit
// valid is a 1 clock pulse when a byte is ready

module uart_rx #(
    parameter CLKS_PER_BIT = 234
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       rxd,
    output reg  [7:0] data,
    output reg        valid
);
    localparam IDLE = 2'd0, START = 2'd1, BITS = 2'd2, STOP = 2'd3;

    // rxd comes from outside, sync it first
    reg rx_s1 = 1'b1, rx_s2 = 1'b1;
    always @(posedge clk) begin
        rx_s1 <= rxd;
        rx_s2 <= rx_s1;
    end

    reg [1:0]  state;
    reg [15:0] cnt;
    reg [2:0]  nbit;
    reg [7:0]  sh;

    always @(posedge clk) begin
        valid <= 1'b0;
        if (rst) begin
            state <= IDLE;
            cnt   <= 16'd0;
            nbit  <= 3'd0;
            data  <= 8'd0;
            sh    <= 8'd0;
        end else begin
            case (state)
                IDLE: begin
                    cnt <= 16'd0;
                    if (!rx_s2) state <= START;
                end
                START: begin
                    // check it's still low halfway through, otherwise it was a glitch
                    if (cnt == (CLKS_PER_BIT / 2) - 1) begin
                        cnt   <= 16'd0;
                        nbit  <= 3'd0;
                        state <= rx_s2 ? IDLE : BITS;
                    end else
                        cnt <= cnt + 16'd1;
                end
                BITS: begin
                    if (cnt == CLKS_PER_BIT - 1) begin
                        cnt <= 16'd0;
                        sh  <= {rx_s2, sh[7:1]};
                        if (nbit == 3'd7) state <= STOP;
                        else              nbit  <= nbit + 3'd1;
                    end else
                        cnt <= cnt + 16'd1;
                end
                STOP: begin
                    if (cnt == CLKS_PER_BIT - 1) begin
                        cnt   <= 16'd0;
                        state <= IDLE;
                        if (rx_s2) begin   // drop it if the stop bit is bad
                            data  <= sh;
                            valid <= 1'b1;
                        end
                    end else
                        cnt <= cnt + 16'd1;
                end
            endcase
        end
    end
endmodule
