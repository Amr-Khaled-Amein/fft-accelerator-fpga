// 8N1 uart transmitter. valid/ready handshake: byte is taken when both are high.
// 27 MHz / 115200 = 234.375 -> 234, about 0.16% off, fine

module uart_tx #(
    parameter CLKS_PER_BIT = 234
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data,
    input  wire       valid,
    output wire       ready,
    output reg        txd
);
    reg [9:0]  shreg;     // stop, data[7:0], start  (sent lsb first)
    reg [3:0]  nbit;
    reg [15:0] cnt;
    reg        active;

    assign ready = ~active;

    always @(posedge clk) begin
        if (rst) begin
            active <= 1'b0;
            txd    <= 1'b1;
            cnt    <= 16'd0;
            nbit   <= 4'd0;
            shreg  <= 10'h3ff;
        end else if (!active) begin
            txd <= 1'b1;
            if (valid) begin
                shreg  <= {1'b1, data, 1'b0};
                active <= 1'b1;
                cnt    <= 16'd0;
                nbit   <= 4'd0;
            end
        end else begin
            txd <= shreg[0];
            if (cnt == CLKS_PER_BIT - 1) begin
                cnt   <= 16'd0;
                shreg <= {1'b1, shreg[9:1]};
                if (nbit == 4'd9)
                    active <= 1'b0;
                else
                    nbit <= nbit + 4'd1;
            end else begin
                cnt <= cnt + 16'd1;
            end
        end
    end
endmodule
