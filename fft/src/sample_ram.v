// one bank of sample memory. simple dual port, 1 write + 1 read per clock,
// read data comes out one clock after the address (gowin maps this to bsram)

module sample_ram #(
    parameter DW = 32,
    parameter AW = 7
)(
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);
    reg [DW-1:0] mem [0:(1<<AW)-1];

    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end
endmodule
