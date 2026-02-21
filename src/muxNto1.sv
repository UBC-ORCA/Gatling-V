module muxNto1 #(
    parameter DATA_WIDTH = 8,
    parameter N,
    parameter ENABLE_STALLING = 0
)(
    input wire clk,
    input wire rst,
    input wire [N-1:0][DATA_WIDTH-1:0] in,
    input wire [$clog2(N)-1:0] sel,
    output reg [DATA_WIDTH-1:0] out,
    input wire stall
);

    always @(posedge clk) begin
        if (rst)
            out <= {DATA_WIDTH{1'b0}};  // Reset output to 0
        else
            if (ENABLE_STALLING) out <= stall ? out : in[sel];
            else out <= in[sel];
    end

endmodule

