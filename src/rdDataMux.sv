module rdDataMux #(
    parameter DATA_WIDTH = 8,
    parameter ID = 0,
    parameter N = 4,
    parameter ENABLE_STALLING = 0
)(
    input wire clk,
    input wire rst,
    input wire stall,
    input wire [2*N-1:0][DATA_WIDTH-1:0] in,
    input wire [1:0][$clog2(N)-1:0] sel,
    input wire uses_scalar_data,
    input wire [DATA_WIDTH-1:0] scalar_data,
    output reg [2-1:0][DATA_WIDTH-1:0] out
);

    localparam int IN_WIDTH = 2 * N;
    
    // OFFSET shifts the input sel range so that sel = ID * 2 maps to index0 = 0
    localparam int OFFSET = 4 * ID;

    always @(posedge clk) begin
        if (rst) begin
            out[0] <= {DATA_WIDTH{1'b0}};
            out[1] <= {DATA_WIDTH{1'b0}};  
        end
        else begin
            out[0] <= (ENABLE_STALLING & stall) ? out[0] : (uses_scalar_data ? scalar_data : in[(2 * sel[0] - OFFSET + IN_WIDTH) % IN_WIDTH]);
            out[1] <= (ENABLE_STALLING & stall) ? out[1] : (in[(((2 * sel[1] - OFFSET + IN_WIDTH) % IN_WIDTH) + 1) % IN_WIDTH]);
        end
    end
endmodule
