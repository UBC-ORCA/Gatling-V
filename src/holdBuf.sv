(* keep_hierarchy = "yes" *) module holdBuf #(
    parameter DATA_WIDTH = 80
) (
    input  wire                      clk,
    input  wire                      rst,
    input  wire                      save,
    input  wire     [DATA_WIDTH-1:0] data,
    input  wire                      valid,
    output reg     [DATA_WIDTH-1:0] data_out,
    output reg                  valid_out
);

    reg [DATA_WIDTH-1:0] fifo_data;
    reg fifo_full;

    always @(posedge clk) begin
        if (rst) begin
            valid_out  <= 'b0;
            data_out   <= 'b0;
            fifo_full  <= 'b0;
            fifo_data  <= 'b0;
        end 
        else begin
            valid_out  <= (~save) & (fifo_full | valid);
            data_out   <= fifo_full ? fifo_data : data;
            fifo_full  <= save;
            fifo_data  <= data;
        end
    end

endmodule
