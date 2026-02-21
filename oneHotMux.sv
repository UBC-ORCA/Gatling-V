(* keep_hierarchy = "yes" *) module oneHotMux #(
    parameter WIDTH = 8,
    parameter N     = 4
) (
    input  wire                    clk,
    input  wire                    rst,    // synchronous reset
    input  wire [N-1:0]            sel,    // one-hot select
    input  wire [N-1:0][WIDTH-1:0] data,   // N inputs of WIDTH each
    output reg [WIDTH-1:0]        out     // registered output
);

    logic [WIDTH-1:0] mux_result;

    always_comb begin
        mux_result = '0;
        for (int i = 0; i < N; i++) begin
            if (sel[i])
                mux_result = data[i];
        end
    end

    always_ff @(posedge clk) begin
        if (rst)
            out <= '0;
        else
            out <= mux_result;
    end

endmodule
