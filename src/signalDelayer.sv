(* keep_hierarchy = "yes" *) module signalDelayer #(
    parameter WIDTH = 8, 
    parameter DELAY = 4,
    parameter ENABLE_STALLING = 0
)(
    input wire clk,             
    input wire rst,     
    input wire stall,  
    input wire [WIDTH-1:0] in,
    output wire [DELAY:0][WIDTH-1:0] out
);

    reg [DELAY-1:0][WIDTH-1:0] r_out;

    always_ff @(posedge clk) begin
          if (rst) begin
            r_out[0] <= 'b0;
          end
          else begin
            if (ENABLE_STALLING) r_out[0] <= stall ? r_out[0] : in;
            else                 r_out[0] <= in;
          end
    end
    
    assign out[0] = in;
    
    genvar i;
    for (i=1 ; i<DELAY+1 ; i=i+1) begin
        assign out[i] = r_out[i-1];
    end 
    for (i=1 ; i<DELAY ; i=i+1) begin
        always_ff @(posedge clk) begin
          if (rst) begin
            r_out[i] <= 'b0;
          end
          else begin
            if (ENABLE_STALLING) r_out[i] <= stall ? r_out[i] : r_out[i-1];
            else r_out[i] <= r_out[i-1];
          end
        end
    end

endmodule
