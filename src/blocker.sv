module blocker #(
    parameter SLOT_COUNT     = 8,
    parameter TIMEOUT_CYCLES = 32
)(
    input  wire                   clk,
    input  wire                   rst,   // synchronous reset
    input  wire                   ack,
    input  wire                   trig,
    input  wire [SLOT_COUNT-1:0]  slot_valid,
    input wire stall,
    output reg                   block
);

    // Internal registers
    reg [$clog2(TIMEOUT_CYCLES):0] counter;
    reg counting;


    always @(posedge clk) begin
        if (rst) begin
            counter  <= {($clog2(TIMEOUT_CYCLES)+1){1'b0}};
            counting <= 1'b0;
            block    <= 1'b0;
        end else begin
            if (!counting) begin
                if (slot_valid == {SLOT_COUNT{1'b0}}) begin
                    counting <= 1'b1;
                    counter  <= {($clog2(TIMEOUT_CYCLES)+1){1'b0}};
                    block    <= ack & trig;
                end
                else begin
                    counting <= 1'b0;
                    counter  <= {($clog2(TIMEOUT_CYCLES)+1){1'b0}};
                    block    <= 1'b0;
                end
            end 
            else begin
                if (counter < TIMEOUT_CYCLES) begin
                    counter  <= counter + {'b0,~stall};
                    counting <= 1'b1;
                    block    <= ack & trig;
                end 
                else begin
                    counting <= 1'b0;
                    counter  <= {($clog2(TIMEOUT_CYCLES)+1){1'b0}};
                    block    <= 1'b0;
                end
            end
        end
    end

endmodule
