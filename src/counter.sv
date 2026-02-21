//(* use_dsp = "yes" *) 
module counter #(
    parameter BANK_COUNT = 4,
    parameter SLOT_COUNT   = BANK_COUNT/2
) (
    input            clk        ,
    input            rst        ,
    input           stall,
    input            [SLOT_COUNT-1:0] slot_init ,
    input            [SLOT_COUNT-1:0] slot_active,
    output reg [SLOT_COUNT-1:0] slot_turn,
    output reg [$clog2(BANK_COUNT):0] count
);

    wire all_free;
    assign all_free = ~(|slot_active);

    if (BANK_COUNT == 4) begin
        always @(posedge clk) begin
            if (rst) begin
                count <= 3'b000;
                slot_turn <= 2'b10;
            end
            else begin
                if (all_free && slot_init[0]) begin
                    count <= 3'b000;
                    slot_turn <= 2'b10;
                end
                else if (all_free && slot_init[1]) begin
                    count <= 3'b010;
                    slot_turn <= 2'b01;
                end
                //else if (all_free) begin
                //    count <= 3'b000;
                //    slot_turn <= 3'b0;
                //end
                else if (~stall) begin
                    count <= count + 1;
                    if (count[1:0] == 2'b11)        slot_turn <= 2'b10;
                    else if (count[1:0] == 2'b01)   slot_turn <= 2'b01;
                    else                            slot_turn <= 2'b00;
                end
                else begin
                    count <= count;
                    slot_turn <= slot_turn;
                end
            end
        end
    end else if (BANK_COUNT == 8) begin
        always @(posedge clk) begin
            if (rst) begin
                count <= 4'b000;
                slot_turn <= 4'b0010;
            end
            else begin
                if (all_free && slot_init[0]) begin
                    count <= 4'b000;//4'b1111;//4'b000;
                    slot_turn <= 4'b0010;
                end
                else if (all_free && slot_init[1]) begin
                    count <= 4'b010;//4'b0001;//4'b010;
                    slot_turn <= 4'b0100;
                end
                else if (all_free && slot_init[2]) begin
                    count <= 4'b100;//4'b0011;//4'b100;
                    slot_turn <= 4'b1000;
                end
                else if (all_free && slot_init[3]) begin
                    count <= 4'b110;//4'b0101;//4'b110;
                    slot_turn <= 4'b0001;
                end
                //else if (all_free) begin
                //    count <= 4'b0;
                //    slot_turn <= 4'b0;
                //end
                else if (~stall) begin
                    count <= count + 1;
                    if (count[2:0] == 3'b111)       slot_turn <= 4'b0010;
                    else if (count[2:0] == 3'b001)  slot_turn <= 4'b0100;
                    else if (count[2:0] == 3'b011)  slot_turn <= 4'b1000;
                    else if (count[2:0] == 3'b101)  slot_turn <= 4'b0001;
                    else                            slot_turn <= 4'b0000;
                end
                else begin
                    count <= count;
                    slot_turn <= slot_turn;
                end
            end
        end
    end
    else begin
       always @(posedge clk) begin
            if (rst) begin
                count <= 'b0;
                slot_turn <= 'b0;
            end
            else begin
                count <= 'b0;
                slot_turn <= 'b0; 
            end 
        end 
    end
endmodule
