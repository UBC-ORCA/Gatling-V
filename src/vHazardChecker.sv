module vHazardChecker #(
    parameter ADDR_WIDTH    = 5, // 32 locations
    parameter WR_PORT_COUNT = 2
) (
    input                                          clk                 ,
    input                                          rst                 ,
    input      [WR_PORT_COUNT-1:0]                 wrStart             ,
    input      [WR_PORT_COUNT-1:0][ADDR_WIDTH-1:0] wrAddr              ,
    input                                          updateExpectedWbAddr,
    input      [   ADDR_WIDTH-1:0]                 expectedWbAddr      ,
    input                                          checkHazard0        ,
    input                                          checkHazard1        ,
    input      [   ADDR_WIDTH-1:0]                 checkWrAddr0        ,
    input      [   ADDR_WIDTH-1:0]                 checkWrAddr1        ,
    output reg                                     noHazard
);

    // Define a single memory bank to store valid signals
    (* ram_style = "distributed" *)reg validBank[0:32-1];
    integer i;
    // Initialize the distributed RAM to 1
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            validBank[i] = 1'b1;
        end
    end

    wire [ADDR_WIDTH-1:0] selected_wrAddr    ;
    wire [ADDR_WIDTH-1:0] selected_addr      ;
    reg  [ADDR_WIDTH-1:0] selected_wrAddr_reg;
    reg          wrStart_reg        ;

    always @(posedge clk) begin
        if (rst) begin
            selected_wrAddr_reg <= 'b0;
            wrStart_reg         <= 'b0;
        end else begin
            selected_wrAddr_reg <= selected_wrAddr;
            wrStart_reg         <= updateExpectedWbAddr & (|wrStart);
        end
    end


    if (WR_PORT_COUNT == 3)
        assign selected_wrAddr = wrStart[0] ? wrAddr[0] : (wrStart[1] ? wrAddr[1] : wrAddr[2]);
    else if (WR_PORT_COUNT == 5)
        assign selected_wrAddr = wrStart[0] ? wrAddr[0] : (
            wrStart[1] ? wrAddr[1] : (
                wrStart[2] ? wrAddr[2] : (
                    wrStart[3] ? wrAddr[3] : wrAddr[4])));
    else
        assign selected_wrAddr = wrAddr[0]; //default case


    assign selected_addr = updateExpectedWbAddr ? expectedWbAddr : ((|wrStart) ? selected_wrAddr : selected_wrAddr_reg);

    always @(posedge clk) begin
        if (updateExpectedWbAddr | (|wrStart) | wrStart_reg)
            validBank[selected_addr] <= ~updateExpectedWbAddr;
    end

    always @(posedge clk) begin
        if (rst) begin
            // Reset noHazard signal only
            noHazard <= 1'b0;
        end else begin
            // Prioritize write operations
            //if (updateExpectedWbAddr) begin
            //    validBank[expectedWbAddr] <= 1'b0; // Clear valid bit when updating expected
            //end
            /*else begin if (wrStart[0]) begin
            validBank[wrAddr[0][ADDR_WIDTH-1:ADDR_WIDTH-5]] <= 1'b1; // Set valid bit for slot0 write
            end else if (wrStart[1]) begin
            validBank[wrAddr[1][ADDR_WIDTH-1:ADDR_WIDTH-5]] <= 1'b1; // Set valid bit for slot1 write
            end
            */
            //     for (i = 0; i < WR_PORT_COUNT; i++) begin
            //         if (wrStart[i]) begin
            //             validBank[wrAddr[i][ADDR_WIDTH-1:ADDR_WIDTH-5]] <= 1'b1;
            //         end
            //     end
            // end
            // Hazard detection
            if (checkHazard0 | checkHazard1) begin
                noHazard <= ((validBank[checkWrAddr0] & !((selected_addr == checkWrAddr0) & updateExpectedWbAddr) & checkHazard0) | (!checkHazard0)) & ((validBank[checkWrAddr1] & !((selected_addr == checkWrAddr1) & updateExpectedWbAddr) & checkHazard1) | (!checkHazard1)); // Check if valid bit is set
                //noHazard <= ((validBank[checkWrAddr0] & checkHazard0) | (!checkHazard0)) & ((validBank[checkWrAddr1] & checkHazard1) | (!checkHazard1)); // Check if valid bit is set
            end else begin
                noHazard <= 1'b0; // Default to no hazard
            end
        end
    end

endmodule
