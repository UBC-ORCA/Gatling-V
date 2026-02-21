module bank #(
    parameter ADDR_WIDTH      = 10             , // Address width (e.g., 10 for 1024 locations)
    parameter DATA_WIDTH      = 64             , // Data width (64 bits)
    parameter BANK_ID         = 0              ,
    parameter BANK_COUNT      = 4              ,
    parameter ENABLE_STALLING = 0
) (
    input  wire                        clk    , // Clock for read port
    input                              rst    ,
    input                              stall  ,
    input  wire     [  ADDR_WIDTH-1:0] wr_addr, // Address for write port
    input  wire     [  ADDR_WIDTH-1:0] rd_addr, // Address for read port
    input  wire     [  DATA_WIDTH-1:0] wr_data, // Data input for write port
    output      reg [  DATA_WIDTH-1:0] rd_data, // Data output for read port
    input  wire                        wr_en  , // Write enable for write port
    input  wire     [DATA_WIDTH/8-1:0] wr_be    // Byte enable for write port
);

    localparam NUM_WORDS       = 1 << ADDR_WIDTH;
    // Memory array declaration
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] memory [0:NUM_WORDS-1];

    initial begin
        // Initialize memory to 0, 1, 2, ..., NUM_WORDS-1
        for (int i = 0; i < NUM_WORDS; i = i + 1) begin
            memory[i] = i * BANK_COUNT + BANK_ID; // Initialize each memory location with its index value
        end
    end

    // Read operation
    always @(posedge clk) begin
        if (rst) begin
            rd_data <= 'b0;
        end
        else begin
            if (ENABLE_STALLING) rd_data <= stall ? rd_data : memory[rd_addr];
            else rd_data         <= memory[rd_addr];
        end
    end

    genvar j;
    for (j = 0; j < DATA_WIDTH/8; j = j + 1) begin
        // Write operation with byte enable
        always @(posedge clk) begin
            if (wr_en) begin
                if (wr_be[j]) memory[wr_addr][j*8+7:j*8]     <= wr_data[j*8+7:j*8];
            end
        end
    end


    //always @(posedge clk) begin
    //// Print the entire memory contents in a formatted way
    //$display("At time %0t, Memory Contents:", $time);
    //for (int i = 0; i < NUM_WORDS; i = i + 1) begin
    //  // Print each memory location with its index in hexadecimal format
    //  $display("Memory[%0d] = %h", i, memory[i]);
    //end
    //end

endmodule
