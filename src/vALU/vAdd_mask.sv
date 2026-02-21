module vAdd_mask #(
    parameter REQ_DATA_WIDTH  = 64,
    parameter RESP_DATA_WIDTH = 64
) (
    input  logic clk,
    input  logic rst,
    input  logic in_valid,
    input  logic [REQ_DATA_WIDTH-1:0] in_m0,      // Input for popcount
    input  logic [REQ_DATA_WIDTH-1:0] in_count,   // Input to be added with popcount
    output logic [RESP_DATA_WIDTH-1:0] out_vec
);

    // --- Popcount Logic Configuration & Signals ---
    localparam CHUNK_SIZE = 4;
    localparam NUM_CHUNKS = (REQ_DATA_WIDTH + CHUNK_SIZE - 1) / CHUNK_SIZE;
    localparam CHUNK_PCOUNT_WIDTH = $clog2(CHUNK_SIZE) + 1;
    localparam TOTAL_PCOUNT_MAX_INDEX = $clog2(REQ_DATA_WIDTH);

    logic [REQ_DATA_WIDTH-1:0]       count_reg;
    logic [TOTAL_PCOUNT_MAX_INDEX:0] popcount_reg;
    logic [TOTAL_PCOUNT_MAX_INDEX:0] npopcount_comb;

    logic [CHUNK_PCOUNT_WIDTH-1:0] chunk_popcounts [NUM_CHUNKS-1:0];

    function automatic [CHUNK_PCOUNT_WIDTH-1:0] calculate_chunk_popcount (
        input logic [CHUNK_SIZE-1:0] chunk_data
    );
        logic [CHUNK_PCOUNT_WIDTH-1:0] p_count;
        p_count = '0;
        for (int i = 0; i < CHUNK_SIZE; i = i + 1) begin
            p_count = p_count + chunk_data[i];
        end
        return p_count;
    endfunction

    // Generate block to calculate popcount for each chunk
    genvar i_chunk_gen_var; // Changed genvar name for clarity
    generate
        for (i_chunk_gen_var = 0; i_chunk_gen_var < NUM_CHUNKS; i_chunk_gen_var = i_chunk_gen_var + 1) begin : gen_chunk_pcount_block
            logic [CHUNK_SIZE-1:0] current_chunk_data; // Local to each generated block

            // Combinational logic to define current_chunk_data
            always_comb begin
                // current_start_bit is effectively constant for each generated always_comb block
                integer current_start_bit_local = i_chunk_gen_var * CHUNK_SIZE;
                for (int k_bit_loop = 0; k_bit_loop < CHUNK_SIZE; k_bit_loop = k_bit_loop + 1) begin
                    if (current_start_bit_local + k_bit_loop < REQ_DATA_WIDTH) begin
                        current_chunk_data[k_bit_loop] = in_m0[current_start_bit_local + k_bit_loop];
                    end else begin
                        current_chunk_data[k_bit_loop] = 1'b0; // Pad with zeros
                    end
                end
            end
            assign chunk_popcounts[i_chunk_gen_var] = calculate_chunk_popcount(current_chunk_data);
        end
    endgenerate

    always_comb begin
        npopcount_comb = '0;
        for (integer j_sum = 0; j_sum < NUM_CHUNKS; j_sum = j_sum + 1) begin
            npopcount_comb = npopcount_comb + chunk_popcounts[j_sum];
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            count_reg    <= '0;
            popcount_reg <= '0;
        end else if (in_valid) begin
            count_reg    <= in_count;
            popcount_reg <= npopcount_comb;
        end else begin
            count_reg    <= '0;
            popcount_reg <= '0;
        end
    end

    assign out_vec = count_reg + popcount_reg;

endmodule