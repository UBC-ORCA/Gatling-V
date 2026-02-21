module vAdd_unit_block 
  #(
    parameter REQ_DATA_WIDTH = 64,
    parameter RESP_DATA_WIDTH = 64,
    parameter SEW_WIDTH = 2,
    parameter OPSEL_WIDTH = 5,
    parameter ENABLE_64_BIT = 1
  ) 
  (
    input  logic clk,
    input  logic rst,
    input  logic [REQ_DATA_WIDTH-1:0] vec0,
    input  logic [REQ_DATA_WIDTH-1:0] vec1,
    input  logic [REQ_DATA_WIDTH-1:0] carry,
    input  logic [SEW_WIDTH-1:0] sew,
    input  logic [OPSEL_WIDTH-1:0] opSel,
    output logic [RESP_DATA_WIDTH+16:0] result
    );

    genvar i;

    logic [REQ_DATA_WIDTH-1:0] w_vec0, w_vec1;
    logic [REQ_DATA_WIDTH+(1+1)*8-1:0] w_op0, w_op1;
    logic [REQ_DATA_WIDTH/8-1:0] v0_sgn, v1_sgn;
    logic [REQ_DATA_WIDTH/8-1:0] v0_ext, v1_ext;

    /*
     * opSel[1:0] == vAdd_opSel
     * vMinMax_en or vMComp_en (1) -> vAdd_opSel = 2'b10 ->  vec0 - vec1
     *                         (0) -> 2'b00 (vadd) - 2'b01 (reserved) - 2'b10 (vsub) - 2'b11 (vrsub)
    */
    assign w_vec0 = opSel[1:0] == 2'b11 ? ~vec0 : vec0;
    assign w_vec1 = opSel[1:0] == 2'b10 ? ~vec1 : vec1;

    /* ?????????????????????????
     * opSel[4] == vMinMax_opSel
     * opSel[2] == vSigned_op0
    */
    always_comb begin
      unique casez ({opSel[4], opSel[2]})
        2'b0?: begin
          v0_sgn = {REQ_DATA_WIDTH/8{1'b1}};
          v1_sgn = {REQ_DATA_WIDTH/8{1'b0}};
        end

        2'b10: begin  // vminu, vmaxu
          v0_sgn = {REQ_DATA_WIDTH/8{1'b0}};
          v1_sgn = {REQ_DATA_WIDTH/8{1'b1}};
        end

        2'b11: begin  // vmin, vmax
          for(int i = 0; i < REQ_DATA_WIDTH/8; i++) begin
            v0_sgn[i] =  vec0[8*(i+1)-1];
            v1_sgn[i] = ~vec1[8*(i+1)-1];
          end
        end
      endcase
    end

    /*
     * 0 -> carry-kill if addition        -> ext = 2'b00
     *      carry-generate if subtraction -> ext = 2'b11
     * 1 -> carry-propagate               -> ext = 2'b01 | 2'b10
    */
    logic [8-1:0] msk;

    always_comb begin
      unique case (sew)
        2'b00: msk = 8'b00000000;
        2'b01: msk = 8'b10101010;
        2'b10: msk = 8'b11101110;
        2'b11: msk = ENABLE_64_BIT ? 8'b11111110: 8'b11101110;
      endcase
    end

    /*
     * opSel[1] == is_sub
    */
    for (i = 0; i < REQ_DATA_WIDTH/8; i++) begin
      assign v0_ext[i] = opSel[1];
      assign v1_ext[i] = opSel[1] ^ msk[i];
    end

    for (i = 0; i < (REQ_DATA_WIDTH >= 64 ? 8 : 4); i++) begin
      assign w_op0[(1+8+1)*i +: (1+8+1)] = {v0_sgn[i], w_vec0[8*i +: 8], v0_ext[i]};
      assign w_op1[(1+8+1)*i +: (1+8+1)] = {v1_sgn[i], w_vec1[8*i +: 8], v1_ext[i]};
    end

    if (ENABLE_64_BIT) begin
      assign result = w_op0 + w_op1 + carry;
    end else begin
      assign result[REQ_DATA_WIDTH/2 +: REQ_DATA_WIDTH/2] = w_op0[REQ_DATA_WIDTH/2+8 +: REQ_DATA_WIDTH/2+8] + 
                                                            w_op1[REQ_DATA_WIDTH/2+8 +: REQ_DATA_WIDTH/2+8] + 
                                                            carry[REQ_DATA_WIDTH/2   +: REQ_DATA_WIDTH/2];
      assign result[0 +: REQ_DATA_WIDTH/2]  =   w_op0[0 +: REQ_DATA_WIDTH/2+8] + 
                                                w_op1[0 +: REQ_DATA_WIDTH/2+8] + 
                                                carry[0 +: REQ_DATA_WIDTH/2];
    end

endmodule

(* keep_hierarchy = "yes" *)
module vAdd_unit_block_2stage_simple #(
    parameter REQ_DATA_WIDTH  = 64,
    parameter RESP_DATA_WIDTH = 64,
    parameter SEW_WIDTH       = 2,
    parameter ENABLE_64_BIT   = 1
) (
    input  logic                          clk,
    input  logic                          rst,
    input  logic [REQ_DATA_WIDTH - 1:0]   vec0,
    input  logic [REQ_DATA_WIDTH - 1:0]   vec1,
    input  logic [SEW_WIDTH - 1:0]        sew,
    output logic [RESP_DATA_WIDTH + 16:0] result
);

    genvar i;

    // --- Logic simplified assuming opSel = 'h0 ---
    logic [REQ_DATA_WIDTH + 16 - 1:0] w_op0, w_op1;
    
    // Since opSel[1:0] is 'b00, w_vec0 is vec0 and w_vec1 is vec1.
    // Since opSel[4] and opSel[2] are 'b0, v0_sgn is all 1s and v1_sgn is all 0s.
    // Since opSel[1] is 'b0, v0_ext is all 0s and v1_ext is equal to msk.
    
    logic [7:0] msk;
    always_comb begin
        unique case (sew)
            2'b00: msk = 8'b00000000;
            2'b01: msk = 8'b10101010;
            2'b10: msk = 8'b11101110;
            2'b11: msk = ENABLE_64_BIT ? 8'b11111110 : 8'b11101110;
        endcase
    end
    
    // Assemble the operands for the wide adder
    for (i = 0; i < (REQ_DATA_WIDTH >= 64 ? 8 : 4); i++) begin
        // v0_sgn is 1, v0_ext is 0
        assign w_op0[(1 + 8 + 1) * i +: (1 + 8 + 1)] = {1'b1, vec0[8 * i +: 8], 1'b0};
        // v1_sgn is 0, v1_ext is msk[i]
        assign w_op1[(1 + 8 + 1) * i +: (1 + 8 + 1)] = {1'b0, vec1[8 * i +: 8], msk[i]};
    end
    // --- End of simplified logic ---

    generate
        if (ENABLE_64_BIT) begin
            localparam ADDER_WIDTH = RESP_DATA_WIDTH + 16;
            localparam HALF_WIDTH  = ADDER_WIDTH / 2;

            reg [ADDER_WIDTH - 1:HALF_WIDTH] s1_w_op0_upper;
            reg [ADDER_WIDTH - 1:HALF_WIDTH] s1_w_op1_upper;
            reg                              s1_carry_out_lower;
            reg [HALF_WIDTH - 1:0]           s1_result_lower;

            // Stage 1: Add lower half
            wire [HALF_WIDTH - 1:0] lower_sum;
            wire                    lower_carry_out;
            assign {lower_carry_out, lower_sum} = w_op0[HALF_WIDTH - 1:0] + w_op1[HALF_WIDTH - 1:0];

            // Stage 1 pipeline registers
            always @(posedge clk) begin
                if (rst) begin
                    s1_w_op0_upper     <= '0;
                    s1_w_op1_upper     <= '0;
                    s1_carry_out_lower <= 1'b0;
                    s1_result_lower    <= '0;
                end else begin
                    s1_w_op0_upper     <= w_op0[ADDER_WIDTH - 1:HALF_WIDTH];
                    s1_w_op1_upper     <= w_op1[ADDER_WIDTH - 1:HALF_WIDTH];
                    s1_carry_out_lower <= lower_carry_out;
                    s1_result_lower    <= lower_sum;
                end
            end

            // Stage 2: Add upper half with registered carry
            wire [ADDER_WIDTH - 1:HALF_WIDTH] upper_sum;
            assign upper_sum = s1_w_op0_upper + s1_w_op1_upper + s1_carry_out_lower;

            assign result = {upper_sum, s1_result_lower};

        end else begin
            // Non-64-bit case
            assign result[REQ_DATA_WIDTH / 2 +: REQ_DATA_WIDTH / 2] = w_op0[REQ_DATA_WIDTH / 2 + 8 +: REQ_DATA_WIDTH / 2 + 8] + w_op1[REQ_DATA_WIDTH / 2 + 8 +: REQ_DATA_WIDTH / 2 + 8];
            assign result[0 +: REQ_DATA_WIDTH / 2]                 = w_op0[0 +: REQ_DATA_WIDTH / 2 + 8] + w_op1[0 +: REQ_DATA_WIDTH / 2 + 8];
        end
    endgenerate

endmodule