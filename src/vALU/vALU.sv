(* keep_hierarchy = "yes" *) module vALU
  import opcodes::*;
  import rvvLitePkg::*;
#(
    // Parameters define the capabilities of THIS vALU instance
    parameter REQ_DATA_WIDTH    = DATA_WIDTH,
    parameter RESP_DATA_WIDTH   = DATA_WIDTH,
    parameter REQ_ADDR_WIDTH    = ADDR_WIDTH, // For tagging, not directly used by core ALU logic if addr is just piped
    parameter REQ_VL_WIDTH      = VL_BITS ,
    parameter REQ_BYTE_EN_WIDTH = DATA_WIDTH/8,
    parameter ID                = 0
)
(
    input  logic clk,
    input  logic rst,
    output logic ready,
    input  valu_req_t  req,
    output valu_resp_t resp
);

    localparam PIPE_DEPTH = 7;
    // Parameters define the capabilities of THIS vALU instance
    localparam AND_OR_XOR_ENABLE = VALU__AND_OR_XOR_ENABLE[ID];
    localparam ADD_SUB_ENABLE    = VALU__ADD_SUB_ENABLE[ID];
    localparam MIN_MAX_ENABLE    = VALU__MIN_MAX_ENABLE[ID];
    localparam VEC_MOVE_ENABLE   = VALU__VEC_MOVE_ENABLE[ID];
    localparam WHOLE_REG_ENABLE  = VALU__WHOLE_REG_ENABLE[ID];
    localparam WIDEN_ADD_ENABLE  = VALU__WIDEN_ADD_ENABLE[ID]; // For vWiden when used with Adder
    localparam REDUCTION_ENABLE  = VALU__REDUCTION_ENABLE[ID];
    localparam MULT_ENABLE       = VALU__MULT_ENABLE[ID];
    localparam SHIFT_ENABLE      = VALU__SHIFT_ENABLE[ID]; // If shifts are separate or part of MULT path
    localparam MULH_SR_ENABLE    = VALU__MULH_SR_ENABLE[ID]; // More specific multiplier capabilities
    localparam MULH_SR_32_ENABLE = VALU__MULH_SR_32_ENABLE[ID];
    localparam NARROW_ENABLE     = VALU__NARROW_ENABLE[ID];
    localparam WIDEN_MUL_ENABLE  = VALU__WIDEN_MUL_ENABLE[ID]; // For vWiden when used with Multiplier
    localparam SLIDE_ENABLE      = VALU__SLIDE_ENABLE[ID];
    localparam SLIDE_N_ENABLE    = VALU__SLIDE_N_ENABLE[ID]; // If slide can handle non-one shifts
    localparam MULT64_ENABLE     = VALU__MULT64_ENABLE[ID]; // Specific 64-bit multiplier capability
    localparam SHIFT64_ENABLE    = VALU__SHIFT64_ENABLE[ID]; // Specific 64-bit shifter capability
    localparam MASK_ENABLE       = VALU__MASK_ENABLE[ID]; // For vMCmp_en, vMOP_en
    localparam MASK_ENABLE_EXT   = VALU__MASK_ENABLE_EXT[ID]; // For vMerge_en, vFirst_Popc_en, vAddSubCarry_en
    localparam FXP_ENABLE        = VALU__FXP_ENABLE[ID];
    localparam ENABLE_64_BIT     = VALU__ENABLE_64_BIT[ID]; // General 64-bit operation capability
    localparam EN_128_MUL        = VALU__EN_128_MUL[ID];


    // Pipelining Logic
    logic [PIPE_DEPTH:0] s_start, s_end;
    logic [PIPE_DEPTH:0] [REQ_BYTE_EN_WIDTH-1:0] s_be;
    logic [PIPE_DEPTH:0] [1:0] s_vxrm;
    logic [PIPE_DEPTH:0] [SEW_WIDTH-1:0] s_sew;
    logic [PIPE_DEPTH:0] [REQ_VL_WIDTH-1:0] s_vl;
    logic [PIPE_DEPTH:0] [REQ_ADDR_WIDTH-$clog2(REGISTER_COUNT)-1:0] s_off_pipe;
    logic [PIPE_DEPTH:0] [$clog2(REGISTER_COUNT)-1:0] s_out_addr_regidx;
    logic [PIPE_DEPTH:0] [REQ_DATA_WIDTH-1:0] s_data0;
    logic [PIPE_DEPTH:0] [REQ_DATA_WIDTH-1:0] s_data1;
    logic [PIPE_DEPTH:0] [REQ_BYTE_EN_WIDTH-1:0] s_avl_be;
    logic [PIPE_DEPTH:0] s_turn;
    logic [PIPE_DEPTH:0] s_valid;
    logic [PIPE_DEPTH:0] [REQ_ADDR_WIDTH-1:0] s_vr_idx;

    signalDelayer #(.WIDTH(1),                  .DELAY(PIPE_DEPTH))     d0  (.clk(clk), .rst(rst), .in(req.start_flag), .out(s_start));
    signalDelayer #(.WIDTH(1),                  .DELAY(PIPE_DEPTH))     d1  (.clk(clk), .rst(rst), .in(req.end_flag),   .out(s_end));
    signalDelayer #(.WIDTH(REQ_BYTE_EN_WIDTH),  .DELAY(1))              d2  (.clk(clk), .rst(rst), .in(req.be),   .out(s_be[1:0]));
    signalDelayer #(.WIDTH(REQ_BYTE_EN_WIDTH),  .DELAY(PIPE_DEPTH-1))   d3  (.clk(clk), .rst(rst), .in(r_req_ctrl.vWiden_en ? vWiden_be_actual : s_be[1]),   .out(s_be[7:2]));
    signalDelayer #(.WIDTH(2),                  .DELAY(PIPE_DEPTH))     d4  (.clk(clk), .rst(rst), .in(req.vxrm),   .out(s_vxrm));
    signalDelayer #(.WIDTH(SEW_WIDTH),          .DELAY(1))              d5  (.clk(clk), .rst(rst), .in(req.sew),   .out(s_sew[1:0]));
    signalDelayer #(.WIDTH(SEW_WIDTH),          .DELAY(PIPE_DEPTH-1))   d6  (.clk(clk), .rst(rst), .in(r_req_ctrl.vWiden_en ? vWiden_sew_actual : s_sew[1]),   .out(s_sew[7:2]));
    signalDelayer #(.WIDTH(REQ_VL_WIDTH),       .DELAY(PIPE_DEPTH))     d7  (.clk(clk), .rst(rst), .in(req.vl),   .out(s_vl));
    signalDelayer #(.WIDTH(REQ_ADDR_WIDTH-$clog2(REGISTER_COUNT)), .DELAY(PIPE_DEPTH))     d8  (.clk(clk), .rst(rst), .in(req.off),   .out(s_off_pipe));
    signalDelayer #(.WIDTH($clog2(REGISTER_COUNT)), .DELAY(PIPE_DEPTH))                d9  (.clk(clk), .rst(rst), .in(req.addr),   .out(s_out_addr_regidx));
    signalDelayer #(.WIDTH(REQ_DATA_WIDTH),     .DELAY(PIPE_DEPTH))     d10  (.clk(clk), .rst(rst), .in(req.data0), .out(s_data0));
    signalDelayer #(.WIDTH(REQ_DATA_WIDTH),     .DELAY(PIPE_DEPTH))     d11  (.clk(clk), .rst(rst), .in(req.data1), .out(s_data1));
    signalDelayer #(.WIDTH(REQ_BYTE_EN_WIDTH),  .DELAY(PIPE_DEPTH))     d12  (.clk(clk), .rst(rst), .in(req.avl_be), .out(s_avl_be));
    signalDelayer #(.WIDTH(1),                  .DELAY(PIPE_DEPTH))     d13  (.clk(clk), .rst(rst), .in(req.turn), .out(s_turn));
    signalDelayer #(.WIDTH(1),                  .DELAY(PIPE_DEPTH))     d14  (.clk(clk), .rst(rst), .in(req.valid), .out(s_valid));
    signalDelayer #(.WIDTH(REQ_ADDR_WIDTH),     .DELAY(PIPE_DEPTH))     d15  (.clk(clk), .rst(rst), .in(req.vr_idx), .out(s_vr_idx));

    logic vID_in_valid, vAdd_min_max_in_valid, vAndOrXor_in_valid, vMul_in_valid, vMerge_in_valid, vFirstPopc_in_valid, vRed_in_valid, vNarrow_in_valid, fxp_in_valid;
    valu_ctrl_t r_req_ctrl;

    // Internal Wires
    logic [REQ_DATA_WIDTH-1:0] vWiden_in0, vWiden_in1;
    logic [REQ_DATA_WIDTH-1:0] vAdd_in0, vAdd_in1, vMul_in0_final, vMul_in1_final, vAndOrXor_in0, vAndOrXor_in1;

    logic [REQ_DATA_WIDTH-1:0] vMul_data_to_shift; // Output of multiply stage, input to shift stage if separate
    logic [REQ_DATA_WIDTH-1:0] vShift_data_from_rs1; // s_data1 for shifts
    logic [REQ_DATA_WIDTH-1:0] vShift_amount_or_mask; // s_data0 or generated mask for shifts

    logic [SEW_WIDTH-1:0] vWiden_sew_actual, vAdd_sew_actual, vMul_sew_actual, vNarrow_sew_actual;
    logic [REQ_BYTE_EN_WIDTH-1:0] vSlide_outBe_actual, vWiden_be_actual, vNarrow_be_actual, vMCmp_outBe_actual, vRed_outBe_actual;

    logic vAdd_outMask_actual, vAndOrXor_outMask_actual;

    logic [REQ_ADDR_WIDTH-$clog2(REGISTER_COUNT)-1:0] vSlide_outOff_piped;

    logic [REQ_ADDR_WIDTH-1:0] vStartIdx_calc;

    logic [RESP_DATA_WIDTH-1:0] vMerge_outVec, vRed_outVec, vID_outVec, vFirst_Popc_outVec, vAdd_outVec, vAndOrXor_outVec, vMul_outVec_raw, vShift_outVec, vSlide_outVec, vNarrow_outVec_final, fxp_out_final;
    logic vMerge_outValid, vRed_outValid, vID_outValid, vFirst_Popc_outValid, vAdd_outValid, vAndOrXor_outValid, vMul_outValid_raw, vShift_outValid, vSlide_outValid, vNarrow_outValid_final;
    logic vMove_outWReg_actual;
    logic vMul_outNarrow_flag;

    logic [REQ_BYTE_EN_WIDTH-1:0] vAAdd_vd_fxp, vAAdd_vd1_fxp, vMul_vd_fxp, vMul_vd1_fxp, vMul_vd10_fxp;
    logic vMove_outSca_actual;
    logic vAAdd_outFXP_flag, vMul_outFXP_flag;

    always @(posedge clk) begin
        if (rst) begin
            vID_in_valid <= 'b0;
            vAdd_min_max_in_valid <= 'b0;
            vAndOrXor_in_valid <= 'b0;
            vMul_in_valid <= 'b0;
            vMerge_in_valid <= 'b0;
            vFirstPopc_in_valid <= 'b0;
            vRed_in_valid <= 'b0;
            vNarrow_in_valid <= 'b0;
            fxp_in_valid <= 'b0;
            r_req_ctrl <= 'b0;
        end
        else begin 
            vID_in_valid <= req.valid & req.ctrl.vID_en;
            vAdd_min_max_in_valid <= req.valid & (req.ctrl.vAdd_en | (MIN_MAX_ENABLE & req.ctrl.vMinMax_en) | (MASK_ENABLE & req.ctrl.vMCmp_en) | (FXP_ENABLE & req.ctrl.vAAdd_en) | (MASK_ENABLE_EXT & req.ctrl.vAddSubCarry_en));
            vAndOrXor_in_valid <= req.valid & (req.ctrl.vAndOrXor_en | (VEC_MOVE_ENABLE & req.ctrl.vMove_en) | (MASK_ENABLE & req.ctrl.vMOP_en));
            vMul_in_valid <= req.valid & ( (MULT_ENABLE & (req.ctrl.vMul_en | (FXP_ENABLE & req.ctrl.vSMul_en))) |
                                         (SHIFT_ENABLE & (req.ctrl.vShiftLeft | req.ctrl.vShiftRight | (FXP_ENABLE & req.ctrl.vSShift_en))) );
            vMerge_in_valid <= req.valid & req.ctrl.vMerge_en;
            vFirstPopc_in_valid <= req.valid & req.ctrl.vFirst_Popc_en;
            vRed_in_valid <= req.valid & (req.ctrl.vRedAndOrXor_en | req.ctrl.vRedSum_min_max_en);
            vNarrow_in_valid <= req.valid & req.ctrl.vNarrow_en & vMul_outNarrow_flag;
            fxp_in_valid <= req.valid & (vAAdd_outFXP_flag | vMul_outFXP_flag);
            r_req_ctrl <= req.ctrl;
        end 
    end

    logic [REQ_DATA_WIDTH-1:0] vSlide_in1_data;
    logic [REQ_ADDR_WIDTH-$clog2(REGISTER_COUNT)-1:0] vSlide_off_calculated;
    logic [($clog2(REQ_DATA_WIDTH/8))-1:0] vSlide_shift_calculated;
    logic vSlide_en, vSlide_opSel, vSlide_insert;
    always @(posedge clk) begin
        if (rst) begin
            vSlide_in1_data <= 'b0;
            vSlide_off_calculated <= 'b0;
            vSlide_shift_calculated <= 'b0;
            vSlide_en <= 'b0;
            vSlide_opSel <= 'b0;
            vSlide_insert <= 'b0;
        end 
        else begin
            vSlide_in1_data <= r_req_ctrl.vSlide_insert ? s_data0[1] : '0;;
            vSlide_off_calculated <= SLIDE_N_ENABLE ? (r_req_ctrl.vSlide_insert ? '0 : s_data0[1] >> ($clog2(REQ_DATA_WIDTH/8) - s_sew[1])) : 'b0; // This offset needs care;
            vSlide_shift_calculated <= SLIDE_N_ENABLE ? (r_req_ctrl.vSlide_insert ? ( (1 << s_sew[1]) % (REQ_DATA_WIDTH/8) ) : (s_data0[1][$clog2(REQ_DATA_WIDTH/8)-1:0] << s_sew[1]) % (REQ_DATA_WIDTH/8)) : (1 << s_sew[1]) % (REQ_DATA_WIDTH/8);
            vSlide_en <= s_valid[1] & r_req_ctrl.vSlide_en;
            vSlide_opSel <= r_req_ctrl.vSlide_opSel;
            vSlide_insert <= r_req_ctrl.vSlide_insert;
        end
    end

    // --- Combinational Logic for preparing inputs to units ---
    assign ready = 1'b1; // Placeholder, real ALU would have busy logic

    assign vAndOrXor_in0 = (r_req_ctrl.vMoveXS_en | r_req_ctrl.vMoveWhole_en) ? s_data1[1] : s_data0[1];
    assign vAndOrXor_in1 = r_req_ctrl.vMove_en ? vAndOrXor_in0 : s_data1[1];

    assign vAdd_sew_actual = r_req_ctrl.vWiden_en ? vWiden_sew_actual : s_sew[1];
    assign vAdd_in0 = r_req_ctrl.vWiden_en ? vWiden_in0 : s_data0[1];
    assign vAdd_in1 = r_req_ctrl.vWiden_en ? vWiden_in1 : s_data1[1];

    // Inputs for Multiply/Shift Path
    assign vShift_data_from_rs1 = s_data1[1]; // vs2 is data to be shifted
    assign vShift_amount_or_mask = s_data0[1]; // vs1 or imm is shift amount / mask
    assign vMul_data_to_shift = s_data1[1]; // For multiply, vs2 is one operand

    assign vMul_sew_actual = r_req_ctrl.vWiden_en ? vWiden_sew_actual : s_sew[1];
    assign vMul_in0_final = r_req_ctrl.vWiden_en ? vWiden_in1 : ( (r_req_ctrl.vShiftLeft || r_req_ctrl.vShiftRight) ? vShift_data_from_rs1 : s_data1[1] ); // If widening, data comes from vWiden, else from inputs
    assign vMul_in1_final = r_req_ctrl.vWiden_en ? vWiden_in0 : ( (r_req_ctrl.vShiftLeft || r_req_ctrl.vShiftRight) ? vShift_amount_or_mask : s_data0[1] );


    assign vStartIdx_calc = s_vr_idx[1] << (3 - s_sew[1]); // 3 assumes max SEW of 64 (2^3 bytes)

    // --- Conditionally Generate Hardware Units ---
    generate
        // Widen Unit (Enabled if either Widening Add or Widening Mul is possible)
        if (WIDEN_ADD_ENABLE || WIDEN_MUL_ENABLE) begin : widen_unit_gen
            vWiden #(.REQ_BYTE_EN_WIDTH(REQ_BYTE_EN_WIDTH), .RESP_DATA_WIDTH(RESP_DATA_WIDTH))
            vWiden_0_inst (
                .in_vec    (s_data0[1]), .in_turn   (s_turn[1]), .in_be     (s_be[1]),
                .in_signed (r_req_ctrl.vSigned_op0), .in_sew    (s_sew[1]),
                .out_be    (vWiden_be_actual), .out_vec   (vWiden_in0), .out_sew   (vWiden_sew_actual) );
            vWiden #(.REQ_BYTE_EN_WIDTH(REQ_BYTE_EN_WIDTH), .RESP_DATA_WIDTH(RESP_DATA_WIDTH))
            vWiden_1_inst (
                .in_vec    (s_data1[1]), .in_turn   (s_turn[1]), .in_be     (s_be[1]),
                .in_signed (r_req_ctrl.vSigned_op1), .in_sew    (s_sew[1]),
                .out_be    ( /* NC */ ), .out_vec   (vWiden_in1), .out_sew   ( /* NC, use vWiden_sew_actual */ ) );
        end else begin : widen_unit_disabled
            assign vWiden_be_actual = s_be[1]; // Pass through BE
            assign vWiden_in0 = s_data0[1];    // Pass through data
            assign vWiden_in1 = s_data1[1];    // Pass through data
            assign vWiden_sew_actual = s_sew[1]; // Pass through SEW
        end

        // Adder/Subtractor/Comparator/MinMax/ID Unit
        if (ADD_SUB_ENABLE) begin : add_sub_unit_gen
            // VID Unit
            vID #(.REQ_BYTE_EN_WIDTH(REQ_BYTE_EN_WIDTH), .REQ_ADDR_WIDTH(REQ_ADDR_WIDTH), .RESP_DATA_WIDTH(RESP_DATA_WIDTH), .ENABLE_64_BIT(ENABLE_64_BIT))
            vID_inst ( .clk (clk), .rst (rst), .in_sew (s_sew[1]), .in_valid (vID_in_valid),
                       .in_addr (s_out_addr_regidx[1]), .in_start_idx(vStartIdx_calc), .out_vec (vID_outVec), .out_valid (vID_outValid) );

            // Main Adder Unit (also handles MinMax, Compare, AvgAdd, AddCarry if enabled)
            vAdd_min_max #(.REQ_DATA_WIDTH(REQ_DATA_WIDTH), .RESP_DATA_WIDTH(RESP_DATA_WIDTH), .REQ_ADDR_WIDTH(REQ_ADDR_WIDTH),
                           .SEW_WIDTH(SEW_WIDTH), .OPSEL_WIDTH(9), .MIN_MAX_ENABLE(MIN_MAX_ENABLE), .MASK_ENABLE(MASK_ENABLE),
                           .FXP_ENABLE(FXP_ENABLE), .MASK_ENABLE_EXT(MASK_ENABLE_EXT), .ENABLE_64_BIT(ENABLE_64_BIT))
            vAdd_min_max_inst (
                .clk (clk), .rst (rst), .in_vec0 (vAdd_in0), .in_vec1 (vAdd_in1), .in_sew (vAdd_sew_actual),
                .in_valid (vAdd_min_max_in_valid),
                .in_opSel ({ (MASK_ENABLE & r_req_ctrl.vMCmp_en), r_req_ctrl.vMask_opSel, (MIN_MAX_ENABLE & r_req_ctrl.vMinMax_en), r_req_ctrl.vMinMax_opSel, r_req_ctrl.vSigned_op0, r_req_ctrl.vAdd_opSel}),
                .in_addr (s_out_addr_regidx[1]), .in_start_idx(vStartIdx_calc[$bits(vStartIdx_calc)-1:0]), .in_req_start(s_start[1]), .in_req_end (s_end[1]),
                .in_be (s_be[1]), .in_avg (FXP_ENABLE & r_req_ctrl.vAAdd_en), .in_carry (MASK_ENABLE_EXT & r_req_ctrl.vAddSubCarry_en),
                .in_mask (~r_req_ctrl.is_masked_op_vm), .out_vec (vAdd_outVec), .out_valid (vAdd_outValid),
                .out_be (vMCmp_outBe_actual), .out_mask (vAdd_outMask_actual), .out_vd (vAAdd_vd_fxp), .out_vd1 (vAAdd_vd1_fxp), .out_fxp (vAAdd_outFXP_flag) );
        end else begin : add_sub_unit_disabled
            assign vID_outVec = '0; assign vID_outValid = 1'b0;
            assign vAdd_outVec = '0; assign vAdd_outValid = 1'b0;
            assign vMCmp_outBe_actual = '0; assign vAdd_outMask_actual = 1'b0;
            assign vAAdd_vd_fxp = '0; assign vAAdd_vd1_fxp = '0; assign vAAdd_outFXP_flag = 1'b0;
        end

        // And/Or/Xor/Move/MaskLogical Unit
        if (AND_OR_XOR_ENABLE) begin : and_or_xor_unit_gen
            vAndOrXor #(.REQ_DATA_WIDTH(REQ_DATA_WIDTH), .RESP_DATA_WIDTH(RESP_DATA_WIDTH), .REQ_ADDR_WIDTH(REQ_ADDR_WIDTH),
                        .OPSEL_WIDTH(3), .WHOLE_REG_ENABLE(WHOLE_REG_ENABLE), .VEC_MOVE_ENABLE(VEC_MOVE_ENABLE), .MASK_ENABLE(MASK_ENABLE))
            vAndOrXor_inst (
                .clk (clk), .rst (rst), .in_vec0 (vAndOrXor_in0), .in_vec1 (vAndOrXor_in1), .in_opSel (r_req_ctrl.vAndOrXor_opSel),
                .in_valid (vAndOrXor_in_valid),
                .in_addr (s_out_addr_regidx[1]), .in_w_reg (WHOLE_REG_ENABLE & r_req_ctrl.vMoveWhole_en),
                .in_sca (VEC_MOVE_ENABLE & r_req_ctrl.vMoveXS_en), .in_mask (MASK_ENABLE & r_req_ctrl.vMOP_en & ~r_req_ctrl.is_masked_op_vm),
                .out_vec (vAndOrXor_outVec), .out_valid (vAndOrXor_outValid), .out_w_reg (vMove_outWReg_actual),
                .out_sca (vMove_outSca_actual), .out_mask (vAndOrXor_outMask_actual) );
        end else begin : and_or_xor_unit_disabled
            assign vAndOrXor_outVec = '0; assign vAndOrXor_outValid = 1'b0;
            assign vMove_outWReg_actual = 1'b0; assign vMove_outSca_actual = 1'b0; assign vAndOrXor_outMask_actual = 1'b0;
        end

        // Multiplier/Shifter Unit
        if (MULT_ENABLE || SHIFT_ENABLE) begin : mult_shift_unit_gen
            // This assumes vMul handles both multiplication and standard shifts.
            // If shifts are a completely separate unit, instantiate it separately guarded by SHIFT_ENABLE.
            vMul #(.REQ_DATA_WIDTH(REQ_DATA_WIDTH), .RESP_DATA_WIDTH(RESP_DATA_WIDTH), .REQ_ADDR_WIDTH(REQ_ADDR_WIDTH),
                   .SEW_WIDTH(SEW_WIDTH), .MULH_SR_ENABLE(MULH_SR_ENABLE), .WIDEN_MUL_ENABLE(WIDEN_MUL_ENABLE), .NARROW_ENABLE(NARROW_ENABLE),
                   .MULH_SR_32_ENABLE(MULH_SR_32_ENABLE), .MUL64_ENABLE(MULT64_ENABLE), .FXP_ENABLE(FXP_ENABLE),
                   .ENABLE_64_BIT(ENABLE_64_BIT), .EN_128_MUL(EN_128_MUL), .OPSEL_WIDTH(2)) // Assuming 2-bit opSel for Mul/Shift types
            vMul_inst (
                .clk (clk), .rst (rst), .in_vec0 (vMul_in0_final), .in_vec1 (vMul_in1_final), .in_sew (vMul_sew_actual),
                .in_valid (vMul_in_valid),
                .in_opSel (r_req_ctrl.vMul_opSel), .in_widen ( (WIDEN_ADD_ENABLE || WIDEN_MUL_ENABLE) & r_req_ctrl.vWiden_en), // Widen flag if widening is enabled and op requests it
                .in_addr (s_out_addr_regidx[1]), .in_fxp_s (FXP_ENABLE & r_req_ctrl.vSShift_en), .in_fxp_mul (FXP_ENABLE & r_req_ctrl.vSMul_en),
                .in_sr (SHIFT_ENABLE & (r_req_ctrl.vShiftRight | r_req_ctrl.vShiftLeft)), .in_sr_64 (SHIFT64_ENABLE & SHIFT_ENABLE & r_req_ctrl.func6[3] & (&s_sew[1])), // Example, adapt from original vALU shift logic
                .in_or_top(1'b0), .in_vd10(1'b0), .in_shift('0), // These shift helpers need to be properly connected if used by vMul for complex shifts
                .in_narrow (NARROW_ENABLE & r_req_ctrl.vNarrow_en), // For narrowing shifts
                .out_vec (vMul_outVec_raw), .out_valid (vMul_outValid_raw), .out_vd (vMul_vd_fxp), .out_vd1 (vMul_vd1_fxp),
                .out_vd10 (vMul_vd10_fxp), .out_narrow (vMul_outNarrow_flag), .out_fxp (vMul_outFXP_flag) );
            // If shifts are truly separate, vShift_outVec would come from a vShift unit.
            // For now, assume vMul_outVec_raw carries shift results if it's a shift op.
            assign vShift_outVec = (SHIFT_ENABLE & (r_req_ctrl.vShiftLeft | r_req_ctrl.vShiftRight)) ? vMul_outVec_raw : '0;
            assign vShift_outValid = (SHIFT_ENABLE & (r_req_ctrl.vShiftLeft | r_req_ctrl.vShiftRight)) ? vMul_outValid_raw : 1'b0;

        end else begin : mult_shift_unit_disabled
            assign vMul_outVec_raw = '0; assign vMul_outValid_raw = 1'b0;
            assign vShift_outVec = '0; assign vShift_outValid = 1'b0;
            assign vMul_vd_fxp = '0; assign vMul_vd1_fxp = '0; assign vMul_vd10_fxp = '0;
            assign vMul_vd_fxp = '0; assign vMul_vd1_fxp = '0; assign vMul_vd10_fxp = '0;
            assign vMul_outNarrow_flag = 1'b0; assign vMul_outFXP_flag = 1'b0;
        end
        
        // Slide Unit
        if (SLIDE_ENABLE) begin : slide_unit_gen
            vSlide #(.REQ_DATA_WIDTH(REQ_DATA_WIDTH), .RESP_DATA_WIDTH(RESP_DATA_WIDTH), .REQ_ADDR_WIDTH(REQ_ADDR_WIDTH),
                     .ENABLE_64_BIT(ENABLE_64_BIT), .SLIDE_N_ENABLE(SLIDE_N_ENABLE))
            vSlide_inst (
                .clk (clk), .rst (rst), .in_vec0 (s_data1[2]), .in_vec1 (vSlide_in1_data), .in_be (s_be[2]), .in_avl_be(s_avl_be[2]),
                .in_off (vSlide_off_calculated), .in_shift (vSlide_shift_calculated), .in_valid (vSlide_en),
                .in_start (s_start[2]), .in_end (s_end[2]), .in_opSel (vSlide_opSel), .in_insert (vSlide_insert),
                .in_addr (s_out_addr_regidx[2]), .out_be (vSlide_outBe_actual), .out_vec (vSlide_outVec), .out_valid (vSlide_outValid),
                .out_off (vSlide_outOff_piped) );
        end else begin : slide_unit_disabled
            assign vSlide_outBe_actual = '0; assign vSlide_outVec = '0; assign vSlide_outValid = 1'b0; assign vSlide_outOff_piped = '0;
        end

        // Mask Extended Operations (Merge, First/Popcount)
        if (MASK_ENABLE_EXT) begin : mask_ext_units_gen
            vMerge #(.REQ_DATA_WIDTH(REQ_DATA_WIDTH), .RESP_DATA_WIDTH(RESP_DATA_WIDTH), .REQ_ADDR_WIDTH(REQ_ADDR_WIDTH))
            vMerge_inst( .clk (clk), .rst (rst), .in_mask (s_be[1]), .in_vec0 (s_data0[1]), .in_vec1 (s_data1[1]),
                         .in_valid (vMerge_in_valid), .in_addr (s_out_addr_regidx[1]),
                         .out_vec (vMerge_outVec), .out_valid (vMerge_outValid) );

            // First/Popcount unit (also needs MASK_ENABLE for its inputs often)
            if (MASK_ENABLE) begin : first_popc_gen
                vFirst_Popc #(.REQ_DATA_WIDTH(REQ_DATA_WIDTH), .RESP_DATA_WIDTH(RESP_DATA_WIDTH), .REQ_ADDR_WIDTH(REQ_ADDR_WIDTH))
                vFirstPopc_inst ( .clk (clk), .rst (rst), .in_m0 (s_data1[1]), .in_valid (vFirstPopc_in_valid),
                                 .in_end (s_end[1]), .in_addr (s_out_addr_regidx[1]), .in_start_idx(vStartIdx_calc), .in_opSel (r_req_ctrl.vFirst_Popc_opSel),
                                 .out_vec (vFirst_Popc_outVec), .out_valid (vFirst_Popc_outValid) );
            end else begin : first_popc_disabled_due_to_mask_enable
                assign vFirst_Popc_outVec = '0; assign vFirst_Popc_outValid = 1'b0;
            end
        end else begin : mask_ext_units_disabled
            assign vMerge_outVec = '0; assign vMerge_outValid = 1'b0;
            assign vFirst_Popc_outVec = '0; assign vFirst_Popc_outValid = 1'b0;
        end

        // Reduction Unit
        if (REDUCTION_ENABLE) begin : red_unit_gen
            vReduction #(.REQ_DATA_WIDTH(REQ_DATA_WIDTH), .RESP_DATA_WIDTH(RESP_DATA_WIDTH), .REQ_ADDR_WIDTH(REQ_ADDR_WIDTH), .ENABLE_64_BIT(ENABLE_64_BIT))
            vRed_inst ( .clk (clk), .rst (rst), .in_vec0 (s_data1[1]), .in_vec1 (s_data0[1]),
                        .in_valid (vRed_in_valid),
                        .in_lop_sum (r_req_ctrl.vRedAndOrXor_en), .in_start (s_start[1]), .in_end (s_end[1]),
                        .in_opSel (r_req_ctrl.vRed_opSel), .in_sew (s_sew[1]), .in_addr (s_out_addr_regidx[1]),
                        .out_vec (vRed_outVec), .out_valid (vRed_outValid), .out_be (vRed_outBe_actual) );
        end else begin : red_unit_disabled
            assign vRed_outVec = '0; assign vRed_outValid = 1'b0; assign vRed_outBe_actual = '0;
        end

        // Narrowing Unit (output stage)
        if (NARROW_ENABLE) begin : narrow_unit_gen
            // Input to narrow typically comes from Mul/Shift unit if it's a narrowing shift
            vNarrow #(.REQ_BYTE_EN_WIDTH(REQ_BYTE_EN_WIDTH), .REQ_ADDR_WIDTH(REQ_ADDR_WIDTH), .REQ_DATA_WIDTH(REQ_DATA_WIDTH), .RESP_DATA_WIDTH(RESP_DATA_WIDTH), .ENABLE_64_BIT(ENABLE_64_BIT))
            vNarrow_inst ( .clk (clk), .rst (rst), .in_vec (vMul_outVec_raw), // Assuming narrow follows mul/shift path
                           .in_valid (vNarrow_in_valid), // Enabled by control AND if mul/shift produced narrowable output
                           .in_sew (s_sew[PIPE_DEPTH]), .in_be (s_be[7]), .in_turn(s_turn[1]), // Pipelined inputs
                           .out_be (vNarrow_be_actual), .out_vec (vNarrow_outVec_final),
                           .out_valid (vNarrow_outValid_final), .out_sew (vNarrow_sew_actual) );
        end else begin : narrow_unit_disabled
            assign vNarrow_be_actual = '0; assign vNarrow_outVec_final = '0;
            assign vNarrow_outValid_final = 1'b0; assign vNarrow_sew_actual = '0;
        end

        // Fixed-Point Rounding Unit
        if (FXP_ENABLE) begin : fxp_unit_gen
            fxp_round #(.DATA_WIDTH(REQ_DATA_WIDTH))
            fxp_inst ( .clk(clk), .rst (rst), .vxrm (s_vxrm[7]), // Pipelined vxrm
                       .in_valid(fxp_in_valid),
                       .vec_in  (vAAdd_outFXP_flag ? vAdd_outVec : vMul_outVec_raw),
                       .v_d     (vAAdd_outFXP_flag ? vAAdd_vd_fxp : vMul_vd_fxp),
                       .v_d1    (vAAdd_outFXP_flag ? vAAdd_vd1_fxp : vMul_vd1_fxp),
                       .v_d10   (vAAdd_outFXP_flag ? vAAdd_vd1_fxp : vMul_vd10_fxp), // Check if vAAdd needs vd10
                       .vec_out (fxp_out_final) );
        end else begin : fxp_unit_disabled
            assign fxp_out_final = (vAAdd_outFXP_flag ? vAdd_outVec : vMul_outVec_raw); // Pass through if FXP disabled and one was active
                                                                                      // This will be selected by final mux
        end
    endgenerate
 
    wire current_resp_valid;
    assign current_resp_valid =  vAdd_outValid | vAndOrXor_outValid | vMul_outValid_raw | vShift_outValid |
                          vSlide_outValid | vMerge_outValid | vFirst_Popc_outValid |
                          vRed_outValid | vID_outValid | vNarrow_outValid_final;
    // --- Output Logic ---
    
    always @(posedge clk) begin
        if (rst) begin
            resp.addr <= '0;
            resp.mask <= 1'b0; resp.scalar <= 1'b0; resp.whole_reg <= 1'b0;
            resp.narrow <= 1'b0;
            resp.sew <= '0;
            resp.vl <= '0;
            resp.be <= '0;
            resp.start_flag <= 1'b0;
            resp.end_flag <= 1'b0;
            resp.off <= '0;
            resp.valid <= 'b0;
        end 
        else begin
          // Pipeline shift
            if (NARROW_ENABLE && vNarrow_outValid_final) begin
                resp.sew <= vNarrow_sew_actual;
                resp.be  <= vNarrow_be_actual;
                resp.narrow <= 1'b1;
            end else begin
                resp.sew <= s_sew[PIPE_DEPTH];
                resp.narrow <= 1'b0;
                if (vSlide_outValid)       resp.be <= vSlide_outBe_actual;
                else if (vAdd_outValid && r_req_ctrl.vMCmp_en && MASK_ENABLE) resp.be <= vMCmp_outBe_actual;
                else if (vRed_outValid)    resp.be <= vRed_outBe_actual;
                else                       resp.be <= s_be[PIPE_DEPTH];
            end
            resp.vl     <= s_vl[PIPE_DEPTH];
            resp.start_flag  <= s_start[PIPE_DEPTH];
            resp.end_flag    <= s_end[PIPE_DEPTH];
            resp.addr   <= s_out_addr_regidx[PIPE_DEPTH];
            resp.off    <= (SLIDE_ENABLE && vSlide_outValid) ? vSlide_outOff_piped : s_off_pipe[PIPE_DEPTH];

            // Determine active output based on valid flags from units (assuming one is active)
       
            resp.mask  <= (vAndOrXor_outValid & vAndOrXor_outMask_actual) | (vAdd_outValid & vAdd_outMask_actual & MASK_ENABLE & r_req_ctrl.vMCmp_en);
            resp.scalar   <= (vFirst_Popc_outValid & MASK_ENABLE_EXT & MASK_ENABLE & r_req_ctrl.vFirst_Popc_en) | (vMove_outSca_actual & VEC_MOVE_ENABLE & r_req_ctrl.vMoveXS_en);
            resp.whole_reg <= (WHOLE_REG_ENABLE & vMove_outWReg_actual & VEC_MOVE_ENABLE & r_req_ctrl.vMoveWhole_en);
            resp.valid <= current_resp_valid;
        end
    end
    
    always @(posedge clk) begin
        if (rst) begin
            resp.data <= 'b0;
        end 
        else if (NARROW_ENABLE && vNarrow_outValid_final)
            resp.data <= vNarrow_outVec_final;
        else if (FXP_ENABLE && (vAAdd_outFXP_flag | vMul_outFXP_flag) && (vAdd_outValid || vMul_outValid_raw))
            resp.data <= fxp_out_final;
        else if (vAndOrXor_outValid)    resp.data <= vAndOrXor_outVec;
        else if (vSlide_outValid)       resp.data <= vSlide_outVec;
        else if (vMerge_outValid)       resp.data <= vMerge_outVec;
        else if (vFirst_Popc_outValid)  resp.data <= vFirst_Popc_outVec;
        else if (vRed_outValid)         resp.data <= vRed_outVec;
        else if (vID_outValid)          resp.data <= vID_outVec;
        else if (vAdd_outValid)         resp.data <= vAdd_outVec;
        else if (vMul_outValid_raw)     resp.data <= vMul_outVec_raw; // Includes shift results if shifts use this path
        // else if (vShift_outValid)    resp.data <= vShift_outVec; // If shifts are a separate output
        else                            resp.data <= '0;
    end
    
    
endmodule