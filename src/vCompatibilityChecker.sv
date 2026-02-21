module vCompatibilityChecker
  import opcodes::*;
  import rvvLitePkg::*; // This now imports the AOR_SEL_*, ADD_SEL_*, MULSHIFT_SEL_* constants

(
    input wire [INSTR_WIDTH-1:0] instr,
    output valu_ctrl_t alu_ctrl_out,
    output wire vStore_en_out,
    output wire [ALU_COUNT-1:0] is_alu_compatible
);

    // Decoded instruction fields
    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [5:0] funct6;
    logic       vm; // mask bit (instr[25])

    assign opcode = instr[6:0];
    assign funct3 = instr[14:12];
    assign funct6 = instr[31:26];
    assign vm     = instr[25];

    valu_ctrl_t alu_ctrl_signals;
    logic local_vStore_en;
    logic local_n_ary_en;
    logic local_vShift_en_generic; // For the generic shift category from original checker

    // Global capability flags based on parameters
    localparam GBL_AND_OR_XOR_EN = |VALU__AND_OR_XOR_ENABLE;
    localparam GBL_ADD_SUB_EN    = |VALU__ADD_SUB_ENABLE;
    localparam GBL_MIN_MAX_EN    = |VALU__MIN_MAX_ENABLE;
    localparam GBL_VEC_MOVE_EN   = |VALU__VEC_MOVE_ENABLE;
    localparam GBL_WHOLE_REG_EN  = |VALU__WHOLE_REG_ENABLE;
    localparam GBL_REDUCTION_EN  = |VALU__REDUCTION_ENABLE;
    localparam GBL_MULT_EN       = |VALU__MULT_ENABLE;
    localparam GBL_SHIFT_EN      = |VALU__SHIFT_ENABLE;
    localparam GBL_NARROW_EN     = |VALU__NARROW_ENABLE;
    localparam GBL_SLIDE_EN      = |VALU__SLIDE_ENABLE;
    localparam GBL_MASK_EN       = |VALU__MASK_ENABLE;
    localparam GBL_MASK_EXT_EN   = |VALU__MASK_ENABLE_EXT;
    localparam GBL_FXP_EN        = |VALU__FXP_ENABLE;


    always_comb begin
        alu_ctrl_signals = '0; // Default all fields to logic 0
        local_vStore_en = 1'b0;
        local_n_ary_en = 1'b0;
        local_vShift_en_generic = 1'b0;

        // --- Direct decodes from instr fields into alu_control_t ---
        alu_ctrl_signals.op_mnr_funct3 = funct3;
        alu_ctrl_signals.func6 = funct6;
        alu_ctrl_signals.is_masked_op_vm = (vm == 1'b0);

        // Category: Moves
        if (GBL_VEC_MOVE_EN) begin
            // ... (move decoding logic as before) ...
            if (GBL_WHOLE_REG_EN && instr inside {VMV_1_R_V, VMV_2_R_V, VMV_4_R_V, VMV_8_R_V}) begin
                alu_ctrl_signals.vMoveWhole_en = 1'b1;
            end
            if (instr inside {VMV_SX}) begin
                alu_ctrl_signals.vMoveSX_en = 1'b1;
            end
            if (instr inside {VMV_XS}) begin
                alu_ctrl_signals.vMoveXS_en = 1'b1;
            end
            if (instr inside {VMV_VV, VMV_VX, VMV_VI}) begin
                alu_ctrl_signals.vMoveVY_en = (vm == 1'b1);
            end
            alu_ctrl_signals.vMove_en = alu_ctrl_signals.vMoveWhole_en | alu_ctrl_signals.vMoveSX_en | alu_ctrl_signals.vMoveXS_en | alu_ctrl_signals.vMoveVY_en;
        end

        // Category: AND/OR/XOR
        if (GBL_AND_OR_XOR_EN) begin
            if (alu_ctrl_signals.vMove_en) begin // If it's a move operation handled by AndOrXor unit
                alu_ctrl_signals.vAndOrXor_en = 1'b1;
                alu_ctrl_signals.vAndOrXor_opSel = AOR_SEL_MOV; // Use defined constant
            end else if (instr inside {VAND_VV, VAND_VX, VAND_VI, VOR_VV, VOR_VX, VOR_VI, VXOR_VV, VXOR_VX, VXOR_VI}) begin
                alu_ctrl_signals.vAndOrXor_en = 1'b1;
                // Assuming funct6[2:0] directly gives the opSel for VAND, VOR, VXOR
                // or map specific funct3 to defined AOR_SEL_AND, AOR_SEL_OR, AOR_SEL_XOR if pkg defines them based on funct3
                alu_ctrl_signals.vAndOrXor_opSel = funct6[2:0]; // Match original vALU logic
            end
        end

        // Category: Merge, ID as before ...
        if (GBL_MASK_EXT_EN) begin
            if (instr inside {VMERGE_VV, VMERGE_VX, VMERGE_VI}) begin
                alu_ctrl_signals.vMerge_en = 1'b1;
            end
        end
        if (GBL_ADD_SUB_EN) begin
            if (instr inside {VID_V}) begin
                alu_ctrl_signals.vID_en = 1'b1;
            end
        end


        // Category: Add/Sub & related
        if (GBL_ADD_SUB_EN) begin
            if (instr inside {VWADDU_VV, VWADDU_VX, VWADD_VV, VWADD_VX, VWSUBU_VV, VWSUBU_VX, VWSUB_VV, VWSUB_VX}) begin
                alu_ctrl_signals.vWiden_en = 1'b1;
            end
            if (instr inside {VADD_VV, VADD_VX, VADD_VI, VSUB_VV, VSUB_VX, VRSUB_VX, VRSUB_VI,
                              VWADDU_VV, VWADDU_VX, VWADD_VV, VWADD_VX, VWSUBU_VV, VWSUBU_VX, VWSUB_VV, VWSUB_VX}) begin
                alu_ctrl_signals.vAdd_en = 1'b1;
                if (instr inside {VADD_VV, VADD_VX, VADD_VI, VWADDU_VV, VWADDU_VX, VWADD_VV, VWADD_VX}) begin
                    alu_ctrl_signals.vAdd_opSel = ADD_SEL_ADD;
                end else begin
                    alu_ctrl_signals.vAdd_opSel = ADD_SEL_SUB;
                end
                // Signedness logic as before...
                if (alu_ctrl_signals.vWiden_en) begin
                    if (instr inside {VWADD_VV, VWADD_VX, VWSUB_VV, VWSUB_VX}) begin // Signed widening
                        alu_ctrl_signals.vSigned_op0 = 1'b1; alu_ctrl_signals.vSigned_op1 = 1'b1;
                    end else begin // Unsigned widening
                        alu_ctrl_signals.vSigned_op0 = 1'b0; alu_ctrl_signals.vSigned_op1 = 1'b0;
                    end
                end else begin // Non-widening
                    alu_ctrl_signals.vSigned_op0 = funct6[0];
                end
            end
            // Min/Max, Averaging Add/Sub, Add/Sub with Carry as before, using GBL flags...
            if (GBL_MIN_MAX_EN) begin
                if (instr inside {VMINU_VV, VMINU_VX, VMIN_VV, VMIN_VX, VMAXU_VV, VMAXU_VX, VMAX_VV, VMAX_VX}) begin
                    alu_ctrl_signals.vAdd_en = 1'b1;
                    alu_ctrl_signals.vMinMax_en = 1'b1;
                    alu_ctrl_signals.vMinMax_opSel = funct6[1]; // 0 for MIN, 1 for MAX based on funct6[1]
                    if (instr inside {VMIN_VV, VMIN_VX, VMAX_VV, VMAX_VX}) alu_ctrl_signals.vSigned_op0 = 1'b1;
                    else alu_ctrl_signals.vSigned_op0 = 1'b0;
                end
            end
            if (GBL_FXP_EN) begin
                if (instr inside {VAADDU_VV, VAADDU_VX, VAADD_VV, VAADD_VX, VASUBU_VV, VASUBU_VX, VASUB_VV, VASUB_VX}) begin
                    alu_ctrl_signals.vAdd_en = 1'b1;
                    alu_ctrl_signals.vAAdd_en = 1'b1;
                    if (instr inside {VAADDU_VV, VAADDU_VX, VAADD_VV, VAADD_VX}) alu_ctrl_signals.vAdd_opSel = ADD_SEL_ADD;
                    else alu_ctrl_signals.vAdd_opSel = ADD_SEL_SUB;
                    if (instr inside {VAADD_VV, VAADD_VX, VASUB_VV, VASUB_VX}) alu_ctrl_signals.vSigned_op0 = 1'b1;
                    else alu_ctrl_signals.vSigned_op0 = 1'b0;
                end
            end
            if (GBL_MASK_EXT_EN) begin
                 if (instr inside {VADC_VV, VADC_VX, VADC_VI, VMADC_VV, VMADC_VX, VMADC_VI, VSBC_VV, VSBC_VX, VMSBC_VV, VMSBC_VX}) begin
                    alu_ctrl_signals.vAdd_en = 1'b1;
                    alu_ctrl_signals.vAddSubCarry_en = 1'b1;
                    if (instr inside {VADC_VV, VADC_VX, VADC_VI, VMADC_VV, VMADC_VX, VMADC_VI}) alu_ctrl_signals.vAdd_opSel = ADD_SEL_ADD;
                    else alu_ctrl_signals.vAdd_opSel = ADD_SEL_SUB;
                end
            end
        end // End GBL_ADD_SUB_EN

        // Mask Compare Ops
        if (GBL_ADD_SUB_EN && GBL_MASK_EN) begin
            if (instr inside {VMSEQ_VV, VMSEQ_VX, VMSEQ_VI, VMSNE_VV, VMSNE_VX, VMSNE_VI,
                              VMSLE_VV, VMSLE_VX, VMSLE_VI, VMSLEU_VV, VMSLEU_VX, VMSLEU_VI,
                              VMSLT_VV, VMSLT_VX, VMSGT_VX, VMSGT_VI}) begin
                alu_ctrl_signals.vAdd_en = 1'b1;
                alu_ctrl_signals.vMCmp_en = 1'b1;
                alu_ctrl_signals.vAdd_opSel = ADD_SEL_CMP; // Use defined constant
                alu_ctrl_signals.vMask_opSel = funct3;
                 if (instr inside {VMSLE_VV, VMSLE_VX, VMSLE_VI, VMSLT_VV, VMSLT_VX, VMSGT_VX, VMSGT_VI})
                    alu_ctrl_signals.vSigned_op0 = 1'b1;
                else
                    alu_ctrl_signals.vSigned_op0 = 1'b0;
            end
        end

        // Reductions as before...
        if (GBL_REDUCTION_EN) begin
            if (instr inside {VREDAND_VS, VREDOR_VS, VREDXOR_VS}) begin
                alu_ctrl_signals.vRedAndOrXor_en = 1'b1;
                alu_ctrl_signals.vRed_opSel = funct6[2:0];
            end
            if (instr inside {VREDSUM_VS, VREDMINU_VS, VREDMIN_VS, VREDMAXU_VS, VREDMAX_VS}) begin
                alu_ctrl_signals.vRedSum_min_max_en = 1'b1;
                alu_ctrl_signals.vRed_opSel = funct6[2:0];
                if (instr inside {VREDMIN_VS, VREDMAX_VS}) alu_ctrl_signals.vSigned_op0 = 1'b1;
                else alu_ctrl_signals.vSigned_op0 = 1'b0;
            end
        end

        // Category: Multiply/Shift path
        if (GBL_MULT_EN || GBL_SHIFT_EN) begin
            if (GBL_MULT_EN && instr inside {VWMUL_VV, VWMUL_VX, VWMULSU_VV, VWMULSU_VX}) begin
                if (!alu_ctrl_signals.vWiden_en) alu_ctrl_signals.vWiden_en = 1'b1;
            end

            local_vShift_en_generic = instr inside {VSLL_VV, VSLL_VX, VSLL_VI, VSRL_VV, VSRL_VX, VSRL_VI, VSRA_VV, VSRA_VX, VSRA_VI, VNSRL_VV, VNSRL_VX, VNSRL_VI, VSSRL_VV, VSSRL_VX, VSSRL_VI, VSSRA_VV, VSSRA_VX, VSSRA_VI};
            if (GBL_SHIFT_EN && local_vShift_en_generic) begin
                alu_ctrl_signals.vMul_en = 1'b1;
                if (instr inside {VSLL_VV, VSLL_VX, VSLL_VI}) begin
                    alu_ctrl_signals.vShiftLeft = 1'b1;
                    alu_ctrl_signals.vMul_opSel = MULSHIFT_SEL_SLL; // Use defined constant
                end
                if (instr inside {VSRL_VV, VSRL_VX, VSRL_VI, VSRA_VV, VSRA_VX, VSRA_VI, VNSRL_VV, VNSRL_VX, VNSRL_VI, VSSRL_VV, VSSRL_VX, VSSRL_VI, VSSRA_VV, VSSRA_VX, VSSRA_VI}) begin
                    alu_ctrl_signals.vShiftRight = 1'b1;
                    alu_ctrl_signals.vMul_opSel = MULSHIFT_SEL_SR;  // Use defined constant
                                                                    // Specific SR type (SRL/SRA) differentiated in vMul unit by funct6/funct3
                end
                if (GBL_NARROW_EN && instr inside {VNSRL_VV, VNSRL_VX, VNSRL_VI}) begin
                    alu_ctrl_signals.vNarrow_en = 1'b1;
                end
                if (GBL_FXP_EN && instr inside {VSSRL_VV, VSSRL_VX, VSSRL_VI, VSSRA_VV, VSSRA_VX, VSSRA_VI}) begin
                    alu_ctrl_signals.vSShift_en = 1'b1;
                end
            end

            if (GBL_MULT_EN && instr inside {VMUL_VV, VMUL_VX, VMULH_VV, VMULH_VX, VWMUL_VV, VWMUL_VX, VWMULSU_VV, VWMULSU_VX}) begin
                alu_ctrl_signals.vMul_en = 1'b1;
                if (instr inside {VMUL_VV, VMUL_VX, VWMUL_VV, VWMUL_VX}) begin
                    alu_ctrl_signals.vMul_opSel = MULSHIFT_SEL_MUL_LOW; // Use defined constant
                end else begin // VMULH, VWMULSU
                    alu_ctrl_signals.vMul_opSel = MULSHIFT_SEL_MUL_HIGH; // Use defined constant
                end
                // Signedness logic as before...
                if (instr inside {VMULH_VV, VMULH_VX} || (alu_ctrl_signals.vWiden_en && instr inside {VWMUL_VV, VWMUL_VX})) begin
                    alu_ctrl_signals.vSigned_op0 = 1'b1; alu_ctrl_signals.vSigned_op1 = 1'b1;
                end else if (alu_ctrl_signals.vWiden_en && instr inside {VWMULSU_VV, VWMULSU_VX}) begin
                    alu_ctrl_signals.vSigned_op0 = 1'b1; alu_ctrl_signals.vSigned_op1 = 1'b0;
                end else begin
                    alu_ctrl_signals.vSigned_op0 = 1'b1;
                    alu_ctrl_signals.vSigned_op1 = (funct6[3] & funct6[1]) | funct6[0];
                end
            end
            if (GBL_MULT_EN && GBL_FXP_EN && instr inside {VSMUL_VV, VSMUL_VX}) begin
                alu_ctrl_signals.vMul_en = 1'b1;
                alu_ctrl_signals.vSMul_en = 1'b1;
                alu_ctrl_signals.vMul_opSel = MULSHIFT_SEL_MUL_LOW; // Assuming saturating multiply uses the low part of multiplier
                alu_ctrl_signals.vSigned_op0 = 1'b1;
                alu_ctrl_signals.vSigned_op1 = 1'b1;
            end
        end // End GBL_MULT_EN || GBL_SHIFT_EN

        // Slide, Mask Logical, First/Popcount as before, using GBL flags...
        if (GBL_SLIDE_EN) begin
            if (instr inside {VSLIDE1UP_VX, VSLIDE1DOWN_VX, VSLIDEUP_VX, VSLIDEUP_VI, VSLIDEDOWN_VX, VSLIDEDOWN_VI}) begin
                alu_ctrl_signals.vSlide_en = 1'b1;
                alu_ctrl_signals.is_slide1_op = instr inside {VSLIDE1DOWN_VX, VSLIDE1UP_VX};
                alu_ctrl_signals.vSlide_insert = alu_ctrl_signals.is_slide1_op;
                alu_ctrl_signals.vSlide_opSel = funct6[0];
            end
        end

        if (GBL_MASK_EN && GBL_AND_OR_XOR_EN) begin
            if (instr inside {VMANDNOT_MM, VMAND_MM, VMOR_MM, VMXOR_MM, VMORNOT_MM, VMNAND_MM, VMNOR_MM, VMXNOR_MM}) begin
                alu_ctrl_signals.vMOP_en = 1'b1;
                alu_ctrl_signals.vAndOrXor_en = 1'b1;
                // Assuming funct6[2:0] directly gives the opSel for these mask logical ops
                // or map to AOR_SEL_AND, AOR_SEL_OR, AOR_SEL_XOR if appropriate.
                alu_ctrl_signals.vAndOrXor_opSel = funct6[2:0];
            end
        end

        if (GBL_MASK_EN && GBL_MASK_EXT_EN) begin
            if (instr inside {VCPOP_M, VFIRST_M}) begin
                alu_ctrl_signals.vFirst_Popc_en = 1'b1;
                alu_ctrl_signals.vFirst_Popc_opSel = (instr inside {VFIRST_M});
            end
        end

        // Store (unconditional decoding)
        if (instr inside {VSTORE}) begin
            local_vStore_en = 1'b1;
        end

        // N-ary (unconditional decoding)
        `ifdef N_ARY_OP_SUPPORT__1
            if (instr == 32'b1000) begin // FIXME, tmp instr from original
                local_n_ary_en = 1'b1;
            end
        `endif
    end // end always_comb

    assign alu_ctrl_out = alu_ctrl_signals;
    assign vStore_en_out = local_vStore_en;

    // is_alu_compatible logic as before...
    genvar i;
    generate
        for (i = 0; i < ALU_COUNT; i = i + 1) begin : compat_loop
            assign is_alu_compatible[i] =
                (VALU__WHOLE_REG_ENABLE[i]  & VALU__VEC_MOVE_ENABLE[i]   & VALU__AND_OR_XOR_ENABLE[i] & alu_ctrl_signals.vMoveWhole_en) |
                (VALU__VEC_MOVE_ENABLE[i]   & VALU__AND_OR_XOR_ENABLE[i] & (alu_ctrl_signals.vMoveSX_en | alu_ctrl_signals.vMoveXS_en | alu_ctrl_signals.vMoveVY_en)) |
                (VALU__AND_OR_XOR_ENABLE[i] & alu_ctrl_signals.vAndOrXor_en & ~(alu_ctrl_signals.vMove_en | alu_ctrl_signals.vMOP_en) ) |
                (VALU__AND_OR_XOR_ENABLE[i] & VALU__MASK_ENABLE[i]       & alu_ctrl_signals.vMOP_en) |
                (VALU__ADD_SUB_ENABLE[i]    & alu_ctrl_signals.vAdd_en & ~(alu_ctrl_signals.vMinMax_en | alu_ctrl_signals.vMCmp_en | alu_ctrl_signals.vAAdd_en | alu_ctrl_signals.vAddSubCarry_en | alu_ctrl_signals.vID_en)) |
                (VALU__ADD_SUB_ENABLE[i]    & VALU__MIN_MAX_ENABLE[i]    & alu_ctrl_signals.vMinMax_en) |
                (VALU__ADD_SUB_ENABLE[i]    & VALU__FXP_ENABLE[i]        & alu_ctrl_signals.vAAdd_en) |
                (VALU__ADD_SUB_ENABLE[i]    & VALU__MASK_ENABLE_EXT[i]   & alu_ctrl_signals.vAddSubCarry_en) |
                (VALU__ADD_SUB_ENABLE[i]    & VALU__MASK_ENABLE[i]       & alu_ctrl_signals.vMCmp_en) |
                (VALU__ADD_SUB_ENABLE[i]    & alu_ctrl_signals.vID_en) |
                (VALU__REDUCTION_ENABLE[i]  & (alu_ctrl_signals.vRedAndOrXor_en | alu_ctrl_signals.vRedSum_min_max_en)) |
                (VALU__MULT_ENABLE[i]       & alu_ctrl_signals.vMul_en & ~(alu_ctrl_signals.vShiftLeft | alu_ctrl_signals.vShiftRight | alu_ctrl_signals.vSShift_en | alu_ctrl_signals.vSMul_en)) |
                (VALU__MULT_ENABLE[i]       & VALU__FXP_ENABLE[i]        & alu_ctrl_signals.vSMul_en) |
                (VALU__SHIFT_ENABLE[i]      & (alu_ctrl_signals.vShiftLeft | alu_ctrl_signals.vShiftRight)) |
                (VALU__SHIFT_ENABLE[i]      & VALU__FXP_ENABLE[i]        & alu_ctrl_signals.vSShift_en) |
                (VALU__SLIDE_ENABLE[i]      & alu_ctrl_signals.vSlide_en) |
                (VALU__MASK_ENABLE[i]       & VALU__MASK_ENABLE_EXT[i]   & alu_ctrl_signals.vFirst_Popc_en) |
                (VALU__MASK_ENABLE_EXT[i]   & alu_ctrl_signals.vMerge_en) |
                (VALU__NARROW_ENABLE[i]     & alu_ctrl_signals.vNarrow_en) |
                local_vStore_en |
                local_n_ary_en;
        end
    endgenerate
endmodule