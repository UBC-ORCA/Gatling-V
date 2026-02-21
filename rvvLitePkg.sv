package rvvLitePkg;

import opcodes::*;
import cva5_config::*;
import riscv_types::*;
import cva5_types::*;
import cxu_types::*;
import cx_dma_types::*;
    
  localparam BANK_COUNT        = 8;
  localparam REGISTER_SIZE     = 1024*8; //Replaced by VLEN
  localparam VLEN = REGISTER_SIZE;
  localparam DATA_WIDTH        = 64;
  localparam REGISTER_COUNT    = 32;
  localparam SCA_REG_DATA_WIDTH = 53;
  //localparam XLEN = 32;
  localparam ADDR_WIDTH        = $clog2(REGISTER_COUNT * (VLEN / DATA_WIDTH));
  localparam INSTR_WIDTH       = 32;
  localparam VL_WIDTH          = $clog2(VLEN) + 1;
  localparam VSTART_WIDTH      = VL_WIDTH -1 ;
  localparam SEW_WIDTH         = 2;
  localparam DW_B              = DATA_WIDTH / 8;
  localparam BYTE_EN_WIDTH = DW_B;
  localparam VLMAX               = VLEN/8;
  localparam VL_BITS             = $clog2(VLMAX)+1;                                                                                    
  localparam DW_B_BITS         = $clog2(DATA_WIDTH/8);
  localparam BANK_ADDR_WIDTH   = ADDR_WIDTH - $clog2(BANK_COUNT);
  //localparam MEM_ADDR_WIDTH    = 32;
  //localparam TRACK_ID_WIDTH    = 4;
  localparam WR_PORT_COUNT     = BANK_COUNT/2+1;
  localparam VALU__AND_OR_XOR_ENABLE = {1'b0, 1'b1, 1'b0, 1'b0};
  localparam VALU__ADD_SUB_ENABLE    = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__MIN_MAX_ENABLE    = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__VEC_MOVE_ENABLE   = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__WHOLE_REG_ENABLE  = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__WIDEN_ADD_ENABLE  = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__REDUCTION_ENABLE  = {1'b1, 1'b1, 1'b1, 1'b0}; 
  localparam VALU__MULT_ENABLE       = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__SHIFT_ENABLE      = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__MULH_SR_ENABLE    = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__MULH_SR_32_ENABLE = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__NARROW_ENABLE     = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__WIDEN_MUL_ENABLE  = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__SLIDE_ENABLE      = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__SLIDE_N_ENABLE    = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__MULT64_ENABLE     = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__SHIFT64_ENABLE    = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__MASK_ENABLE       = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__MASK_ENABLE_EXT   = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__FXP_ENABLE        = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__ENABLE_64_BIT     = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam VALU__EN_128_MUL        = {1'b1, 1'b1, 1'b1, 1'b1}; 
  localparam SLOT_COUNT        = BANK_COUNT/2;
  localparam LD_SLOT_COUNT     = 1;
  localparam ALU_COUNT = SLOT_COUNT;
  localparam ENABLE_STALLING = 0;


  //`define N_ARY_OP_SUPPORT__1


  // For: alu_control_t.vAndOrXor_opSel [2:0]
  // Based on original vALU: vMove_en ? 3'b010 : req_func_id[2:0];
  // We'll define MOV explicitly. For AND/OR/XOR, we assume funct6[2:0] provides the code.
  // The sub-module vAndOrXor needs to interpret these.
  localparam AOR_SEL_MOV   = 3'b010; // As per original vALU logic for moves
  // If VAND_VV funct6[2:0] is, e.g., 3'b000, then that's what vAndOrXor_opSel becomes.
  // No separate constants needed if funct6 bits are passed directly for non-move logical ops.

  // For: alu_control_t.vAdd_opSel [1:0]
  // Based on original vALU: (req_func_id[2] | vMCmp_en) ? 2'b10 : req_func_id[1:0];
  // req_func_id[1:0] (funct6[1:0]) for ADD/SUB, '10' for COMPARE.
  localparam ADD_SEL_ADD     = 2'b00; // Example if funct6[1:0] for ADD is 00
  localparam ADD_SEL_SUB     = 2'b01; // Example if funct6[1:0] for SUB is 01
  localparam ADD_SEL_CMP     = 2'b10; // For compare operations
  // Other values like 2'b11 could be for other adder functions if any.

  // For: alu_control_t.vMul_opSel [1:0]
  // Based on original vALU: vShiftRight ? {req_func_id[0],1'b0} : req_func_id[1:0];
  // This implies funct6 bits determine the operation.
  // Let's define symbolic names for clarity in compatibilityChecker.
  localparam MULSHIFT_SEL_MUL_LOW  = 2'b00; // Example: For regular multiplication (result bits 63:0)
  localparam MULSHIFT_SEL_MUL_HIGH = 2'b01; // Example: For high part of multiplication (VMULH, etc.)
  localparam MULSHIFT_SEL_SLL      = 2'b10; // Example: For Shift Left Logical
  localparam MULSHIFT_SEL_SR       = 2'b11; // Example: For Shift Right (Logical/Arithmetic further distinguished by func6/funct3 bits within vMul unit)

    
  typedef struct packed {
      reg [INSTR_WIDTH-1:0] instr;
      reg [TRACK_ID_WIDTH-1:0] track_id;
      reg [VL_WIDTH-1:0] vl;
      reg [VL_WIDTH-1:0] vstart;
      reg [VL_WIDTH-2:0] begin_idx; 
      reg [VL_WIDTH-2:0] end_idx;
      reg is_widen;
      reg is_narrow;
      reg [5-1:0] imm;
      reg [BANK_ADDR_WIDTH-1:0] vs1;
      reg [BANK_ADDR_WIDTH-1:0] vs2;
      reg [BANK_ADDR_WIDTH-1:0] vs3;
      reg [BANK_ADDR_WIDTH-1:0] vs4;
      reg [ADDR_WIDTH-1:0] vd;
      reg [XLEN-1:0] rs1;
      reg [XLEN-1:0] rs2;
      reg [SEW_WIDTH-1:0] sew;
      reg [SEW_WIDTH-1:0] eff_sew;
      reg is_vStore;
      reg is_vALU;
      reg is_vCfg;
      reg is_vLoad;
      reg is_readIssue;
      reg [3-1:0] vxrm;
     reg uses_vs1;
     reg uses_vs2;
     reg uses_vd;
     reg [DATA_WIDTH-1:0] scalar_data;
     reg is_n_ary_operation;
     reg [SLOT_COUNT-1:0] is_alu_compatible;
     reg is_alu_req_data0_scalar;
      reg [DATA_WIDTH-1:0] alu_scalar_req_data0;
      reg [DATA_WIDTH/8-1:0] tail_byte_enable;
      reg [DATA_WIDTH/8-1:0] head_byte_enable;
      reg overwrite_scalar;
  } r_decoded_vinstruction_t;


typedef struct packed {
    logic [INSTR_WIDTH-1:0] instr;
    logic [TRACK_ID_WIDTH-1:0] track_id;
    logic [VL_WIDTH-1:0] vl;
    logic [VL_WIDTH-1:0] vstart;
    logic [VL_WIDTH-2:0] begin_idx; 
    logic [VL_WIDTH-2:0] end_idx;
    logic is_widen;
    logic is_narrow;
    logic [5-1:0] imm;
    logic [BANK_ADDR_WIDTH-1:0] vs1;
    logic [BANK_ADDR_WIDTH-1:0] vs2;
    logic [BANK_ADDR_WIDTH-1:0] vs3;
    logic [BANK_ADDR_WIDTH-1:0] vs4;
    logic [ADDR_WIDTH-1:0] vd;
    logic [XLEN-1:0] rs1;
    logic [XLEN-1:0] rs2;
    logic [SEW_WIDTH-1:0] sew;
    logic [SEW_WIDTH-1:0] eff_sew;
    logic is_vStore;
    logic is_vALU;
    logic is_vCfg;
    logic is_vLoad;
    logic is_readIssue;
    logic [3-1:0] vxrm;
    logic uses_vs1;
    logic uses_vs2;
    logic uses_vd;
    logic [DATA_WIDTH-1:0] scalar_data;
    logic is_n_ary_operation;
    logic [SLOT_COUNT-1:0] is_alu_compatible;
    logic is_alu_req_data0_scalar;
    logic [DATA_WIDTH-1:0] alu_scalar_req_data0;
    logic [DATA_WIDTH/8-1:0] tail_byte_enable;
    logic [DATA_WIDTH/8-1:0] head_byte_enable;
    logic overwrite_scalar;
  } decoded_vinstruction_t;


  typedef struct packed {
     logic valid;
     logic [INSTR_WIDTH-1:0] instr;
     logic [VL_WIDTH-1:0] vl;
     logic [ADDR_WIDTH-1:0] wbAddr;
     logic [DW_B-1:0] byteEnable;
     logic [SEW_WIDTH-1:0] sew;
     logic [3-1:0] vxrm;
     logic turn;
     logic [ADDR_WIDTH-1:0] vr_idx; 
     logic alu_start;
     logic alu_end;
  } decoded_valu_vinstruction_t;

 typedef struct packed {
    // Enables for major functional blocks/operation categories
     logic vMoveWhole_en;      // Vector whole register move
     logic vMoveSX_en;         // Vector move scalar to vector element
     logic vMoveXS_en;         // Vector move vector element to scalar
     logic vMoveVY_en;         // Vector move vector to vector (masked by datapath typically)
     logic vMove_en;           // Combined move enable

     logic vAndOrXor_en;       // Enable for AND/OR/XOR unit
     logic vMerge_en;          // Enable for VMERGE operations
     logic vID_en;             // Enable for VID (vector ID) operation
     logic vAdd_en;            // General enable for Add/Sub/Compare/Min/Max unit
     logic vMinMax_en;         // Specific enable for Min/Max operations within Add/Sub unit
     logic vAAdd_en;           // Enable for Averaging/Saturating Add/Sub (Fixed-Point mode for Add/Sub unit)
     logic vAddSubCarry_en;    // Enable for Add/Sub with Carry operations

     logic vRedAndOrXor_en;    // Enable for AND/OR/XOR reductions
     logic vRedSum_min_max_en; // Enable for Sum/Min/Max reductions

     logic vMul_en;            // General enable for Multiply/Shift unit (includes standard shifts)
     logic vSShift_en;         // Enable for Saturating Shifts (Fixed-Point mode for Mul/Shift unit)
     logic vSMul_en;           // Enable for Saturating Multiply (Fixed-Point mode for Mul/Shift unit)

     logic vSlide_en;          // Enable for Slide operations
     logic vMCmp_en;           // Enable for Mask Compare operations (e.g., VMSEQ, uses Add/Sub unit)
     logic vMOP_en;            // Enable for Mask Logical OPerations (e.g., VMAND, uses AndOrXor unit)
     logic vFirst_Popc_en;     // Enable for VFIRST_M / VCPOP_M

     logic vWiden_en;          // Enable for widening operations (affects add/mul inputs)
     logic vNarrow_en;         // Enable for narrowing operations (e.g., VNSRL output stage)

     logic vShiftLeft;         // Instruction is a logical/arithmetic left shift
     logic vShiftRight;        // Instruction is a logical/arithmetic right shift

    // Selectors / Operation modifiers
     logic [2:0] vAndOrXor_opSel; // Selects specific AND/OR/XOR/Move operation
     logic       vMinMax_opSel;   // Selects Min (0) or Max (1) for Min/Max unit
     logic [1:0] vAdd_opSel;      // Selects Add/Sub/Compare type for basic adder logic
     logic       vSigned_op0;     // Indicates if operand 0 is signed (for widening, add unit)
     logic       vSigned_op1;     // Indicates if operand 1 is signed (for widening, mul unit)
     logic [2:0] vRed_opSel;      // Selects specific reduction operation
     logic [1:0] vMul_opSel;      // Selects Multiply/Shift type for multiplier unit
     logic       vSlide_insert;   // True for VSLIDE1UP/DOWN (insert element vs slide existing)
     logic       vSlide_opSel;    // Selects Slide Up (0) or Down (1)
     logic [2:0] vMask_opSel;     // Selects mask generation type (e.g., for compares like VMSEQ)
     logic       vFirst_Popc_opSel; // Selects VFIRST_M (1) or VCPOP_M (0)

    // Decoded instruction fields/properties for convenience within ALU
     logic       is_masked_op_vm;   // True if instruction has vm=0 (mask is active)
     logic       is_slide1_op;    // True for VSLIDE1UP/DOWN instructions
     logic [2:0] op_mnr_funct3;   // Decoded instr[14:12] (funct3)
     logic [5:0] func6;           // Decoded instr[31:26] (funct6) or relevant upper bits
  } valu_ctrl_t;
  

 typedef struct packed {
    logic                           valid;
    logic                           start_flag;
    logic                           end_flag;
    logic                           turn;
    valu_ctrl_t                      ctrl;
    logic [SEW_WIDTH-1:0]           sew;
    logic [DATA_WIDTH-1:0]          data0;
    logic [DATA_WIDTH-1:0]          data1;
    logic [$clog2(REGISTER_COUNT)-1:0] addr;
    logic [BYTE_EN_WIDTH-1:0]       be;
    logic [BYTE_EN_WIDTH-1:0]       avl_be; // For slide
    logic [VL_BITS-1:0]             vl;     
    logic [ADDR_WIDTH-1:0]          vr_idx; // For VID, VFIRST_M index calculation
    logic [ADDR_WIDTH-$clog2(REGISTER_COUNT)-1:0] off; // For slide output offset
    logic [1:0]                     vxrm; // Vector Fixed-Point Rounding Mode
    logic [ADDR_WIDTH-1:0]          wbAddr;
    logic [INSTR_WIDTH-1:0]         instr;
 } valu_req_t;

 typedef struct packed {
    logic                                          valid;
    logic                                          start_flag;
    logic                                          end_flag;
    logic [DATA_WIDTH-1:0]                         data;
    logic [BYTE_EN_WIDTH-1:0]                      be;
    logic [$clog2(REGISTER_COUNT)-1:0]             addr;
    logic [ADDR_WIDTH-$clog2(REGISTER_COUNT)-1:0]  off;
    logic [VL_BITS-1:0]                            vl;
    logic [SEW_WIDTH-1:0]                          sew;
    logic                                          mask; // Output for mask generating operations
    logic                                          scalar; // Output for mask generating operations
    logic                                          whole_reg; // Output is a whole register move
    logic                                          narrow; // Output has been narrowed
 } valu_resp_t;

 typedef struct packed {
    logic [ADDR_WIDTH-1:0]    addr;
    logic [DATA_WIDTH-1:0]    data;
    logic                     valid;
    logic                     start_flag;
    logic                     end_flag;
    logic [BYTE_EN_WIDTH-1:0] be;
 } dstream_t;

endpackage
