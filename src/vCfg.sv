module vCfg 
    import opcodes::*;
    import rvvLitePkg::*;
    import cva5_config::*;
    import riscv_types::*;
    import cva5_types::*;
    import cxu_types::*;
    import cx_dma_types::*; 
  (
    input  logic clk,
    input  logic rst,
    input  logic valid,             // Indicates a VSET* instruction is present and its inputs (insn, rf) are valid
    input  vcfg_instruction_t insn, // Contains decoded instruction fields relevant to VSET* ops
    input  logic [XLEN-1:0]   rf[1:2],   // rf[1] for rs1, rf[2] for rs2 (used for VSETVL's vtype)

    output logic [VL_BITS-1:0] vl,
    output logic [VL_BITS-1:0] vstart, // Placeholder, review its intended behavior/width
    output logic [4:0]        vs3,    // Placeholder
    output logic [4:0]        vs4,    // Placeholder
    output vtype_t            vtype,   // The resulting vector type (ensure vtype_t is packed)
    output logic              ack,
    output logic req_pulse
  );

  assign vstart = 'b0;
  assign vs3 = 'b0;
  assign vs4 = 'b0;

  logic [XLEN-1:0] avl;
  logic [XLEN-1:0] r_avl;
  logic r_is_vsetivli;
  vtype_t nvtype;
  vtype_t r_nvtype;
  logic r_valid;
  reg [2:0] counter;
  reg r_req_pulse;
  always_ff @(posedge clk) begin
    if (rst) counter <= 'b0;
    else if (ack) counter <= 'b0;
    else if (valid & ~r_valid) counter <= 'b1;
    else counter <= counter;
  end
    
  assign ack = counter == 'b1;
  
  assign req_pulse = ~r_valid & valid;
  
  always_ff @(posedge clk) begin
    if (rst) begin
        r_is_vsetivli <= 'b0;
        r_avl <= 'b0;
        r_nvtype <= '0;
        //ack      <= 'b0;
    end
    else begin
        r_is_vsetivli <= valid ? (insn inside {VSETIVLI}) : r_is_vsetivli;
        r_avl <= valid ? avl : r_avl;
        r_nvtype <= valid ? nvtype : r_nvtype;
        //ack     <= counter == 'b1;
    end
  end 
    
  always_ff @(posedge clk) begin
    if (rst) r_valid <= 'b0;
    else r_valid <= valid;
  end
  always_ff @(posedge clk) begin
    if (rst) r_req_pulse <= 'b0;
    else r_req_pulse <= req_pulse;
  end
  always_comb begin
    unique casez (insn)
      VSETIVLI: nvtype = (XLEN)'(insn[29:20]);
      VSETVLI : nvtype = (XLEN)'(insn[30:20]);
      VSETVL  : nvtype = rf[2];
      default : nvtype = 0;
    endcase
    
    /*
    if (nvtype.vill == 1'b1 ||                                        // illegal vtype 
          nvtype.zeros != 23'b0 ||                                    // reserved vtype
            nvtype.vsew > 3'($clog2((ENABLE_64_BIT ? 64 : 32)/8)) ||  // VSEW > ELEN
              nvtype.vsew != nvtype.vlmul ||                          // VSEW/VLMUL != 8
                nvtype.vma == 1'b1)                                   // mask agnostic
      nvtype = '{vill: 1'b1, default: '0};
    */
  end

  always_comb begin
    unique casez (insn)
      VSETIVLI: begin 
        avl = (XLEN)'(insn[19:15]);
      end

      VSETVLI, VSETVL: begin
        if (insn.rs1_addr != 5'd0) begin
          avl = rf[1];
        end else if (insn.rd_addr != 5'd0) begin
          avl = {XLEN{1'b1}};
        end else begin
          avl = (XLEN)'(vl);
        end
      end

      default: begin
        avl = 0;
      end
    endcase
  end
    
    always_ff @(posedge clk) begin
        if (rst) begin
            vtype <= 0;
            vl <= 'b0;
        end 
        else if (r_req_pulse) begin
            vtype <= r_nvtype;
            vl <= r_avl[VL_BITS-1:0];//r_nvtype.vill == 1'b1 ? 'b0 : ((r_avl > VLMAX & (~r_is_vsetivli)) ? VLMAX[VL_BITS-1:0] : r_avl[VL_BITS-1:0]);
        end
        else begin
            vtype <= vtype;
            vl <= vl;
        end 
    end

endmodule