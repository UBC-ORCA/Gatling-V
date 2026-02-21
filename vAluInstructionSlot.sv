//(* keep_hierarchy = "yes" *)
module vAluInstructionSlot
    import opcodes::*;
    import rvvLitePkg::*;
    import cva5_config::*;
    import riscv_types::*;
    import cva5_types::*;
    import cxu_types::*;
    import cx_dma_types::*;
(
    input  wire                       clk                    ,
    input  wire                       rst                    ,
    input  wire                       init                   ,
    input  decoded_vinstruction_t      decoded_instr          ,
    input  valu_ctrl_t                 alu_ctrl,
    input  wire                       block                  ,
    input wire stall,
    output wire                       active,
    output wire                       internal_active,
    output reg [     DATA_WIDTH-1:0]  alu_scalar_req_data0   ,
    output reg                        is_alu_req_data0_scalar,
    output wire [BANK_ADDR_WIDTH-1:0] vs1                    ,
    output wire [BANK_ADDR_WIDTH-1:0] vs2                    ,
    output valu_req_t  decoded_alu_instr,
    output reg vStore_valid
);
    wire imm_done, early_done;
    assign active = (internal_active & !early_done) | (init & !imm_done);
    wire [7:0] turn_delay; //TODO fix width
    wire [6:0][ADDR_WIDTH-1:0] wbAddr_delay; //TODO fix width
    wire [6:0][DW_B-1:0] byteEnable_delay; //TODO fix width
    wire [6:0][ADDR_WIDTH-1:0] vr_idx_delay; //TODO fix width
    wire [6:0] alu_start_delay; //TODO fix width
    wire [6:0] alu_end_delay; //TODO fix width
    wire [6:0] alu_valid_delay; //TODO fix width
    wire [6:0] vStore_valid_delay; //TODO fix width

    r_decoded_vinstruction_t saved_decoded_instr  ;
    valu_ctrl_t saved_alu_ctrl;
    
    valu_ctrl_t alu_ctrl_delayed [6:0];
    wire [6:0][INSTR_WIDTH-1:0] instr_delayed;
    wire [6:0][VL_WIDTH-1:0] vl_delayed;
    wire [6:0][SEW_WIDTH-1:0] sew_delayed;
    wire [6:0][3-1:0] vxrm_delayed;
    wire [6:0][DATA_WIDTH-1:0] alu_scalar_req_data0_delayed;
    wire [6:0] is_alu_req_data0_scalar_delayed;   
    wire valid, done, turn, start;
    wire [ADDR_WIDTH-1:0] vr_idx;
    wire [ADDR_WIDTH-1:0] wbAddr;
    wire [DATA_WIDTH/8-1:0] wbBE;
    reg [ADDR_WIDTH-1:0] r_vr_idx;
    
    always @(posedge clk) begin
        if (rst) begin
            decoded_alu_instr <= 'b0;
            vStore_valid <= 'b0;
            alu_scalar_req_data0 <= 'b0;
            is_alu_req_data0_scalar <= 'b0;
            r_vr_idx <= 'b0;
        end else begin
            decoded_alu_instr.valid      <= alu_valid_delay[3] & ~stall; 
            decoded_alu_instr.start_flag  <= alu_start_delay[3]; 
            decoded_alu_instr.end_flag    <= alu_end_delay[3]; 
            decoded_alu_instr.turn       <= turn_delay[3]; 
            decoded_alu_instr.ctrl       <= alu_ctrl_delayed[3];
            decoded_alu_instr.sew        <= sew_delayed[3];
            decoded_alu_instr.addr     <= wbAddr_delay[3][ADDR_WIDTH-1:ADDR_WIDTH-$clog2(REGISTER_COUNT)]; 
            decoded_alu_instr.be <= byteEnable_delay[3]; 
            decoded_alu_instr.avl_be <= byteEnable_delay[3]; 
            decoded_alu_instr.vl         <= vl_delayed[3];
            decoded_alu_instr.vr_idx     <= vr_idx_delay[2]; 
            decoded_alu_instr.off     <= wbAddr_delay[3][ADDR_WIDTH-1-$clog2(REGISTER_COUNT):0]; 
            decoded_alu_instr.vxrm       <= vxrm_delayed[3];
            decoded_alu_instr.data0       <= 'b0;
            decoded_alu_instr.data1       <= 'b0;
            decoded_alu_instr.wbAddr     <= wbAddr_delay[3]; 
            decoded_alu_instr.instr      <= instr_delayed[3];
            vStore_valid                 <= vStore_valid_delay[3] & ~stall; 
            alu_scalar_req_data0         <= alu_scalar_req_data0_delayed[2];
            is_alu_req_data0_scalar      <= is_alu_req_data0_scalar_delayed[2];
            r_vr_idx                     <= vr_idx;
        end
    end

    r_decoded_vinstruction_t next_decoded_instr;

    always_comb begin
        next_decoded_instr = decoded_instr;
        next_decoded_instr.alu_scalar_req_data0 = decoded_instr.overwrite_scalar ? w_alu_scalar_req_data0 : decoded_instr.alu_scalar_req_data0;
    end

    always @(posedge clk) begin
        if (rst) begin
            saved_decoded_instr <= 'b0;
            saved_alu_ctrl <= 'b0;
        end
        else if (init) begin
            saved_decoded_instr <= next_decoded_instr;
            saved_alu_ctrl <= alu_ctrl;
        end
        else begin
            saved_decoded_instr <= saved_decoded_instr;
            saved_alu_ctrl <= saved_alu_ctrl;
        end
    end
    
    logic [DATA_WIDTH-1:0] w_alu_scalar_req_data0;
    
    always_comb begin
        unique case (decoded_instr.eff_sew)
            2'b00 : w_alu_scalar_req_data0 = {DATA_WIDTH/8{decoded_instr.alu_scalar_req_data0[8-1:0]}};
            2'b01 : w_alu_scalar_req_data0 = {DATA_WIDTH/16{decoded_instr.alu_scalar_req_data0[16-1:0]}};
            2'b10 : w_alu_scalar_req_data0 = {DATA_WIDTH/32{decoded_instr.alu_scalar_req_data0[32-1:0]}};
            2'b11 : w_alu_scalar_req_data0 = {DATA_WIDTH/64{(64)'(signed'(decoded_instr.alu_scalar_req_data0[32-1:0]))}};
        endcase
    end

    signalDelayer #(.WIDTH(1),          .DELAY(7)) d1  (.clk(clk), .rst(rst), .in(turn), .out(turn_delay));
    signalDelayer #(.WIDTH(ADDR_WIDTH), .DELAY(6)) d2  (.clk(clk), .rst(rst), .in(wbAddr), .out(wbAddr_delay));
    signalDelayer #(.WIDTH(DW_B),       .DELAY(6)) d3  (.clk(clk), .rst(rst), .in(wbBE), .out(byteEnable_delay));
    signalDelayer #(.WIDTH(ADDR_WIDTH), .DELAY(6)) d4  (.clk(clk), .rst(rst), .in(r_vr_idx << (3-saved_decoded_instr.sew)), .out(vr_idx_delay)); //TODO fix width
    signalDelayer #(.WIDTH(1),          .DELAY(6)) d5  (.clk(clk), .rst(rst), .in(valid & start), .out(alu_start_delay));
    signalDelayer #(.WIDTH(1),          .DELAY(6)) d6  (.clk(clk), .rst(rst), .in(valid & done), .out(alu_end_delay));
    signalDelayer #(.WIDTH(1),          .DELAY(6)) d7  (.clk(clk), .rst(rst), .in(valid & (~block) & (~saved_decoded_instr.is_vStore)), .out(alu_valid_delay));
    signalDelayer #(.WIDTH(1),          .DELAY(6)) d8  (.clk(clk), .rst(rst), .in(valid & (~block) & saved_decoded_instr.is_vStore), .out(vStore_valid_delay));
    signalDelayer #(.WIDTH(INSTR_WIDTH),.DELAY(6)) d9  (.clk(clk), .rst(rst), .in(saved_decoded_instr.instr), .out(instr_delayed));
    signalDelayer #(.WIDTH(VL_WIDTH),   .DELAY(6)) d10 (.clk(clk), .rst(rst), .in(saved_decoded_instr.vl), .out(vl_delayed));
    signalDelayer #(.WIDTH(3),          .DELAY(6)) d11 (.clk(clk), .rst(rst), .in(saved_decoded_instr.vxrm), .out(vxrm_delayed));
    signalDelayer #(.WIDTH(SEW_WIDTH),  .DELAY(6)) d12 (.clk(clk), .rst(rst), .in(saved_decoded_instr.sew), .out(sew_delayed));
    signalDelayer #(.WIDTH(DATA_WIDTH), .DELAY(6)) d13 (.clk(clk), .rst(rst), .in(saved_decoded_instr.alu_scalar_req_data0), .out(alu_scalar_req_data0_delayed));
    signalDelayer #(.WIDTH(1),          .DELAY(6)) d14 (.clk(clk), .rst(rst), .in(saved_decoded_instr.is_alu_req_data0_scalar), .out(is_alu_req_data0_scalar_delayed));

    always @(posedge clk) begin
        if (rst) begin
            alu_ctrl_delayed[1] <= 'b0;
            alu_ctrl_delayed[2] <= 'b0;
            alu_ctrl_delayed[3] <= 'b0;
        end
        else begin
            alu_ctrl_delayed[1] <= saved_alu_ctrl;
            alu_ctrl_delayed[2] <= alu_ctrl_delayed[1];
            alu_ctrl_delayed[3] <= alu_ctrl_delayed[2];
        end
    end
    
    vSlotCtrl vSlotCtrl (
        .clk             (clk),
        .rst             (rst),
        .start_pulse     (init),
        .widen_mode      (decoded_instr.is_widen),
        .narrow_mode     (decoded_instr.is_narrow),
        .reduction_mode  (alu_ctrl.vRedAndOrXor_en | alu_ctrl.vRedSum_min_max_en),
        .start_idx       (decoded_instr.begin_idx),
        .end_idx         (decoded_instr.end_idx),
        .stall           (stall),
        .init_vs1        (decoded_instr.is_vStore ? decoded_instr.vd[ADDR_WIDTH-1:$clog2(BANK_COUNT)] : decoded_instr.vs1),
        .init_vs2        (decoded_instr.vs2),
        .init_vd         (decoded_instr.vd),
        .tail_byte_enable(decoded_instr.tail_byte_enable),
        .head_byte_enable(decoded_instr.head_byte_enable),
        .valid           (valid),
        .start           (start),
        .done            (done),
        .active          (internal_active),
        .turn            (turn),
        .cur_idx         (vr_idx),
        .wbAddr          (wbAddr),
        .wbBE            (wbBE),
        .vs1             (vs1),
        .vs2             (vs2),
        .imm_done(imm_done),
        .early_done(early_done)
    );

endmodule