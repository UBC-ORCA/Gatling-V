module vInstrDecoder
    import opcodes::*;
    import rvvLitePkg::*;
    import cva5_config::*;
    import riscv_types::*;
    import cva5_types::*;
    import cxu_types::*;
    import cx_dma_types::*;
(
    input  wire                                          clk,
    input  wire                                          rst,
    input  wire                                          valid,
    output                                               ready,
    input  wire     [               8:0]                 ack,
    input  wire                                          vcfg_ack,
    input  wire     [   INSTR_WIDTH-1:0]                 instr,
    input  wire     [TRACK_ID_WIDTH-1:0]                 track_id,
    input  wire     [    DATA_WIDTH-1:0]                 rs1,
    input  wire     [    DATA_WIDTH-1:0]                 rs2,
    input  wire     [             5-1:0]                 vs3,
    input  wire     [             5-1:0]                 vs4,
    input  wire     [             3-1:0]                 vxrm,
    input  wire     [       VL_BITS-1:0]                 vl,
    input  wire     [       VL_BITS-1:0]                 vstart,
    input  wire     [     SEW_WIDTH-1:0]                 sew,
    input       read_attr_t                              read_attr,
    input  wire     [ WR_PORT_COUNT-1:0][ADDR_WIDTH-1:0] wrAddr,
    input  wire     [ WR_PORT_COUNT-1:0]                 wrStart,
    output wire                                          req,
    output wire                                          vcfg_req,
    output      r_decoded_vinstruction_t                  decoded_instr,
    output      valu_ctrl_t                               decoded_alu_ctrl,
    output      reg [             8-1:0]                 unaligned_len,
    output      reg [          XLEN-1:0]                 unaligned_addr
);

    wire  [      SEW_WIDTH-1:0] eff_sew;
    wire                        is_widen;
    wire                        is_narrow;
    wire  [       VL_WIDTH-2:0] begin_idx;
    wire  [       VL_WIDTH-2:0] end_idx;
    logic [     DATA_WIDTH-1:0] scalar_data;
    wire  [     SLOT_COUNT-1:0] is_alu_compatible;
    valu_ctrl_t                 alu_ctrl;
    wire                        uses_vs1;
    wire                        uses_vs2;
    wire                        uses_vd;
    wire                        hazardCheck0;
    wire                        hazardCheck1;
    wire  [BANK_ADDR_WIDTH-1:0] hazardAddr0;
    wire  [BANK_ADDR_WIDTH-1:0] hazardAddr1;
    reg                         internal_req;
    wire                        noHazard;
    wire  [       VL_WIDTH-1:0] adjusted_vl;
    wire  [   VSTART_WIDTH-1:0] adjusted_vstart;
    wire  [  BYTE_EN_WIDTH-1:0] normal_tailBE;
    wire  [  BYTE_EN_WIDTH-1:0] normal_headBE;

    reg                      internal_ready;
    reg r_ready;
    reg r_noHazard;
    reg [   INSTR_WIDTH-1:0] r_instr;
    wire [   INSTR_WIDTH-1:0] selected_instr;
    reg [DATA_WIDTH-1:0] r_rs1, r_rs2;
    wire [DATA_WIDTH-1:0] selected_rs1, selected_rs2;
    reg [TRACK_ID_WIDTH-1:0] r_track_id;
    reg [3-1:0] r_vxrm;
    wire [TRACK_ID_WIDTH-1:0] selected_track_id;
    wire [3-1:0] selected_vxrm;

    assign selected_instr = ready ? instr : r_instr;
    assign selected_rs1 = ready ? rs1 : r_rs1;
    assign selected_rs2 = ready ? rs2 : r_rs2;
    assign selected_track_id = ready ? track_id : r_track_id;
    assign selected_vxrm = ready ? vxrm : r_vxrm;
        
    //assign upper_lower = sew[1] ? (sew[0] ? vl[0] : vl[1]) : vl[2];
    assign adjusted_vl     = sew[1] ? (sew[0] ? 3'b000 : {vl[0],2'b00}) : (sew[0] ? {vl[1:0],1'b0} : vl[2:0]);
    assign adjusted_vstart = sew[1] ? (sew[0] ? 3'b000 : {vstart[0],2'b00}) : (sew[0] ? {vstart[1:0],1'b0} : vstart[2:0]);
    assign normal_tailBE   = (adjusted_vl == 3'b000) ? 8'hFF : ((1 << adjusted_vl) - 1);
    assign normal_headBE   = 0 - (1 << adjusted_vstart);

    assign uses_vs1 = selected_instr inside {VALU_OPIVV, VALU_OPMVV} & ~(selected_instr inside {VID_V, VMV_XS, VCPOP_M, VFIRST_M});
    assign uses_vs2 = selected_instr inside {VALU_OPIVV, VALU_OPMVV, VALU_OPIVI, VALU_OPIVX, VALU_OPMVX} & ~(selected_instr inside {VID_V, VMV_SX, VMV_VV, VMV_VX, VMV_VI});
    assign uses_vd  = selected_instr inside {VLOAD} | (selected_instr inside {VALU_OPIVV, VALU_OPMVV, VALU_OPIVI, VALU_OPIVX, VALU_OPMVX} & ~(selected_instr inside {VMV_XS, VCPOP_M, VFIRST_M}));

    assign ready = internal_ready | ack[7];

    assign hazardAddr0  = (selected_instr inside {VSTORE}) ? selected_instr[11:7] : selected_instr[19:15];
    assign hazardAddr1  = selected_instr[24:20];
    assign hazardCheck0 = (ready ? valid : r_valid) & (uses_vs1 | (selected_instr inside {VSTORE}));
    assign hazardCheck1 = (ready ? valid : r_valid) & uses_vs2;
    assign req          = (internal_req & !ack[7]) & (noHazard  | decoded_instr.is_vCfg | decoded_instr.is_vLoad);
    assign vcfg_req     = (internal_req & !ack[7]) & decoded_instr.is_vCfg;

    always @(posedge clk) begin
        if (rst)        internal_ready <= 'b1;
        else if (valid) internal_ready <= 'b0;

        //else if (ack[7] & valid)   internal_ready <= 'b0;
        else if (ack[7])   internal_ready <= 'b1;
        //else if (valid) internal_ready <= (begin_idx == end_idx);//
        else            internal_ready <= internal_ready;
    end
    r_decoded_vinstruction_t  r_decoded_instr;
    reg r_valid;

    always @(posedge clk) begin
        if      (rst) r_decoded_instr <= 'b0;
        else r_decoded_instr <= decoded_instr;
    end

    always @(posedge clk) begin
        if      (rst)           r_valid  <= 'b0;
        else if (ready)         r_valid  <= valid;
        else                    r_valid <= r_valid;
    end
    
    always @(posedge clk) begin
        if      (rst)           r_noHazard  <= 'b0;
        else if (r_ready)       r_noHazard  <= noHazard;
        else                    r_noHazard <= r_noHazard;
    end
    
    always @(posedge clk) begin
        if (rst) internal_req <= 'b0;
        else internal_req <= ready ? valid : r_valid;
    end
    
    always @(posedge clk) begin
        if (rst) r_ready <= 'b0;
        else r_ready <= ready;
    end

    always @(posedge clk) begin
        if (rst) begin
            r_instr <= 'b0;
            r_rs1   <= 'b0;
            r_rs2   <= 'b0;
            r_track_id <= 'b0;
            r_vxrm  <= 'b0;
        end
        else begin
            r_instr <= ready ? instr : r_instr;
            r_rs1   <= ready ? rs1   : r_rs1;
            r_rs2   <= ready ? rs2   : r_rs2;
            r_track_id <= ready ? track_id : r_track_id;
            r_vxrm  <= ready ? vxrm : r_vxrm;
        end
    end
    
    always @(posedge clk) begin
        if (rst) begin
            decoded_instr <= 'b0;
            unaligned_len <= 'b0;
            unaligned_addr <= 'b0;
            decoded_alu_ctrl <= 'b0;
        end
        else begin
            decoded_instr.instr                   <=  selected_instr;
            decoded_instr.track_id                <=  selected_track_id;
            decoded_instr.vl                      <=  {'b0,vl};
            decoded_instr.vstart                  <=  {'b0,vstart};
            decoded_instr.imm                     <=  selected_instr[19:15];
            decoded_instr.rs1                     <=  selected_rs1;
            decoded_instr.rs2                     <=  selected_rs2;
            decoded_instr.begin_idx               <=  begin_idx;
            decoded_instr.end_idx                 <=  end_idx;
            decoded_instr.is_widen                <=  is_widen;
            decoded_instr.is_narrow               <=  is_narrow;
            decoded_instr.vs1                     <=  {selected_instr[19:15],{(ADDR_WIDTH-5-$clog2(BANK_COUNT)){1'b0}}} + {begin_idx[ADDR_WIDTH-1:$clog2(BANK_COUNT)]};
            decoded_instr.vs2                     <=  {selected_instr[24:20],{(ADDR_WIDTH-5-$clog2(BANK_COUNT)){1'b0}}} + {begin_idx[ADDR_WIDTH-1:$clog2(BANK_COUNT)]};
            decoded_instr.vs3                     <=  {vs3,{(ADDR_WIDTH-5-$clog2(BANK_COUNT)){1'b0}}} + {begin_idx[ADDR_WIDTH-1:$clog2(BANK_COUNT)]};
            decoded_instr.vs4                     <=  {vs4,{(ADDR_WIDTH-5-$clog2(BANK_COUNT)){1'b0}}} + {begin_idx[ADDR_WIDTH-1:$clog2(BANK_COUNT)]};
            decoded_instr.vd                      <=  {selected_instr[11:7] ,{(ADDR_WIDTH-5){1'b0}}} + (is_widen ? {begin_idx[ADDR_WIDTH-1:$clog2(BANK_COUNT)+1],{($clog2(BANK_COUNT)+1){1'b0}}} : {begin_idx[ADDR_WIDTH-1:$clog2(BANK_COUNT)],{$clog2(BANK_COUNT){1'b0}}});
            decoded_instr.sew                     <=  sew;
            decoded_instr.eff_sew                 <=  eff_sew;
            decoded_instr.is_vStore               <=  selected_instr inside {VSTORE};
            decoded_instr.is_vALU                 <=  selected_instr inside {VALU_CFG} & ~(selected_instr inside {VCFG});
            decoded_instr.is_vCfg                 <=  selected_instr inside {VCFG};
            decoded_instr.is_vLoad                <=  selected_instr inside {VLOAD};
            decoded_instr.is_readIssue            <=  'b0;//read_attr <= <=  READ_ISSUE;
            decoded_instr.vxrm                    <=  selected_vxrm;
            decoded_instr.uses_vs1                <=  uses_vs1;
            decoded_instr.uses_vs2                <=  uses_vs2;
            decoded_instr.uses_vd                 <=  uses_vd;
            decoded_instr.scalar_data             <=  scalar_data;
            decoded_instr.is_n_ary_operation      <=  'b0;//selected_instr <= <=  4'b1000; //TODO custom instruction that requires 3 source operands
            decoded_instr.is_alu_compatible       <=  is_alu_compatible;
            decoded_instr.is_alu_req_data0_scalar <=  selected_instr inside {VALU_OPIVI, VALU_OPIVX, VALU_OPMVX};
            decoded_instr.alu_scalar_req_data0    <=  (DATA_WIDTH)'(scalar_data);
            decoded_instr.tail_byte_enable        <=  normal_tailBE;
            decoded_instr.head_byte_enable        <=  normal_headBE;
            decoded_instr.overwrite_scalar        <=  ~(selected_instr inside {VSLIDEUP_VI, VSLIDEDOWN_VI, VSLIDE1UP_VX, VSLIDE1DOWN_VX, VSLIDEUP_VX, VSLIDEDOWN_VX});
            unaligned_len                         <=  (8)'(vl >> (DW_B_BITS - selected_instr[13:12]));
            unaligned_addr                        <=  selected_rs1;
            decoded_alu_ctrl                      <=  alu_ctrl;
        end
    end
    
    assign eff_sew = is_narrow ? (sew - 'b1) : sew;

    if ((|VALU__WIDEN_ADD_ENABLE) | (|VALU__WIDEN_MUL_ENABLE)) begin
        assign is_widen = selected_instr inside {VWADDU_VV, VWADDU_VX, VWADD_VV, VWADD_VX, VWSUBU_VV, VWSUBU_VX, VWSUB_VV, VWSUB_VX, VWMUL_VV, VWMUL_VX, VWMULSU_VV, VWMULSU_VX};
    end else begin
        assign is_widen = 1'b0;
    end

    if ((|VALU__NARROW_ENABLE)) begin
        assign is_narrow = selected_instr inside {VNSRL_VV, VNSRL_VX, VNSRL_VI};
    end else begin
        assign is_narrow = 1'b0;
    end

    assign end_idx =
        (sew == 2'b00) ? {'b0, ((vl - 1) / (DATA_WIDTH/8))}  :
        (sew == 2'b01) ? {'b0, ((vl - 1) / (DATA_WIDTH/16))}  :
        (sew == 2'b10) ? {'b0, ((vl - 1) / (DATA_WIDTH/32))}  :
        (sew == 2'b11) ? {'b0, ((vl - 1) / (DATA_WIDTH/64))}        :
        'b0;

    assign begin_idx =
        (sew == 2'b00) ? {'b0, (vstart / (DATA_WIDTH/8))} :
        (sew == 2'b01) ? {'b0, (vstart / (DATA_WIDTH/16))} :
        (sew == 2'b10) ? {'b0, (vstart / (DATA_WIDTH/32))} :
        (sew == 2'b11) ? {'b0, (vstart / (DATA_WIDTH/64))}       :
        'b0;

    always_comb begin
        unique casez (selected_instr)
            VALU_OPIVI : begin
                unique casez (selected_instr)
                    VSLIDEDOWN_VI, VSLIDEUP_VI, VSLL_VI, VSRA_VI, VSRL_VI, VNSRL_VI: scalar_data = (DATA_WIDTH)'(selected_instr[19:15]);
                    default : scalar_data = (DATA_WIDTH)'(signed'(selected_instr[19:15]));
                endcase
            end
            default : scalar_data = (DATA_WIDTH)'(selected_rs1);
        endcase
    end

    vCompatibilityChecker vCompatibilityChecker (
        .instr            (selected_instr            ),
        .is_alu_compatible(is_alu_compatible),
        .alu_ctrl_out     (alu_ctrl         ),
        .vStore_en_out    (                 )
    );

    genvar                         i;
    logic [WR_PORT_COUNT-1:0][4:0] wrAddr_reg;
    logic [WR_PORT_COUNT-1:0]      wrStart_reg;
    //assign noHazard = 1'b1;

    /*
    for (i=0 ; i<WR_PORT_COUNT ; i=i+1) begin
        always @(posedge clk) begin
            if (rst) begin
                wrAddr_reg[i] <= 'b0;
                wrStart_reg[i] <= 'b0;
            end else begin
                wrAddr_reg[i] <= wrAddr[i][ADDR_WIDTH-1:ADDR_WIDTH-5];
                wrStart_reg[i] <= wrStart[i];
            end
        end
    end
    */
    logic [1:0] wrStart_delay;
    logic [1:0][ADDR_WIDTH-1:0] wrAddr_delay;

    signalDelayer #(.WIDTH(1), .DELAY(1)) d0 (.clk(clk), .rst(rst), .in(wrStart[SLOT_COUNT]), .out(wrStart_delay)); //Because the load might be delayed 1 cycle
    signalDelayer #(.WIDTH(ADDR_WIDTH), .DELAY(1)) d1 (.clk(clk), .rst(rst), .in(wrAddr[SLOT_COUNT]), .out(wrAddr_delay)); //Because the load might be delayed 1 cycle

    for (i=0 ; i<WR_PORT_COUNT ; i=i+1) begin
        if(i==SLOT_COUNT) begin
            assign wrAddr_reg[i] = wrStart_delay[1][ADDR_WIDTH-1:ADDR_WIDTH-5];
            assign wrStart_reg[i] = wrStart_delay[1];
        end
        else begin
            assign wrAddr_reg[i] = wrAddr[i][ADDR_WIDTH-1:ADDR_WIDTH-5];
            assign wrStart_reg[i] = wrStart[i];
        end
    end
    
    vHazardChecker #(
        .ADDR_WIDTH   (5            ),
        .WR_PORT_COUNT(WR_PORT_COUNT)
    ) vHazardChecker (
        .clk                 (clk          ),
        .rst                 (rst          ),
        .wrStart             (wrStart_reg  ),
        .wrAddr              (wrAddr_reg   ),
        .updateExpectedWbAddr(ack[6] & r_decoded_instr.uses_vd),
        .expectedWbAddr      (r_decoded_instr.instr[11:7]),
        .checkHazard0        (hazardCheck0      ),
        .checkHazard1        (hazardCheck1      ),
        .checkWrAddr0        (hazardAddr0),
        .checkWrAddr1        (hazardAddr1),
        .noHazard            (noHazard     )
    );



endmodule
