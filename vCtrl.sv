module vCtrl
    import opcodes::*;
    import rvvLitePkg::*;
    import cva5_config::*;
    import riscv_types::*;
    import cva5_types::*;
    import cxu_types::*;
    import cx_dma_types::*;
(
    input  wire                                                                         clk           ,
    input  wire                                                                         rst           ,
    output wire                                                                         stall         ,
    output      valu_req_t  [     SLOT_COUNT-1:0]                                       alu_req       ,
    input       valu_resp_t [     SLOT_COUNT-1:0]                                       alu_resp      ,
    input  wire                                                                         req_valid     ,
    input  wire             [    INSTR_WIDTH-1:0]                                       req_instr     ,
    input  wire             [ TRACK_ID_WIDTH-1:0]                                       req_track_id  ,
    input  wire             [     DATA_WIDTH-1:0]                                       req_data0     ,
    input  wire             [     DATA_WIDTH-1:0]                                       req_data1     ,
    output      reg                                                                     req_ready     ,
    input                   [              3-1:0]                                       req_vxrm      ,
    input       read_attr_t                                                             req_read_attr ,
    output wire             [     DATA_WIDTH-1:0]                                       resp_data     ,
    output wire                                                                         resp_valid    ,
    gen_interface.master                                                                m_read_req    ,
    gen_interface.master                                                                m_write_req   ,
    stream_interface.slave                                                              s_read_stream ,
    stream_interface.master                                                             m_write_stream,
    input  wire             [  WR_PORT_COUNT-1:0][ADDR_WIDTH-1:0]                       wrAddr        ,
    input  wire             [  WR_PORT_COUNT-1:0]                                       wrStart       ,
    output wire             [BANK_ADDR_WIDTH-1:0]                                       rdAddr        ,
    input  wire             [     BANK_COUNT-1:0][DATA_WIDTH-1:0]                       store_data    ,
    output      reg         [     SLOT_COUNT-1:0][           1:0][$clog2(SLOT_COUNT):0] rdAlignCtrl   ,
    output wire             [     SLOT_COUNT-1:0][DATA_WIDTH-1:0]                       rdOverrideData,
    output wire             [     SLOT_COUNT-1:0]                                       rdOverrideEn  ,
    output      dstream_t   [  LD_SLOT_COUNT-1:0]                                       load_dstream
);
    reg [SEW_WIDTH-1:0] sew;

    always_ff @(posedge clk) begin
        if (rst) sew <= 'b0;
        else     sew <= vtype.vsew;
    end

    genvar                            i,j;
    logic [4:0][$clog2(SLOT_COUNT):0] counter_delay;

    for (j=0 ; j<SLOT_COUNT ; j=j+1) begin
        always @(posedge clk) begin
            if  (rst) begin
                rdAlignCtrl[j][0]  <= 'b0;
                rdAlignCtrl[j][1]  <= 'b0;
            end
            else begin
                rdAlignCtrl[j][0]  <= stall ? rdAlignCtrl[j][0] : counter_delay[2][$clog2(SLOT_COUNT):0];
                rdAlignCtrl[j][1]  <= stall ? rdAlignCtrl[j][1] : counter_delay[2][$clog2(SLOT_COUNT):0];
            end
        end
    end

    signalDelayer #(.WIDTH($clog2(SLOT_COUNT)+1), .DELAY(4),.ENABLE_STALLING(ENABLE_STALLING)) d1 (.clk(clk), .rst(rst), .stall(stall), .in(counter), .out(counter_delay));

    logic                                                   req                 ;
    logic [                       8:0]                      ack                 ;
    decoded_vinstruction_t                         decoded_instr       ;
    valu_ctrl_t                                    alu_ctrl            ;
    logic [                     8-1:0]                      unaligned_len       ;
    logic [                  XLEN-1:0]                      unaligned_addr      ;
    logic [$clog2(REGISTER_COUNT)-1:0]                      vs3, vs4;
    logic [            SLOT_COUNT-1:0]                      slot_init           ;
    logic [            SLOT_COUNT-1:0]                      slot_blocked        ;
    logic [            SLOT_COUNT-1:0]                      slot_active         ;
    logic [            SLOT_COUNT-1:0]                      slot_internal_active;
    logic [         LD_SLOT_COUNT-1:0]                      ld_slot_init        ;
    logic [      $clog2(SLOT_COUNT):0]                      counter             ;
    logic [          2*SLOT_COUNT-1:0][BANK_ADDR_WIDTH-1:0] slot_operand        ;
    vtype_t                                       vtype               ;
    logic [              VL_WIDTH-1:0]                      vl                  ;
    logic [          VSTART_WIDTH-1:0]                      vstart              ;
    logic [            SLOT_COUNT-1:0]                      slot_vStore_valid   ;
    logic [         LD_SLOT_COUNT-1:0]                      ld_slot_valid       ;
    logic                                                   vcfg_ack            ;

    wire vcfg_req;
    assign stall = 'b0;

    wire                                          vcfg_req_pulse        ;
    wire      [LD_SLOT_COUNT-1:0][ADDR_WIDTH-1:0] ld_addr               ;
    dstream_t                                selected_store_dstream;
    dstream_t [LD_SLOT_COUNT-1:0]                 load_dstream_raw      ;

    assign selected_store_dstream.valid      = |slot_vStore_valid;
    assign selected_store_dstream.start_flag = slot_vStore_valid[1] ? alu_req[1].start_flag : alu_req[0].start_flag;
    assign selected_store_dstream.end_flag   = slot_vStore_valid[1] ? alu_req[1].end_flag   : alu_req[0].end_flag;
    assign selected_store_dstream.data       = slot_vStore_valid[1] ? store_data[2]         : store_data[0];
    assign selected_store_dstream.be         = slot_vStore_valid[1] ? alu_req[1].be         : alu_req[0].be;
    assign selected_store_dstream.addr       = '0;

    for (genvar l = 0; l < LD_SLOT_COUNT; l++) begin : g_load_dstream
        always_comb begin
            load_dstream[l]      = load_dstream_raw[l];
            load_dstream[l].addr = ld_addr[l];
        end
    end

    valu_resp_t selected_alu_resp;

    always_comb begin
        selected_alu_resp = '0;
        for (int k = SLOT_COUNT-1; k >= 0; k--) begin
            if (alu_resp[k].valid & alu_resp[k].end_flag)
                selected_alu_resp = alu_resp[k];
        end
    end


    vInstrDecoder vInstrDecoder (
        .clk             (clk           ),
        .rst             (rst           ),
        .valid           (req_valid     ),
        .ack             (ack           ),
        .vcfg_ack        (vcfg_ack      ),
        .ready           (req_ready     ),
        .instr           (req_instr     ),
        .track_id        (req_track_id  ),
        .rs1             (req_data0     ),
        .rs2             (req_data1     ),
        .vs3             (vs3           ),
        .vs4             (vs4           ),
        .vxrm            (req_vxrm      ),
        .vl              (vl            ),
        .vstart          (vstart        ),
        .sew             (sew           ),
        .read_attr       (req_read_attr ),
        .req             (req           ),
        .vcfg_req        (vcfg_req      ),
        .decoded_instr   (decoded_instr ),
        .decoded_alu_ctrl(alu_ctrl      ),
        .unaligned_len   (unaligned_len ),
        .unaligned_addr  (unaligned_addr),
        .wrAddr          (wrAddr        ),
        .wrStart         (wrStart       )
    );


    vOperationDispatcher vOperationDispatcher (
        .clk                 (clk                 ),
        .rst                 (rst                 ),
        .req                 (req                 ),
        .ack                 (ack                 ),
        .stall               (stall               ),
        .decoded_instr       (decoded_instr       ),
        .slot_active         (slot_active         ),
        .slot_internal_active(slot_internal_active),
        .load_dstream        (load_dstream        ),
        .slot_init           (slot_init           ),
        .ld_slot_init        (ld_slot_init        ),
        .slot_blocked        (slot_blocked        ),
        .counter             (counter             ),
        .ld_slot_valid       (ld_slot_valid       ),
        .vcfg_done           (vcfg_ack            )
    );


    generate
        for (j=0 ; j<LD_SLOT_COUNT ; j=j+1) begin : loadSlot_array
            vLoadSlot vLoadSlot (
                .clk          (clk),
                .rst          (rst),
                .init         (ld_slot_init[j]),
                .decoded_instr(decoded_instr),
                .updateAddr   (load_dstream[j].valid),
                .ldAddr       (ld_addr[j])
            );
        end
    endgenerate


    generate
        for (i=0 ; i<SLOT_COUNT ; i=i+1) begin : aluInstructionSlot_array
            vAluInstructionSlot vAluInstructionSlot (
                .clk                    (clk),
                .rst                    (rst),
                .init                   (slot_init[i]),
                .decoded_instr          (decoded_instr),
                .alu_ctrl               (alu_ctrl),
                .block                  (slot_blocked[i]),
                .stall                  (stall),
                .active                 (slot_active[i]),
                .internal_active (slot_internal_active[i]),
                .alu_scalar_req_data0   (rdOverrideData[i]),
                .is_alu_req_data0_scalar(rdOverrideEn[i]),
                .vs1                    (slot_operand[i*2]),
                .vs2                    (slot_operand[i*2+1]),
                .decoded_alu_instr      (alu_req[i]),
                .vStore_valid           (slot_vStore_valid[i])
            );
        end
    endgenerate


    muxNto1 #(.DATA_WIDTH(BANK_ADDR_WIDTH), .N(BANK_COUNT), .ENABLE_STALLING(ENABLE_STALLING)) rdAddrMux (
        .clk(clk                          ),
        .rst(rst                          ),
        .stall(stall),
        .sel(counter[$clog2(SLOT_COUNT):0]),
        .in (slot_operand                 ),
        .out(rdAddr                       )
    );


    vCfg vCfg (
        .clk      (clk                                    ),
        .rst      (rst                                    ),
        .valid    (vcfg_req                               ),
        .insn     (decoded_instr.instr                    ),
        .rf       ('{decoded_instr.rs1, decoded_instr.rs2}),
        .vl       (vl                                     ),
        .vstart   (vstart                                 ),
        .vtype    (vtype                                  ),
        .vs3      (vs3                                    ),
        .vs4      (vs4                                    ),
        .ack      (vcfg_ack                               ),
        .req_pulse(vcfg_req_pulse                         )
    );


    vRespFifoWrapper vRespFifoWrapper (
        .clk          (clk                                        ),
        .rst          (rst                                        ),
        .alu_resp     (selected_alu_resp                          ),
        .store_dstream(selected_store_dstream                     ),
        .load_dstream (load_dstream[0]                            ),
        .is_vcfg      (vcfg_ack                                   ),
        .is_valu      ((|slot_init) & decoded_instr.is_vALU       ),
        .is_vload     (|ld_slot_init                              ),
        .is_vstore    ((|slot_init) & decoded_instr.is_vStore     ),
        .instr_valid  ((|slot_init) | (|ld_slot_init) | (vcfg_ack)),
        .stall        (stall                                      ),
        .vl           (vl                                         ),
        .resp_valid   (resp_valid                                 ),
        .resp_data    (resp_data                                  )
    );


    vStoreAxiWrapper vStoreAxiWrapper (
        .clk           (clk                                   ),
        .rst           (rst                                   ),
        .stall         (stall                                 ),
        .unaligned_len (unaligned_len                         ),
        .unaligned_addr(unaligned_addr                        ),
        .track_id      (decoded_instr.track_id                ),
        .is_vstore     ((|slot_init) & decoded_instr.is_vStore),
        .store_dstream (selected_store_dstream                ),
        .m_write_req   (m_write_req                           ),
        .m_write_stream(m_write_stream                        )
    );


    vLoadAxiWrapper vLoadAxiWrapper (
        .clk           (clk                       ),
        .rst           (rst                       ),
        .stall         (stall                     ),
        .unaligned_len (unaligned_len             ),
        .unaligned_addr(unaligned_addr            ),
        .track_id      (decoded_instr.track_id    ),
        .en_ld         (ld_slot_valid[0]          ),
        .is_vload      (ld_slot_init[0]           ),
        .is_readIssue  (decoded_instr.is_readIssue),
        .load_dstream  (load_dstream_raw          ),
        .m_read_req    (m_read_req                ),
        .s_read_stream (s_read_stream             )
    );

endmodule