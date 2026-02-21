(* keep_hierarchy = "yes" *)module gatlingV
    import opcodes::*;
    import rvvLitePkg::*;
    import cva5_config::*;
    import riscv_types::*;
    import cva5_types::*;
    import cxu_types::*;
    import cx_dma_types::*;
(
    input  wire                   clk           ,
    input  wire                   rst           ,
    input  wire                   req_valid     ,
    output wire                   req_ready     ,
    input  wire [INSTR_WIDTH-1:0] req_instr     ,
    input       read_attr_t       req_read_attr ,
    input       track_id_t        req_track_id  ,
    input  wire [          3-1:0] req_vxrm      ,
    input  wire [ DATA_WIDTH-1:0] req_data0     ,
    input  wire [ DATA_WIDTH-1:0] req_data1     ,
    output wire                   resp_valid    ,
    output wire [ DATA_WIDTH-1:0] resp_data     ,
    gen_interface.master          m_read_req    ,
    gen_interface.master          m_write_req   ,
    stream_interface.slave        s_read_stream ,
    stream_interface.master       m_write_stream
);

    valu_req_t  [SLOT_COUNT-1:0] alu_req ;
    valu_resp_t [SLOT_COUNT-1:0] alu_resp;

    valu_req_t  [   SLOT_COUNT-1:0] vCtrl_alu_req ;
    valu_resp_t [   SLOT_COUNT-1:0] vCtrl_alu_resp;
    dstream_t   [LD_SLOT_COUNT-1:0] load_dstream  ;

    wire [BANK_COUNT-1:0][ADDR_WIDTH-1:0] wrAddr ;
    wire [BANK_COUNT-1:0][DATA_WIDTH-1:0] wrData ;
    wire [BANK_COUNT-1:0]                 wrEn   ;
    wire [BANK_COUNT-1:0]                 wrStart;
    wire [BANK_COUNT-1:0][      DW_B-1:0] wrBE   ;

    wire [     SLOT_COUNT-1:0][           1:0][$clog2(SLOT_COUNT):0] rdAlignCtrl   ;
    wire [BANK_ADDR_WIDTH-1:0]                                       rdAddr        ;
    wire [     BANK_COUNT-1:0][DATA_WIDTH-1:0]                       rdData        ;
    wire [     SLOT_COUNT-1:0][DATA_WIDTH-1:0]                       rdOverrideData;
    wire [     SLOT_COUNT-1:0]                                       rdOverrideEn  ;

    wire stall;


    genvar i,j,z;
    generate
        for (i=0 ; i<SLOT_COUNT ; i=i+1) begin : vALU_array
            vALU #(.ID(i)) vALU (
                .clk           (clk                                          ),
                .rst           (rst                                          ),
                .ready     (                                             ),
                .req (alu_req[i]),
                .resp(alu_resp[i])
            );
            always_comb begin
                alu_req[i] = vCtrl_alu_req[i];
                alu_req[i].data0 = rdData[2*i];
                alu_req[i].data1 = rdData[2*i+1];
            end
            assign vCtrl_alu_resp[i] = alu_resp[i];

            assign wrAddr[i]  = {alu_resp[i].addr, alu_resp[i].off};
            assign wrData[i]  = alu_resp[i].data;
            assign wrEn[i]    = alu_resp[i].valid & ~alu_resp[i].scalar;
            assign wrBE[i]    = alu_resp[i].be;
            assign wrStart[i] = alu_resp[i].start_flag;
        end
        for (j=SLOT_COUNT ; j<SLOT_COUNT+LD_SLOT_COUNT ; j=j+1) begin
            assign wrAddr[j]  = load_dstream[j-SLOT_COUNT].addr;
            assign wrData[j]  = load_dstream[j-SLOT_COUNT].data;
            assign wrEn[j]    = load_dstream[j-SLOT_COUNT].valid;
            assign wrBE[j]    = {DW_B{1'b1}};
            assign wrStart[j] = load_dstream[j-SLOT_COUNT].start_flag;
        end

        for (z=SLOT_COUNT+LD_SLOT_COUNT ; z<BANK_COUNT ; z=z+1) begin
            assign wrAddr[z]  = 'b0;
            assign wrData[z]  = 'b0;
            assign wrEn[z]    = 'b0;
            assign wrBE[z]    = 'b0;
            assign wrStart[z] = 'b0;
        end
    endgenerate


    gatlingVRF #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .BANK_COUNT(BANK_COUNT),
        .DATA_WIDTH(DATA_WIDTH),
        .ENABLE_STALLING(ENABLE_STALLING)
    ) gatlingVRF (
        .clk   (clk   ),
        .rst   (rst   ),
        .stall(stall),
        // Write ports
        .wrAddr(wrAddr),
        .wrData(wrData),
        .wrEn  (wrEn  ),
        .wrBE  (wrBE  ),
        // Read ports
        .rdAddr(rdAddr),
        .rdData(rdData),
        .rdAlignCtrl(rdAlignCtrl),
        .rdOverrideData(rdOverrideData),
        .rdOverrideEn(rdOverrideEn)
    );

    vCtrl vCtrl (
        .clk           (clk                       ),
        .rst           (rst                       ),
        .stall         (stall                     ),
        .alu_req       (vCtrl_alu_req             ),
        .alu_resp      (vCtrl_alu_resp            ),
        .req_valid     (req_valid                 ),
        .req_instr     (req_instr                 ),
        .req_track_id  (req_track_id              ),
        .req_data0     (req_data0                 ),
        .req_data1     (req_data1                 ),
        .req_ready     (req_ready                 ),
        .req_vxrm      (req_vxrm                  ),
        .req_read_attr (req_read_attr             ),
        .resp_data     (resp_data                 ),
        .resp_valid    (resp_valid                ),
        .m_read_req    (m_read_req                ),
        .m_write_req   (m_write_req               ),
        .s_read_stream (s_read_stream             ),
        .m_write_stream(m_write_stream            ),
        .wrAddr        (wrAddr[WR_PORT_COUNT-1:0] ),
        .wrStart       (wrStart[WR_PORT_COUNT-1:0]),
        .rdAddr        (rdAddr                    ),
        .rdAlignCtrl   (rdAlignCtrl               ),
        .rdOverrideData(rdOverrideData            ),
        .rdOverrideEn  (rdOverrideEn              ),
        .store_data    (rdData                    ),
        .load_dstream  (load_dstream              )
    );


endmodule