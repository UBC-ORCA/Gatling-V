module vOperationDispatcher
    import opcodes::*;
    import rvvLitePkg::*;
    import cva5_config::*;
    import riscv_types::*;
    import cva5_types::*;
    import cxu_types::*;
    import cx_dma_types::*;
(
    input  wire                                  clk                 ,
    input  wire                                  rst                 ,
    input  wire                                  req                 ,
    output      reg       [                 8:0] ack                 ,
    input  wire                                  stall               ,
    input       decoded_vinstruction_t           decoded_instr       ,
    input  wire           [      SLOT_COUNT-1:0] slot_active         ,
    input  wire           [      SLOT_COUNT-1:0] slot_internal_active,
    input       dstream_t [   LD_SLOT_COUNT-1:0] load_dstream        ,
    input  wire                                  vcfg_done           ,
    output      reg       [      SLOT_COUNT-1:0] slot_init           ,
    output      reg       [   LD_SLOT_COUNT-1:0] ld_slot_init        ,
    output      reg       [      SLOT_COUNT-1:0] slot_blocked        ,
    output wire           [$clog2(SLOT_COUNT):0] counter             ,
    output      reg       [   LD_SLOT_COUNT-1:0] ld_slot_valid
);

    wire                     proceed;
    wire [   SLOT_COUNT-1:0] w_slot_init   ;
    wire [   SLOT_COUNT-1:0] slot_turn     ;
    wire [LD_SLOT_COUNT-1:0] w_ld_slot_init;
    reg                      n_ary_started ;
    wire                     block         ;
    wire                     w_ack         ;
    reg  [   SLOT_COUNT-1:0] slot_is_vstore;

    genvar i;

    assign proceed = ENABLE_STALLING ? ~stall : 1'b1;

    wire [SLOT_COUNT-1:0] init_condition;

    assign w_ack = (|w_slot_init) | (vcfg_done) | (|ld_slot_init);

    assign w_slot_init[0] = req && proceed && (~block) && (~(decoded_instr.is_vStore && |(slot_is_vstore))) &&
        (~decoded_instr.is_vLoad) && init_condition[0];

    assign w_slot_init[1] = req && proceed && (~block) && (~(decoded_instr.is_vStore && |(slot_is_vstore))) &&
        (~decoded_instr.is_vLoad) && init_condition[1] && (~init_condition[0]);

    for (i=2 ; i<SLOT_COUNT ; i=i+1) begin
        assign w_slot_init[i] = req && proceed && (~block) && (~(decoded_instr.is_vStore && |(slot_is_vstore))) &&
            (~decoded_instr.is_vLoad) && init_condition[i] && ~(|init_condition[i-1:0]);
    end

    if (SLOT_COUNT == 2) begin
        for (i=0 ; i<SLOT_COUNT ; i=i+1) begin
            assign init_condition[i] = decoded_instr.is_alu_compatible[i] &&
                (~slot_active[i]) && ((~(|slot_active)) || slot_turn[i]);
        end
    end
    else if (SLOT_COUNT == 4) begin
        for (i=0 ; i<SLOT_COUNT ; i=i+1) begin
            assign init_condition[i] = decoded_instr.is_alu_compatible[i] &&
                (~slot_active[i]) && ((~(|slot_active)) || slot_turn[i]);
        end
    end

    assign w_ld_slot_init[0] = req && proceed && (~block) && decoded_instr.is_vLoad && (~ld_slot_valid[0]);

    for (i=0 ; i<9 ; i=i+1) begin
        always @(posedge clk) begin
            if (rst) ack[i] <= 'b0;
            else     ack[i] <= w_ack;
        end
    end

    always @(posedge clk) begin
        if (rst) n_ary_started <= 'b0;
        else if (w_ack) n_ary_started <= 'b0;
        else n_ary_started <= slot_init[0];
    end

    for (i=0 ; i<SLOT_COUNT ; i=i+1) begin
        always @(posedge clk) begin
            if(rst) begin
                slot_is_vstore[i] <= 'b0;
            end
            else if(slot_init[i]) begin
                slot_is_vstore[i] <= decoded_instr.is_vStore;
            end
            else if(~slot_active[i]) begin
                slot_is_vstore[i] <= 1'b0;
            end
            else begin
                slot_is_vstore[i] <= slot_is_vstore[i];
            end
        end
    end

    for (i=0 ; i<LD_SLOT_COUNT ; i=i+1) begin
        always @(posedge clk) begin
            if(rst) begin
                ld_slot_valid[i] <= 'b0;
            end
            else if(w_ld_slot_init[i]) begin 
                ld_slot_valid[i] <= 'b1;
            end
            else if(load_dstream[i].end_flag) begin
                ld_slot_valid[i] <= 'b0;
            end
            else begin
                ld_slot_valid[i] <= ld_slot_valid[i];
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            slot_init    <= 'b0;
            ld_slot_init <= 'b0;
            slot_blocked <= 'b0;
        end else begin
            slot_init                    <= w_slot_init;
            ld_slot_init                 <= w_ld_slot_init;
            slot_blocked[0]              <= decoded_instr.is_n_ary_operation;
            slot_blocked[SLOT_COUNT-1:1] <= 'b0;
        end
    end

    counter #(.BANK_COUNT(SLOT_COUNT*2)) counter_inst (
        .clk        (clk                 ),
        .rst        (rst                 ),
        .stall      (stall               ),
        .slot_init  (slot_init           ),
        .slot_active(slot_internal_active),
        .count      (counter             ),
        .slot_turn  (slot_turn           )
    );
    //assign slot_turn = 'hF;
    blocker #(
        .SLOT_COUNT    (SLOT_COUNT+LD_SLOT_COUNT),
        .TIMEOUT_CYCLES(8                       )
    ) blocker_inst (
        .clk       (clk                                                                                               ),
        .rst       (rst                                                                                               ),
        .ack       (ack[8]                                                                                            ),
        .stall     (stall                                                                                             ),
        .trig      (decoded_instr.is_vCfg | (decoded_instr.is_vALU & (decoded_instr.is_widen | decoded_instr.is_narrow))),
        .slot_valid({ld_slot_valid,slot_active}                                                                       ),
        .block     (block                                                                                             )
    );

endmodule
