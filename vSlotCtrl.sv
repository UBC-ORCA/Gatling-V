module vSlotCtrl
    import opcodes::*;
    import rvvLitePkg::*;
    import cva5_config::*;
    import riscv_types::*;
    import cva5_types::*;
    import cxu_types::*;
    import cx_dma_types::*;
(
    input  wire                           clk             ,
    input  wire                           rst             , 
    input  wire                           start_pulse     , 
    input  wire                           widen_mode      , 
    input  wire                           narrow_mode     ,
    input  wire                           reduction_mode  ,
    input  wire     [     ADDR_WIDTH-1:0] start_idx       ,
    input  wire     [     ADDR_WIDTH-1:0] end_idx         ,
    input  wire                           stall           ,
    input  wire     [BANK_ADDR_WIDTH-1:0] init_vs1        ,
    input  wire     [BANK_ADDR_WIDTH-1:0] init_vs2        ,
    input  wire     [     ADDR_WIDTH-1:0] init_vd         ,
    input  wire     [   DATA_WIDTH/8-1:0] tail_byte_enable,
    input  wire     [   DATA_WIDTH/8-1:0] head_byte_enable,
    output      reg                       valid           ,
    output      reg                       start,
    output      reg                       done            ,
    output      reg                       active          ,
    output      reg                       turn            ,
    output      wire [     ADDR_WIDTH-1:0] cur_idx         ,
    output      reg [     ADDR_WIDTH-1:0] wbAddr          ,
    output      reg [   DATA_WIDTH/8-1:0] wbBE            ,
    output      reg [BANK_ADDR_WIDTH-1:0] vs1             ,
    output      reg [BANK_ADDR_WIDTH-1:0] vs2,
    output      wire imm_done,
    output      wire early_done    
);

    localparam int ALIGN_BITS = $clog2(BANK_COUNT);

    // Aligned start of the first chunk
    wire [ADDR_WIDTH-1:0] base_start   ;
    assign base_start = {start_idx[ADDR_WIDTH-1:ALIGN_BITS], {ALIGN_BITS{1'b0}}};
    reg r_widen_mode, r_narrow_mode, r_reduction_mode;
    reg [ADDR_WIDTH-1:0] r_start_idx, r_end_idx;
    reg [   DATA_WIDTH/8-1:0] r_tail_byte_enable, r_head_byte_enable;
    reg [     ADDR_WIDTH-1:0] cur;
    assign cur_idx = cur - 1;
    assign imm_done = (!widen_mode && base_start == end_idx);
    assign early_done = (r_widen_mode) ? (turn && cur == r_end_idx) : (cur == r_end_idx);
    always_ff @(posedge clk) begin
        if (rst) begin
            r_widen_mode <= 'b0; 
            r_narrow_mode <= 'b0;
            r_reduction_mode <= 'b0;
            r_start_idx <= 'b0;
            r_end_idx <= 'b0;
            r_tail_byte_enable <= 'b0;
            r_head_byte_enable <= 'b0;
        end
        else if (start_pulse) begin
            r_widen_mode <= widen_mode; 
            r_narrow_mode <= narrow_mode;
            r_reduction_mode <= reduction_mode;
            r_start_idx <= start_idx;
            r_end_idx <= end_idx;
            r_tail_byte_enable <= tail_byte_enable;
            r_head_byte_enable <= head_byte_enable;
        end
        else begin
            r_widen_mode <= r_widen_mode; 
            r_narrow_mode <= r_narrow_mode;
            r_reduction_mode <= r_reduction_mode;
            r_start_idx <= r_start_idx;
            r_end_idx <= r_end_idx;
            r_tail_byte_enable <= r_tail_byte_enable;
            r_head_byte_enable <= r_head_byte_enable;
        end
    end
    
    always_ff @(posedge clk) begin
        if (rst) begin
            cur <= 'b0;
            active  <= 'b0;
            start   <= 'b0;
            done    <= 'b0;
            valid   <= 'b0;
            turn    <= 'b0;
            wbAddr  <= 'b0;
            wbBE    <= 'b0;
            vs1     <= 'b0;
            vs2     <= 'b0;
        end else begin
            if (start_pulse) begin
                cur <= base_start + 1;
                done    <= 1'b0;
                turn    <= 1'b0;
                wbAddr  <= init_vd;
                vs1     <= init_vs1;
                vs2     <= init_vs2;
                // Active only if non-trivial range
                if (!widen_mode && base_start == end_idx) begin
                    // Non-repeat, single element range -> done immediately
                    active <= 1'b0;
                    done   <= 1'b1;
                end else begin
                    active <= 1'b1;
                    done   <= 1'b0;
                end

                if (base_start == start_idx) begin
                    start <= 'b1;
                    if (base_start == end_idx) begin
                        wbBE <= tail_byte_enable & head_byte_enable;
                    end
                    else begin
                        wbBE <= head_byte_enable;
                    end
                end
                else begin
                    wbBE <= 'b0;
                    start <= 'b0;
                end

                // VALID immediately if inside requested range
                valid <= (base_start >= start_idx && base_start <= end_idx);

            end
            else if (active & ~stall) begin
                // VALID is masked to the actual requested range
                

                if (!r_widen_mode) begin
                    if (cur == r_start_idx) begin
                        start <= 'b1;
                        if (cur == r_end_idx) begin
                            wbBE <= r_tail_byte_enable & r_head_byte_enable;
                        end
                        else begin
                            wbBE <= r_head_byte_enable;
                        end
                    end
                    else begin
                        start <= 'b0;
                        if (cur == r_end_idx) begin
                            wbBE <= r_tail_byte_enable;
                        end
                        else begin
                            wbBE <= {BYTE_EN_WIDTH{1'b1}};
                        end
                    end
                    valid <= (cur >= r_start_idx && cur <= r_end_idx);

                    // ---------------- NORMAL/NARROW MODE ----------------
                    if (!r_narrow_mode) turn <= turn;
                    else turn <= ~turn;

                    if (cur[ALIGN_BITS-1:0] == 'b01) vs1 <= vs1 + 'b1;
                    else                             vs1 <= vs1;
                    if (cur[ALIGN_BITS-1:0] == 'b10) vs2 <= vs2 + 'b1;
                    else                             vs2 <= vs2;
                    if (!r_reduction_mode) begin
                        if (!r_narrow_mode) begin
                            wbAddr <= wbAddr + 'b1;
                        end
                        else begin
                            if (cur[0] == 'b0) wbAddr <= wbAddr + 'b1;
                            else                             wbAddr <= wbAddr;
                        end
                    end
                    else begin
                        wbAddr <= wbAddr;
                    end
                    if (cur == r_end_idx) begin
                        cur <= cur + 1;
                        //cur_idx <= cur_idx + 1;
                        done    <= 1'b1;
                        active  <= 1'b0;
                    end else begin
                        cur <= cur + 1;
                        //cur_idx <= cur_idx + 1;
                        done    <= done;
                        active  <= active;
                    end

                end
                else begin
                    // ---------------- WIDEN MODE ----------------
                    if (cur[ALIGN_BITS-1:0] == {ALIGN_BITS{1'b0}}) begin
                        if (!turn) cur <= cur - 'd3;
                        else cur<= cur + 1;
                        // End of current aligned block reached
                        vs1 <= vs1;
                        vs2 <= vs2;
                        if (!turn) begin
                            if (cur == r_start_idx) begin
                                start <= 'b1;
                                if (cur == r_end_idx+1) begin
                                    wbBE <= r_tail_byte_enable & r_head_byte_enable;
                                end
                                else begin
                                    wbBE <= r_head_byte_enable;
                                end
                            end
                            else begin
                                start <= 'b0;
                                if (cur == r_end_idx+1) begin
                                    wbBE <= r_tail_byte_enable;
                                end
                                else begin
                                    wbBE <= {BYTE_EN_WIDTH{1'b1}};
                                end
                            end
                            valid <= (cur >= r_start_idx && cur <= r_end_idx+1);
                            // First phase done -> start second phase
                            //cur_idx <= {cur_idx[ADDR_WIDTH-1:ALIGN_BITS], {ALIGN_BITS{1'b0}}}; // go back to block start
                            turn    <= 1'b1;
                            if (!r_reduction_mode) wbAddr <= wbAddr - 'd5;
                            else                 wbAddr <= wbAddr;
                        end
                        else begin
                            start <= 'b0;
                            if (cur == r_start_idx) begin
                                if (cur == r_end_idx) begin
                                    wbBE <= r_tail_byte_enable & r_head_byte_enable;
                                end
                                else begin
                                    wbBE <= r_head_byte_enable;
                                end
                            end
                            else begin
                                if (cur == r_end_idx) begin
                                    wbBE <= r_tail_byte_enable;
                                end
                                else begin
                                    wbBE <= {BYTE_EN_WIDTH{1'b1}};
                                end
                            end
                            valid <= (cur >= r_start_idx && cur <= r_end_idx);
                            // Second phase done -> move to next block
                            //cur_idx <= cur_idx + 1;
                            turn    <= 1'b0;
                            if (!r_reduction_mode) wbAddr <= wbAddr + 'b1;
                            else                wbAddr <= wbAddr;
                        end
                    end
                    else begin
                        if (cur == r_start_idx) begin
                            start <= 'b1;
                            if (cur == r_end_idx) begin
                                wbBE <= r_tail_byte_enable & r_head_byte_enable;
                            end
                            else begin
                                wbBE <= r_head_byte_enable;
                            end
                        end
                        else begin
                            start <= 'b0;
                            if (cur == r_end_idx) begin
                                wbBE <= r_tail_byte_enable;
                            end
                            else begin
                                wbBE <= {BYTE_EN_WIDTH{1'b1}};
                            end
                        end
                        valid <= (cur >= r_start_idx && cur <= r_end_idx);
                        // Inside the chunk, increment normally
                        cur <= cur + 1;
                        turn    <= turn;
                        if (!r_reduction_mode) wbAddr <= wbAddr + 'd2;
                        else                 wbAddr <= wbAddr;
                        if (cur[ALIGN_BITS-1:0] == 'b01 & turn) vs1 <= vs1 + 'b1;
                        else                             vs1 <= vs1;
                        if (cur[ALIGN_BITS-1:0] == 'b10 & turn) vs2 <= vs2 + 'b1;
                        else                             vs2 <= vs2;
                    end

                    // Check if second phase reached end_idx -> DONE
                    if (turn && cur == r_end_idx) begin
                        done   <= 1'b1;
                        active <= 1'b0;
                    end
                    else begin
                        done   <= 1'b0;
                        active <= 1'b1;
                    end
                end
            end
            else begin
                cur <= cur;
                start <= start;
                active  <= active;
                done    <= done;
                valid   <= active & (cur >= r_start_idx && cur <= r_end_idx);
                turn    <= turn;
                wbBE    <= wbBE;
                vs1     <= vs1;
                vs2     <= vs2;
                wbAddr  <= wbAddr;
            end
        end
    end

endmodule
