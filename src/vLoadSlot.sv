module vLoadSlot
    import opcodes::*;
    import rvvLitePkg::*;
    import cva5_config::*;
    import riscv_types::*;
    import cva5_types::*;
    import cxu_types::*;
    import cx_dma_types::*;
(
    input  wire                       clk          ,
    input  wire                       rst          ,
    input  wire                       init         ,
    input       decoded_vinstruction_t decoded_instr,
    input  wire                       updateAddr   ,
    output      reg [ADDR_WIDTH-1:0]  ldAddr
);

    always @(posedge clk) begin
        if (rst) begin
            ldAddr <= 'b0;
        end
        else if (init) begin
            ldAddr <= decoded_instr.vd;
        end
        else begin
            ldAddr <= ldAddr + {'b0,updateAddr};
        end
    end

endmodule