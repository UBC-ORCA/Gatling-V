(* keep_hierarchy = "yes" *)module wrCtrl #(
    parameter ADDR_WIDTH = 32, // Address width
    parameter DATA_WIDTH = 64, // Data width
    parameter BANK_COUNT = 4
) (
    input                                                      clk        ,
    input                                                      rst        ,
    input  [BANK_COUNT-1:0][                   ADDR_WIDTH-1:0] wrAddr     ,
    input  [BANK_COUNT-1:0][                   DATA_WIDTH-1:0] wrData     ,
    input  [BANK_COUNT-1:0]                                    wrEn       ,
    input  [BANK_COUNT-1:0][                 DATA_WIDTH/8-1:0] wrBE       ,
    output [BANK_COUNT-1:0][ADDR_WIDTH-$clog2(BANK_COUNT)-1:0] bank_wrAddr,
    output [BANK_COUNT-1:0][                   DATA_WIDTH-1:0] bank_wrData,
    output [BANK_COUNT-1:0][                 DATA_WIDTH/8-1:0] bank_wrBE  ,
    output [BANK_COUNT-1:0]                                    bank_wrEn
);

    localparam BANK_ADDR_WIDTH = ADDR_WIDTH - $clog2(BANK_COUNT)                 ;
    localparam BYTE_EN_WIDTH   = DATA_WIDTH / 8                                  ;
    localparam TOTAL_WIDTH     = BANK_ADDR_WIDTH + DATA_WIDTH + BYTE_EN_WIDTH + 1;
    localparam LD_PORT_ID      = BANK_COUNT / 2                                  ; //assuming ALU_COUNT=BANK_COUNT/2

    wire [   ADDR_WIDTH-1:0]                  wrAddr_fifo;
    wire [   DATA_WIDTH-1:0]                  wrData_fifo;
    wire [BYTE_EN_WIDTH-1:0]                  wrBE_fifo  ;
    wire                                      wrEn_fifo  ;
    reg  [   BANK_COUNT-1:0][ BANK_COUNT-1:0] sel        ;
    wire [   BANK_COUNT-1:0][TOTAL_WIDTH-1:0] in_wr      ;

    wire [BANK_COUNT-1:0]                    concurrent_access;
    reg  [BANK_COUNT-1:0][   ADDR_WIDTH-1:0] r_wrAddr         ;
    reg  [BANK_COUNT-1:0][   DATA_WIDTH-1:0] r_wrData         ;
    reg  [BANK_COUNT-1:0]                    r_wrEn           ;
    reg  [BANK_COUNT-1:0][BYTE_EN_WIDTH-1:0] r_wrBE           ;

    genvar i,j;
    for (j = 0; j < BANK_COUNT; j = j + 1) begin
        always @(posedge clk) begin
            if (rst) begin
                r_wrAddr[j] <= 'b0;
                r_wrData[j] <= 'b0;
                r_wrEn[j]   <= 'b0;
                r_wrBE[j]   <= 'b0;
            end
            else begin
                r_wrAddr[j] <= wrAddr[j];
                r_wrData[j] <= wrData[j];
                r_wrEn[j]   <= wrEn[j];
                r_wrBE[j]   <= wrBE[j];
            end
        end
    end

    for (i=0 ; i<BANK_COUNT ; i=i+1) begin
        if(i>=LD_PORT_ID) begin
            assign concurrent_access[i] = 1'b0;
        end
        else begin
            assign concurrent_access[i] = (wrAddr[i][$clog2(BANK_COUNT)-1:0] == wrAddr[LD_PORT_ID][$clog2(BANK_COUNT)-1:0]) && wrEn[i];
        end
    end

    holdBuf #(.DATA_WIDTH(ADDR_WIDTH+DATA_WIDTH+BYTE_EN_WIDTH)) holdBuffer (
        .clk      (clk                                                     ),
        .rst      (rst                                                     ),
        .save     (|concurrent_access                                      ),
        .data     ({wrAddr[LD_PORT_ID],wrBE[LD_PORT_ID],wrData[LD_PORT_ID]}),
        .valid    (wrEn[LD_PORT_ID]                                        ),
        .data_out ({wrAddr_fifo,wrBE_fifo,wrData_fifo}                     ),
        .valid_out(wrEn_fifo                                               )
    );

    for (i=0 ; i<BANK_COUNT ; i=i+1) begin
        for (j=0 ; j<BANK_COUNT ; j=j+1) begin
            always @(posedge clk) begin
                if(rst) sel[i][j] <= 'b0;
                else    sel[i][j] <= (wrAddr[j][$clog2(BANK_COUNT)-1:0] == i[$clog2(BANK_COUNT)-1:0]) && wrEn[j];
            end
        end
    end

    for (j=0 ; j<BANK_COUNT ; j=j+1) begin
        if (j==LD_PORT_ID) begin
            assign in_wr[j] = {wrEn_fifo, wrBE_fifo, wrData_fifo, wrAddr_fifo[ADDR_WIDTH-1: $clog2(BANK_COUNT)]};
        end
        else begin
            assign in_wr[j] = {r_wrEn[j], r_wrBE[j], r_wrData[j], r_wrAddr[j][ADDR_WIDTH-1: $clog2(BANK_COUNT)]};
        end
    end


    generate
        for (i=0 ; i<BANK_COUNT ; i=i+1) begin : mux_array
            oneHotMux #(
                .WIDTH(TOTAL_WIDTH),
                .N(BANK_COUNT)
            ) oneHotMux (
                .clk  (clk),
                .rst  (rst),
                .sel  (sel[i]),
                .data (in_wr),
                .out  ({bank_wrEn[i], bank_wrBE[i], bank_wrData[i], bank_wrAddr[i]})
            );
        end
    endgenerate


endmodule
