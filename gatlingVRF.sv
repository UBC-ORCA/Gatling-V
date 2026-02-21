module gatlingVRF #(
  parameter ADDR_WIDTH      = 7 ,
  parameter BANK_COUNT      = 4 ,
  parameter DATA_WIDTH      = 32,
  parameter ENABLE_STALLING = 0
) (
  input                                                                                clk           ,
  input                                                                                rst           ,
  input                                                                                stall         ,
  //Write ports
  input  [                   BANK_COUNT-1:0][  ADDR_WIDTH-1:0]                         wrAddr        ,
  input  [                   BANK_COUNT-1:0][  DATA_WIDTH-1:0]                         wrData        ,
  input  [                   BANK_COUNT-1:0]                                           wrEn          ,
  input  [                   BANK_COUNT-1:0][DATA_WIDTH/8-1:0]                         wrBE          ,
  //Read ports
  input  [ADDR_WIDTH-$clog2(BANK_COUNT)-1:0]                                           rdAddr        ,
  output [                   BANK_COUNT-1:0][  DATA_WIDTH-1:0]                         rdData        ,
  input  [                 BANK_COUNT/2-1:0][             1:0][$clog2(BANK_COUNT/2):0] rdAlignCtrl   ,
  input  [                 BANK_COUNT/2-1:0][  DATA_WIDTH-1:0]                         rdOverrideData,
  input  [                 BANK_COUNT/2-1:0]                                           rdOverrideEn
);

  localparam BANK_ADDR_WIDTH = ADDR_WIDTH - $clog2(BANK_COUNT);

  wire [2*BANK_COUNT-1:0][     DATA_WIDTH-1:0] rdDataRaw      ;
  wire [  BANK_COUNT-1:0][BANK_ADDR_WIDTH-1:0] bank_wrAddr    ;
  wire [  BANK_COUNT-1:0][     DATA_WIDTH-1:0] bank_wrData    ;
  wire [  BANK_COUNT-1:0][   DATA_WIDTH/8-1:0] bank_wrBE      ;
  wire [  BANK_COUNT-1:0]                      bank_wrEn      ;
  wire [  BANK_COUNT-1:0][BANK_ADDR_WIDTH-1:0] bank_rdAddr    ;
  wire [  BANK_COUNT-1:0][     DATA_WIDTH-1:0] bank_rdData    ;
  reg  [  BANK_COUNT-1:0][     DATA_WIDTH-1:0] bank_rdData_reg;
  reg  [  BANK_COUNT-2:0][BANK_ADDR_WIDTH-1:0] bank_rdAddr_reg;


  genvar i, j;
  for (j=0 ; j<BANK_COUNT ; j=j+1) begin
    assign rdDataRaw[2*j]   = bank_rdData_reg[j];
    assign rdDataRaw[2*j+1] = bank_rdData[j];
  end


  always_ff @(posedge clk) begin
    if (rst) begin
      bank_rdAddr_reg[0] <= 'b0;
    end
    else begin
      bank_rdAddr_reg[0] <= (ENABLE_STALLING&stall)?bank_rdAddr_reg[0] : rdAddr;
    end
  end

  assign bank_rdAddr[0] = rdAddr;

  for (i=1 ; i<BANK_COUNT ; i=i+1) begin
    assign bank_rdAddr[i] = bank_rdAddr_reg[i-1];
  end
  for (i=1 ; i<BANK_COUNT-1 ; i=i+1) begin
    always_ff @(posedge clk) begin
      if (rst) begin
        bank_rdAddr_reg[i] <= 'b0;
      end
      else begin
        bank_rdAddr_reg[i] <= (ENABLE_STALLING & stall) ? bank_rdAddr_reg[i] : bank_rdAddr_reg[i-1];
      end
    end
  end

  wrCtrl #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .BANK_COUNT(BANK_COUNT),
    .DATA_WIDTH(DATA_WIDTH)
  ) wrCtrl (
    .clk        (clk        ),
    .rst        (rst        ),
    .wrAddr     (wrAddr     ),
    .wrData     (wrData     ),
    .wrEn       (wrEn       ),
    .wrBE       (wrBE       ),
    .bank_wrAddr(bank_wrAddr),
    .bank_wrData(bank_wrData),
    .bank_wrBE  (bank_wrBE  ),
    .bank_wrEn  (bank_wrEn  )
  );

  generate
    for (i=0 ; i<BANK_COUNT ; i=i+1) begin : bank_array
      bank #(
        .ADDR_WIDTH(BANK_ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH     ),
        .BANK_ID   (i              ),
        .BANK_COUNT(BANK_COUNT     ),
        .ENABLE_STALLING(ENABLE_STALLING)
      ) bank (
        .clk    (clk         ),
        .rst    (rst         ),
        .stall(stall),
        .wr_addr(bank_wrAddr[i]),
        .wr_data(bank_wrData[i]),
        .wr_en  (bank_wrEn[i]  ),
        .wr_be  (bank_wrBE[i]  ),
        .rd_addr(bank_rdAddr[i]),
        .rd_data(bank_rdData[i]   )
      );
    end
  endgenerate

  generate
    for (j=0 ; j<BANK_COUNT/2 ; j=j+1) begin : rdDataMux_array
      rdDataMux #(
        .DATA_WIDTH(DATA_WIDTH),
        .N(BANK_COUNT),
        .ID(j),
        .ENABLE_STALLING(ENABLE_STALLING)
      ) rdDataMux (
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .in(rdDataRaw),
        .sel(rdAlignCtrl[j]),
        .uses_scalar_data(rdOverrideEn[j]),
        .scalar_data(rdOverrideData[j]),
        .out({rdData[j*2+1],rdData[j*2]})
      );
    end
  endgenerate


  for (i=0 ; i<BANK_COUNT ; i=i+1) begin
    always @(posedge clk) begin
      if (rst) begin
        bank_rdData_reg[i] <= 'b0;
      end
      else begin
        bank_rdData_reg[i] <= (ENABLE_STALLING & stall) ? bank_rdData_reg[i] : bank_rdData[i];
      end
    end
  end

endmodule