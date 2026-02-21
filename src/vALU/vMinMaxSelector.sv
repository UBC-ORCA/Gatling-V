(* use_dsp = "no" *) 
module vMinMaxSelector
  #(
    parameter REQ_DATA_WIDTH        = 64,
    parameter RESP_DATA_WIDTH       = 64,
    parameter SEW_WIDTH             = 2,
    parameter REQ_BE_WIDTH          = REQ_DATA_WIDTH/8,
    parameter ENABLE_64_BIT         = 1
  )
  (
    input logic clk,
    input logic rst,
    input  logic minMax_sel,
    input  logic [SEW_WIDTH-1:0] sew,
    input  logic [REQ_DATA_WIDTH-1:0] vec0,
    input  logic [REQ_DATA_WIDTH-1:0] vec1,
    input  logic [REQ_DATA_WIDTH+16:0] sub_result,
    output reg [RESP_DATA_WIDTH-1:0] minMax_result,
    output reg [REQ_BE_WIDTH-1:0] equal,
    output reg [REQ_BE_WIDTH-1:0] lt
  );

  localparam DW_B = REQ_DATA_WIDTH >= 64 ? REQ_DATA_WIDTH/8 : 4;
  localparam NUM_SEWS = 1 << SEW_WIDTH;
  logic [REQ_BE_WIDTH-1:0] sgns [NUM_SEWS];
  logic [REQ_BE_WIDTH-1:0] lts [NUM_SEWS];
  logic [REQ_BE_WIDTH-1:0] equals [NUM_SEWS];
  logic [REQ_BE_WIDTH-1:0] sgn_bits;

  logic [RESP_DATA_WIDTH-1:0] w_minMax_result;
  logic [REQ_BE_WIDTH-1:0] w_equal;
  logic [REQ_BE_WIDTH-1:0] w_lt;

  always @(posedge clk) begin
    if(rst) begin
        minMax_result <= 'b0;
        equal <= 'b0;
        lt <= 'b0;
    end
    else begin
        minMax_result <= w_minMax_result;
        equal <= w_equal;
        lt <= w_lt;
    end
  end

  genvar j;

  for (j = 0; j < NUM_SEWS-1; j++) begin
    localparam W_B = 1 << j;
    always_comb begin
      for (int k = 0; k < DW_B/W_B; k++) begin
        sgns[j][k*W_B +: W_B] = {W_B{sub_result[(k*W_B+W_B-1)*(1+8+1)+1+8 +: 1]}};
      end
    end
  end

  if (ENABLE_64_BIT) begin 
    localparam W_B = 1 << (NUM_SEWS-1);
    always_comb begin
      for (int k = 0; k < DW_B/W_B; k++) begin
        sgns[NUM_SEWS-1][k*W_B +: W_B] = {W_B{sub_result[(k*W_B+W_B-1)*(1+8+1)+1+8 +: 1]}};
      end
    end
  end else begin
    assign sgns[NUM_SEWS-1] = 'h0;
  end

  for (j = 0; j < NUM_SEWS-1; j++) begin
    localparam W_B = 1 << j;
    always_comb begin
      lts[j] = (DW_B)'(0);
      for (int k = 0; k < DW_B/W_B; k++) begin
        lts[j][k*1 +: 1] = sub_result[(k*W_B+W_B-1)*(1+8+1)+1+8 +: 1];
      end
    end
  end

  if (ENABLE_64_BIT) begin 
    localparam W_B = 1 << (NUM_SEWS-1);
    always_comb begin
      lts[NUM_SEWS-1] = (DW_B)'(0);
      for (int k = 0; k < DW_B/W_B; k++) begin
        lts[NUM_SEWS-1][k*1 +: 1] = sub_result[(k*W_B+W_B-1)*(1+8+1)+1+8 +: 1];
      end
    end
  end else begin
    assign lts[NUM_SEWS-1] = 'h0;
  end

  for (j = 0; j < NUM_SEWS-1; j++) begin
    localparam W_B = 1 << j;
    always_comb begin
      equals[j] = (DW_B)'(0);
      for (int k = 0; k < DW_B/W_B; k++) begin
        equals[j][k*1 +: 1] = 1'b1; 
        for (int w = 0; w < W_B; w++) begin
          equals[j][k*1 +: 1] &= sub_result[(k*W_B+w)*(1+8+1)+1 +: 1+8] == (1+8)'(0);
        end
      end
    end
  end

  if (ENABLE_64_BIT) begin 
    localparam W_B = 1 << (NUM_SEWS-1);
    always_comb begin
      equals[NUM_SEWS-1] = (DW_B)'(0);
      for (int k = 0; k < DW_B/W_B; k++) begin
        equals[NUM_SEWS-1][k*1 +: 1] = 1'b1; 
        for (int w = 0; w < W_B; w++) begin
          equals[NUM_SEWS-1][k*1 +: 1] &= sub_result[(k*W_B+w)*(1+8+1)+1 +: 1+8] == (1+8)'(0);
        end
      end
    end
  end else begin
    assign equals[NUM_SEWS-1] = 'h0;
  end

  always_comb begin
    if (ENABLE_64_BIT && REQ_DATA_WIDTH >= 64) begin
      w_equal    = equals[sew];
      w_lt       = lts[sew];
      sgn_bits = sgns[sew];
    end else begin
      w_equal    = sew == 2'b11 ? equals[2] : equals[sew];
      w_lt       = sew == 2'b11 ? lts[2]    : lts[sew];
      sgn_bits = sew == 2'b11 ? sgns[2]   : sgns[sew];
    end

    for (int i = 0; i < REQ_BE_WIDTH; i++)
      w_minMax_result[i*8 +: 8] = sgn_bits[i] != minMax_sel ? vec0[i*8 +: 8] : vec1[i*8 +: 8];
  end

endmodule
