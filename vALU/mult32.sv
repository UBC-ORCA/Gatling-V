module mult32 
  #(
    parameter INPUT_WIDTH = 18
  )
  (
    input  logic clk,
    input  logic rst,
    input  logic in_shift,
    input  logic signed [INPUT_WIDTH-1:0] in_a0,
    input  logic signed [INPUT_WIDTH-1:0] in_a1,
    input  logic signed [INPUT_WIDTH-1:0] in_b0,
    input  logic signed [INPUT_WIDTH-1:0] in_b1,
    output logic signed [INPUT_WIDTH-1:0] out_mult8_b0,
    output logic signed [INPUT_WIDTH-1:0] out_mult8_b1,
    output logic signed [INPUT_WIDTH-1:0] out_mult8_b2,
    output logic signed [INPUT_WIDTH-1:0] out_mult8_b3,
    output logic signed [INPUT_WIDTH*2-2:0] out_mult16_p0,
    output logic signed [INPUT_WIDTH*2-2:0] out_mult16_p1,
    output logic signed [INPUT_WIDTH*2+30:0] out_mult32
  );

  logic signed [INPUT_WIDTH-1:0] A[2];
  logic signed [INPUT_WIDTH-1:0] B[2];
  logic signed [INPUT_WIDTH-1:0] prods8_A[2][2];
  logic signed [INPUT_WIDTH-1:0] prods8_B[2][2];
  logic signed [1+16-1:0] prods8[2][2];
  logic signed [1+32-1:0] prods16[2][2];
  logic signed [1+32-1:0] r_prods16[2][2];
  logic signed [1+1+64-1:0] prod32;
  logic signed [1+1+64-1:0] r_prod32;

  logic shift;
  logic [2*INPUT_WIDTH-1:0] addend;
  
  logic [16:0] add_b0, add_b1, add_b2, add_b3;
  logic [32:0] add_p0, add_p1, r_add_p0, r_add_p1;
  logic [16:0] w_add_b0, w_add_b1, w_add_b2, w_add_b3;
  logic [32:0] w_add_p0, w_add_p1;
  logic [1+1+64-1:0] w_add_prod32;
  logic [1+1+64-1:0] add_prod32, r_add_prod32, r2_add_prod32;
    
  assign w_add_b0 = shift ? (1+16)'(addend[        1+8 +:  8]) : (1+16)'(0);
  assign w_add_b1 = shift ? (1+16)'(addend[          0 +:  8]) : (1+16)'(0);
  assign w_add_b2 = shift ? (1+16)'(addend[2*(1+8)+1+8 +:  8]) : (1+16)'(0);
  assign w_add_b3 = shift ? (1+16)'(addend[2*(1+8)+  0 +:  8]) : (1+16)'(0);
  assign w_add_p0 = shift ? (1+32)'(addend[   0 +: 16]) : (1+32)'(0);
  assign w_add_p1 = shift ? (1+32)'(addend[2+16 +: 16]) : (1+32)'(0);
  assign w_add_prod32 = shift ? (1+1+64)'({addend[0 +: 16], addend[2+16 +: 16]}) : (1+1+64)'(0);
  
  always_ff @(posedge clk) begin
    shift <= in_shift;
    addend   <= {A[1], A[0]};
    add_b0 <= w_add_b0;
    add_b1 <= w_add_b1;
    add_b2 <= w_add_b2;
    add_b3 <= w_add_b3;
    add_p0 <= w_add_p0;
    add_p1 <= w_add_p1;
    add_prod32 <= w_add_prod32;
    r_add_p0 <= add_p0;
    r_add_p1 <= add_p1;
    r_add_prod32 <= add_prod32;
    r2_add_prod32 <= r_add_prod32;
    for (int k = 0; k < 2; k++) begin
        prods8_A[0][k] <= in_a0[k*(1+8) +: (1+8)];
        prods8_B[0][k] <= in_b0[k*(1+8) +: (1+8)];
        prods8_A[1][k] <= in_a1[k*(1+8) +: (1+8)];
        prods8_B[1][k] <= in_b1[k*(1+8) +: (1+8)];
    end 
    if (rst) begin
      shift <= 'b0;
      addend <= 'b0;
      add_b0 <= 'b0;
      add_b1 <= 'b0;
      add_b2 <= 'b0;
      add_b3 <= 'b0;
      add_p0 <= 'b0;
      add_p1 <= 'b0;
      add_prod32 <= 'b0;
      r_add_p0 <= 'b0;
      r_add_p1 <= 'b0;
      r_add_prod32 <= 'b0;
      r2_add_prod32 <= 'b0;
      {prods8_A[1][1], prods8_A[1][0], prods8_A[0][1], prods8_A[0][0]} <= 'b0;
      {prods8_B[1][1], prods8_B[1][0], prods8_B[0][1], prods8_B[0][0]} <= 'b0;
    end
  end

    /*
  always_comb begin
    A[0] = in_a0;
    A[1] = in_a1;
    B[0] = in_b0;
    B[1] = in_b1;
  end
  */
  always_ff @(posedge clk) begin
        if (rst) begin
          A[0] <= 'b0;
          A[1] <= 'b0;
          B[0] <= 'b0;
          B[1] <= 'b0;
        end
        else begin
           A[0] <= in_a0;
            A[1] <= in_a1;
            B[0] <= in_b0;
            B[1] <= in_b1;
        end
  end

  always_ff @(posedge clk) begin
    for (int j = 0; j < 2; j++) begin
      for (int k = 0; k < 2; k++) begin
        prods8[j][k] <= prods8_A[j][k] * prods8_B[j][k];

        if (rst)
          prods8[j][k] <= 'b0;
      end
    end
  end

  always_ff @(posedge clk) begin
    for (int i = 0; i < 2; i++) begin
      for (int j = 0; j < 2; j++) begin
        prods16[i][j] <= A[i] * B[j];
        r_prods16[i][j] <= prods16[i][j];
        if (rst)
          prods16[i][j] <= 'b0;
          r_prods16[i][j] <= 'b0;
      end
    end
  end

  always_comb begin
    prod32 = {(1+64)'(r_prods16[0][0]) << 32} + 
             {(1+64)'(r_prods16[0][1]) << 16} + 
             {(1+64)'(r_prods16[1][0]) << 16} + 
             {(1+64)'(r_prods16[1][1]) <<  0} ;
  end

  //TODO Clean up logic
  always_ff @(posedge clk) begin
    out_mult8_b0  <= prods8[0][1]  + add_b0;
    out_mult8_b1  <= prods8[0][0]  + add_b1;
    out_mult8_b2  <= prods8[1][1]  + add_b2;
    out_mult8_b3  <= prods8[1][0]  + add_b3;

    out_mult16_p0 <= r_prods16[0][0] + r_add_p0;
    out_mult16_p1 <= r_prods16[1][1] + r_add_p1;

    out_mult32    <= r_prod32 + r2_add_prod32;
    r_prod32      <= prod32;
    if (rst) begin
      out_mult8_b0  <= 'b0;
      out_mult8_b1  <= 'b0;
      out_mult8_b2  <= 'b0;
      out_mult8_b3  <= 'b0;
      out_mult16_p0 <= 'b0;
      out_mult16_p1 <= 'b0;
      out_mult32    <= 'b0;
      r_prod32      <= 'b0;
    end
  end

endmodule
