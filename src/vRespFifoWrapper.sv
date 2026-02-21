module vRespFifoWrapper     
    import opcodes::*;
    import rvvLitePkg::*;
    import cva5_config::*;
    import riscv_types::*;
    import cva5_types::*;
    import cxu_types::*;
    import cx_dma_types::*;
(
  input  logic                  clk            ,
  input  logic                  rst            ,
  input  valu_resp_t            alu_resp      ,
  input  dstream_t              store_dstream,
  input  dstream_t              load_dstream,
  input  logic                  is_vcfg        ,
  input  logic                  is_valu        ,
  input  logic                  is_vload       ,
  input  logic                  is_vstore      ,
  input  logic                  instr_valid    ,
  input  logic                  stall          ,
  input  logic [      VL_BITS-1:0] vl             ,
  output logic                  resp_valid     ,
  output logic [DATA_WIDTH-1:0] resp_data
);

  localparam NUM_RESP_PORTS = 4;
  localparam ALU   = 0;
  localparam CFG   = 1;
  localparam LOAD  = 2;
  localparam STORE = 3;

  logic load_last_reg, en_ld_reg;
  logic is_vcfg_reg, is_valu_reg, is_vload_reg, is_vstore_reg, instr_valid_reg;
  logic                  data_buffer_valids   [NUM_RESP_PORTS];
  logic [DATA_WIDTH-1:0] data_buffer_data_outs[NUM_RESP_PORTS];

  fifo_interface #(.DATA_WIDTH(DATA_WIDTH)) data_buffers [NUM_RESP_PORTS] ();

  genvar r;

  for (r = 0; r < NUM_RESP_PORTS; ++r) begin
    cva5_fifo #(
      .DATA_WIDTH(DATA_WIDTH),
      .FIFO_DEPTH(8)
    ) data_buffer_block (
      .clk (clk),
      .rst (rst),
      .fifo (data_buffers[r])
    );
  end

  genvar i;
  for (i = 0; i < NUM_RESP_PORTS; i=i+1) begin
    always_comb begin
      data_buffer_valids[i]    = data_buffers[i].valid;
      data_buffer_data_outs[i] = data_buffers[i].data_out;
    end
  end

  always_comb begin
    data_buffers[ALU].potential_pop  = resp_port_buffer.pop & resp_port_buffer.data_out == ALU;
    data_buffers[ALU].potential_push = alu_resp.valid & alu_resp.end_flag;
    data_buffers[ALU].pop  = data_buffers[ALU].potential_pop;
    data_buffers[ALU].push = data_buffers[ALU].potential_push;
    data_buffers[ALU].data_in=(alu_resp.valid&alu_resp.scalar)?alu_resp.data[DATA_WIDTH-1:0] : 'h0;

    data_buffers[CFG].potential_pop  = resp_port_buffer.pop & resp_port_buffer.data_out == CFG;
    data_buffers[CFG].potential_push = is_vcfg_reg;
    data_buffers[CFG].pop  = data_buffers[CFG].potential_pop;
    data_buffers[CFG].push = data_buffers[CFG].potential_push;
    data_buffers[CFG].data_in = (XLEN)'(vl);

    data_buffers[LOAD].potential_pop  = resp_port_buffer.pop & resp_port_buffer.data_out == LOAD;
    data_buffers[LOAD].potential_push = en_ld_reg & load_last_reg;
    data_buffers[LOAD].pop  = data_buffers[LOAD].potential_pop;
    data_buffers[LOAD].push = data_buffers[LOAD].potential_push;
    data_buffers[LOAD].data_in = '0;

    data_buffers[STORE].potential_pop  = resp_port_buffer.pop & resp_port_buffer.data_out == STORE;
    data_buffers[STORE].potential_push = (store_dstream.valid & store_dstream.end_flag);
    data_buffers[STORE].pop  = data_buffers[STORE].potential_pop;
    data_buffers[STORE].push = data_buffers[STORE].potential_push;
    data_buffers[STORE].data_in = '0;
  end

  fifo_interface #(.DATA_WIDTH($clog2(NUM_RESP_PORTS))) resp_port_buffer ();

  cva5_fifo #(
    .DATA_WIDTH($clog2(NUM_RESP_PORTS)),
    .FIFO_DEPTH(32                    )
  ) resp_port_buffer_block (.clk(clk), .rst(rst), .fifo(resp_port_buffer));

  always_comb begin
    resp_port_buffer.potential_pop  = resp_port_buffer.valid & data_buffer_valids[resp_port_buffer.data_out];
    resp_port_buffer.potential_push = 1'b0;
    resp_port_buffer.data_in        = ALU;

    if (instr_valid_reg & ~stall) begin
      resp_port_buffer.potential_push = 1'b1;
      if (is_valu_reg)
        resp_port_buffer.data_in = ALU;
      else if (is_vcfg_reg)
        resp_port_buffer.data_in = CFG;
      else if (is_vstore_reg)
        resp_port_buffer.data_in = STORE;
      else if (is_vload_reg)
        resp_port_buffer.data_in = LOAD;
    end

    resp_port_buffer.pop  = resp_port_buffer.potential_pop;
    resp_port_buffer.push = resp_port_buffer.potential_push;
  end

  always_comb begin
    resp_valid = resp_port_buffer.pop;
    resp_data  = data_buffer_data_outs[resp_port_buffer.data_out];
  end
      
      
  always @(posedge clk) begin
    if (rst) begin
        load_last_reg <= 'b0;
        en_ld_reg <= 'b0;
        is_vcfg_reg <= 'b0;
        is_valu_reg <= 'b0;
        is_vload_reg <= 'b0;
        is_vstore_reg <= 'b0;
        instr_valid_reg <= 'b0;
    end
    else begin
        load_last_reg <= load_dstream.end_flag;
        en_ld_reg <= load_dstream.valid;
        is_vcfg_reg <= is_vcfg;
        is_valu_reg <= is_valu;
        is_vload_reg <= is_vload;
        is_vstore_reg <= is_vstore;
        instr_valid_reg <= instr_valid;
    end
  end
    
endmodule
