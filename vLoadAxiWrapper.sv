module vLoadAxiWrapper
  import opcodes::*;
  import rvvLitePkg::*;
  import cva5_config::*;
  import riscv_types::*;
  import cva5_types::*;
  import cxu_types::*;
  import cx_dma_types::*;
#(
  parameter BYPASS_LD_FIFO = 1
)
(
  input  logic                      clk           ,
  input  logic                      rst           ,
  input  logic                      stall         ,
  input  logic [             8-1:0] unaligned_len ,
  input  logic [    XLEN-1:0] unaligned_addr,
  input  logic [TRACK_ID_WIDTH-1:0] track_id      ,
  input  logic                      en_ld         ,
  input  logic                      is_vload      ,
  input  logic                      is_readIssue  ,
  output  dstream_t                  load_dstream,
  gen_interface.master              m_read_req    ,
  stream_interface.slave            s_read_stream
);

  logic                      r_ready;
  logic                      r_valid;
  logic                      r_last ;
  logic [    DATA_WIDTH-1:0] r_data ;
  logic [TRACK_ID_WIDTH-1:0] r_id   ;
  logic [             2-1:0] r_resp ;
  logic                      r_first;

  logic                      ar_ready                ;
  logic                      ar_valid                ;
  logic [             3-1:0] ar_size                 ;
  logic [             8-1:0] ar_len                  ;
  logic [TRACK_ID_WIDTH-1:0] ar_id                   ;
  logic [MEM_ADDR_WIDTH-1:0] ar_addr                 ;
  logic [  3+DATA_WIDTH-1:0] ld_data_w_valid_last_bit;

  logic stream_done;

  mem_packet_t mem_packets[0:2];
  mem_id_t     mem_ids    [0:2];

  logic [MEM_ADDR_WIDTH-1:0] unaligned_ar_addr;
  logic [             8-1:0] unaligned_ar_len ;

  logic                  read_aligner_valid;
  logic                  read_aligner_start;
  logic                  read_aligner_end  ;
  logic [DATA_WIDTH-1:0] read_aligner_data ;
  logic                  read_aligner_idle ;
  logic [ DW_B_BITS-1:0] read_aligner_shamt;

  assign load_dstream.data       = ld_data_w_valid_last_bit[DATA_WIDTH-1:0];
  assign load_dstream.valid = ld_data_w_valid_last_bit[DATA_WIDTH+2];
  assign load_dstream.end_flag  = ld_data_w_valid_last_bit[DATA_WIDTH+1];
  assign load_dstream.start_flag = ld_data_w_valid_last_bit[DATA_WIDTH];
  assign load_dstream.be = {DW_B{1'b1}};
  assign load_dstream.addr = '0;

  always_comb begin
    ar_ready         = m_read_req.ready;
    m_read_req.valid = ar_valid;
    m_read_req.data  = mem_packets[READ];
    m_read_req.id    = mem_ids[READ];
  end

  always_comb begin
    mem_packets[READ] = '{base_address  : ar_addr,
      end_address : ar_addr + (DATA_WIDTH/8)*(ar_len + 1) - 1,
      size        : ar_size,
      stride      : 32'b1};
    mem_ids[READ] = ar_id;
  end

  fifo_interface #(.DATA_WIDTH(TRACK_ID_WIDTH+8+32)) load_req_buffer ();

  cva5_fifo #(
    .DATA_WIDTH(TRACK_ID_WIDTH+8+32),
    .FIFO_DEPTH(1                  )
  ) load_req_buffer_block (
    .clk (clk            ),
    .rst (rst            ),
    .fifo(load_req_buffer)
  );

  always_comb begin
    load_req_buffer.potential_pop  = ar_ready & ar_valid;
    load_req_buffer.potential_push = is_vload & ~stall;
    //FIXME PERF Can be executed earlier
    load_req_buffer.pop            = load_req_buffer.potential_pop;
    load_req_buffer.push           = load_req_buffer.potential_push;
    load_req_buffer.data_in        = {{(TRACK_ID_WIDTH)'(track_id)},
      (8)'(unaligned_len),
      (32)'(unaligned_addr)};
    {ar_id, unaligned_ar_len, unaligned_ar_addr} = load_req_buffer.data_out;
  end


  always_comb begin
    ar_valid = load_req_buffer.valid;
    ar_addr  = (unaligned_ar_addr >> $clog2(DATA_WIDTH/8)) << $clog2(DATA_WIDTH/8);
    ar_len   = unaligned_ar_addr[$clog2(DATA_WIDTH/8)-1:0] == '0 ? unaligned_ar_len :
      unaligned_ar_len + 1;
    ar_size = 3'b011;
  end


  always_comb begin
    s_read_stream.tready = r_ready;
    r_valid              = s_read_stream.tvalid;
    r_last               = s_read_stream.tlast;
    r_data               = s_read_stream.tdata;
    r_id                 = s_read_stream.tid;
    r_resp               = '0; //unused
  end

  read_burst_aligner #(.DATA_WIDTH(DATA_WIDTH)) read_burst_aligner_block (
    .clk    (clk                        ),
    .rst    (rst                        ),
    .i_valid(r_ready & r_valid          ),
    .i_start(r_ready & r_valid & r_first),
    .i_end  (r_ready & r_valid & r_last ),
    .i_data (r_data                     ),
    .i_shamt(read_aligner_shamt         ),
    .o_valid(read_aligner_valid         ),
    .o_start(read_aligner_start         ),
    .o_end  (read_aligner_end           ),
    .o_data (read_aligner_data          ),
    .o_idle (read_aligner_idle          )    //Note: Unused?
  );

  always_ff @(posedge clk) begin
    if (r_ready & r_valid)
      r_first <= r_last;

    if (rst)
      r_first <= 1'b1;
  end

  always @(posedge clk) begin
    if (ar_ready & ar_valid)
      read_aligner_shamt <= unaligned_ar_addr[$clog2(DATA_WIDTH/8)-1:0]; //FIXME URGENT

    if (rst)
      read_aligner_shamt <= '0;
  end

  always @(posedge clk) begin
    if (rst) stream_done <= 1'b1;
    else if(read_aligner_valid & read_aligner_end) stream_done <= 1'b0;
    else if(load_dstream.valid & load_dstream.end_flag) stream_done <= 1'b1;
    else stream_done <= stream_done;
  end

  fifo_interface #(.DATA_WIDTH(3+DATA_WIDTH)) load_buffers ();

  cva5_fifo_block #(
    .DATA_WIDTH(3+DATA_WIDTH),
    .FIFO_DEPTH(VLMAX       )
  ) load_buffer_block (
    .clk (clk         ),
    .rst (rst         ),
    .fifo(load_buffers)
  );


  always_comb begin
    load_buffers.potential_pop  = stream_done & en_ld & ~ld_data_w_valid_last_bit[DATA_WIDTH+1];
    load_buffers.potential_push = read_aligner_valid;
    load_buffers.pop            = load_buffers.potential_pop;
    load_buffers.push           = load_buffers.potential_push;
    load_buffers.data_in        = {read_aligner_valid,(read_aligner_valid & read_aligner_end), (read_aligner_valid & read_aligner_start), read_aligner_data};
  end
  
  
  generate
    if (BYPASS_LD_FIFO) begin : g_bypass
      assign ld_data_w_valid_last_bit = {read_aligner_valid,(read_aligner_valid & read_aligner_end), (read_aligner_valid & read_aligner_start), read_aligner_data};
    end 
    else begin 
      assign ld_data_w_valid_last_bit    = load_buffers.data_out;
    end
  endgenerate

  assign r_ready = 1'b1; // TODO fifo_ld_full??

endmodule