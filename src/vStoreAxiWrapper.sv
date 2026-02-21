module vStoreAxiWrapper
    import opcodes::*;
    import rvvLitePkg::*;
    import cva5_config::*;
    import riscv_types::*;
    import cva5_types::*;
    import cxu_types::*;
    import cx_dma_types::*;
(
    input  logic                       clk,
    input  logic                       rst,
    input  logic                       stall, // Stall signal for pushing to store_req_buffer
    input  logic [7:0]                 unaligned_len,  // awlen (number of transfers - 1)
    input  logic [MEM_ADDR_WIDTH-1:0]  unaligned_addr, // Start address
    input  logic [TRACK_ID_WIDTH-1:0]  track_id,       // AXI ID
    input  logic                       is_vstore,      // Qualifies the request
    input  dstream_t                   store_dstream,
    gen_interface.master               m_write_req,    // AXI Address Write Channel
    stream_interface.master            m_write_stream  // AXI Data Write Channel
);

    // AXI Address Write (AW) Channel internal signals
    logic                       aw_ready_internal; // From m_write_req.ready
    logic                       aw_valid_internal; // To m_write_req.valid
    logic [2:0]                 aw_size_internal;  // AXI size (log2 of bytes per transfer)
    logic [7:0]                 aw_len_internal;   // AXI len (burst length - 1)
    logic [TRACK_ID_WIDTH-1:0]  aw_id_internal;
    logic [MEM_ADDR_WIDTH-1:0]  aw_addr_internal;

    // AXI Data Write (W) Channel internal signals
    logic                       w_ready_internal;  // From m_write_stream.tready
    logic                       w_valid_internal;  // To m_write_stream.tvalid
    logic [DATA_WIDTH-1:0]      w_data_internal;
    logic [DW_B-1:0]            w_strb_internal;
    logic                       w_last_internal;

    // Intermediate signals for address/length from store_req_buffer
    logic [MEM_ADDR_WIDTH-1:0]  buffered_unaligned_addr;
    logic [7:0]                 buffered_unaligned_len;

    // Write Aligner Interface Signals
    logic                       write_aligner_out_valid;
    logic                       write_aligner_out_start; // Unused in this module, but kept for interface consistency
    logic                       write_aligner_out_end;
    logic [DATA_WIDTH-1:0]      write_aligner_out_data;
    logic [DW_B-1:0]            write_aligner_out_be;
    logic                       write_aligner_out_idle;  // Potentially unused
    logic [DW_B_BITS > 0 ? DW_B_BITS-1 : 0 : 0] write_aligner_in_shamt; // Shift amount for aligner

    // FIFO for store requests (AW channel info)
    // Consider increasing FIFO_DEPTH if aw_ready_internal often deasserts,
    // and is_vstore can generate requests faster than AXI AW channel can accept.
    localparam STORE_REQ_FIFO_DEPTH = 1; // Original depth
    fifo_interface #(.DATA_WIDTH(TRACK_ID_WIDTH + 8 + MEM_ADDR_WIDTH)) store_req_buffer ();
    cva5_fifo #(
        .DATA_WIDTH(TRACK_ID_WIDTH + 8 + MEM_ADDR_WIDTH),
        .FIFO_DEPTH(STORE_REQ_FIFO_DEPTH)
    ) store_req_buffer_block (
        .clk(clk),
        .rst(rst),
        .fifo(store_req_buffer)
    );

    always_comb begin
        store_req_buffer.potential_pop = aw_ready_internal & aw_valid_internal;
        store_req_buffer.pop = store_req_buffer.potential_pop; // Pop if AXI AW is ready and we have a valid req
        store_req_buffer.potential_push = is_vstore & ~stall;
        store_req_buffer.push = store_req_buffer.potential_push; //Push if it's a vstore and not stalled
        store_req_buffer.data_in = {track_id, unaligned_len, unaligned_addr};

        {aw_id_internal, buffered_unaligned_len, buffered_unaligned_addr} = store_req_buffer.data_out;
    end
        // Adjust burst length if start address is unaligned
        logic [DW_B_BITS > 0 ? DW_B_BITS-1 : 0 : 0] start_offset;
        assign start_offset = (DW_B_BITS > 0) ? buffered_unaligned_addr[DW_B_BITS-1:0] : '0;
        assign aw_len_internal  = (start_offset == '0) ? buffered_unaligned_len : (buffered_unaligned_len + 1);
    // Combinational logic for preparing AXI AW channel signals
    always_comb begin
        aw_valid_internal = store_req_buffer.valid; // Valid if request FIFO has data

        // Align address to data bus width boundary
        if (DW_B_BITS > 0) begin
            aw_addr_internal = (buffered_unaligned_addr >> DW_B_BITS) << DW_B_BITS;
        end else begin // DATA_WIDTH <= 8 bits
            aw_addr_internal = buffered_unaligned_addr;
        end



        // Determine AXI size (AxSIZE) based on DATA_WIDTH
        // AXI Size is log2(number of bytes in transfer)
        case (DW_B)
            1:  aw_size_internal = 3'b000; // 1 byte
            2:  aw_size_internal = 3'b001; // 2 bytes
            4:  aw_size_internal = 3'b010; // 4 bytes
            8:  aw_size_internal = 3'b011; // 8 bytes (e.g., DATA_WIDTH = 64)
            16: aw_size_internal = 3'b100; // 16 bytes (e.g., DATA_WIDTH = 128)
            // Add cases for 32, 64, 128 bytes if wider DATA_WIDTH is supported by AXI
            default: aw_size_internal = 3'b011; // Default for common 64-bit case or assert error
        endcase
    end

    // FIFO for store data (W channel info)
    // Depth VLMAX = VLEN/8 (e.g., 64 for VLEN=512, DATA_WIDTH=64)
    fifo_interface #(.DATA_WIDTH(1 + DW_B + DATA_WIDTH)) store_data_buffer (); // w_last + w_strb + w_data
    cva5_fifo #(
        .DATA_WIDTH(1 + DW_B + DATA_WIDTH),
        .FIFO_DEPTH(VLMAX)
    ) store_data_buffer_block (
        .clk(clk),
        .rst(rst),
        .fifo(store_data_buffer)
    );

    logic [1 + DW_B + DATA_WIDTH - 1 : 0] packed_w_data_from_fifo;

    always_comb begin
        store_data_buffer.potential_pop = w_ready_internal & w_valid_internal;
        store_data_buffer.pop  = store_data_buffer.potential_pop; // Pop if AXI W is ready and we have valid data
        store_data_buffer.potential_push = write_aligner_out_valid;
        store_data_buffer.push = store_data_buffer.potential_push;             // Push when aligner has valid output
        store_data_buffer.data_in = {write_aligner_out_end, write_aligner_out_be, write_aligner_out_data};

        packed_w_data_from_fifo = store_data_buffer.data_out;
    end

    // Combinational logic for preparing AXI W channel signals
    always_comb begin
        w_valid_internal = store_data_buffer.valid; // Valid if data FIFO has data
        w_data_internal  = packed_w_data_from_fifo[DATA_WIDTH-1:0];
        w_strb_internal  = packed_w_data_from_fifo[DATA_WIDTH + DW_B -1 : DATA_WIDTH];
        w_last_internal  = packed_w_data_from_fifo[1 + DW_B + DATA_WIDTH -1];
    end

    // Instantiate Write Burst Aligner
    assign write_aligner_in_shamt = (DW_B_BITS > 0) ? buffered_unaligned_addr[DW_B_BITS-1:0] : '0;

    write_burst_aligner #(
        .DATA_WIDTH(DATA_WIDTH)
    ) write_burst_aligner_block (
        .clk(clk),
        .rst(rst),
        .i_valid(store_dstream.valid),
        .i_start(store_dstream.start_flag),
        .i_end(store_dstream.end_flag),
        .i_data(store_dstream.data),
        .i_be(store_dstream.be),
        .i_shamt(write_aligner_in_shamt),
        .o_valid(write_aligner_out_valid),
        .o_start(write_aligner_out_start), // Note: o_start may not be directly used by store_data_buffer
        .o_end(write_aligner_out_end),
        .o_data(write_aligner_out_data),
        .o_be(write_aligner_out_be),
        .o_idle(write_aligner_out_idle)     // Note: o_idle marked as unused in original
    );

    mem_packet_t                mem_packet;

    assign mem_packet = '{base_address  : aw_addr_internal,
                          end_address   : aw_addr_internal + (DW_B * (aw_len_internal + 1)) - 1,
                          size          : aw_size_internal,
                          stride        : 32'b1};

    // AXI Master Interface Connections
    // Address Write (AW) Channel
    assign m_write_req.valid = aw_valid_internal;
    assign aw_ready_internal = m_write_req.ready;
    assign m_write_req.data  = mem_packet;
                                 // Or stride could be '1' if it means something else in your gen_interface
    assign m_write_req.id    = aw_id_internal;

    // Data Write (W) Channel
    assign m_write_stream.tvalid = w_valid_internal;
    assign w_ready_internal      = m_write_stream.tready;
    assign m_write_stream.tdata  = w_data_internal;
    assign m_write_stream.tstrb  = w_strb_internal;
    assign m_write_stream.tlast  = w_last_internal;
    assign m_write_stream.tid    = '0; // TID unused on W channel for AXI4

    // Write Response (B) Channel
    // Module is always ready to accept write responses.
    // Ensure this is acceptable; if responses need processing or can cause backpressure,
    // more complex bready logic would be needed.
    // The gen_interface.master for m_write_req likely includes bvalid, bresp, bid as inputs.
    // These are not explicitly used here other than being ready.
    // assign m_write_req.bready = 1'b1; // Assuming bready is part of m_write_req interface
    // If gen_interface does not bundle B channel with AW, then a separate B channel interface is needed.
    // The original code had `assign b_ready = 1;` without connecting it.
    // Assuming bready is part of the `gen_interface.master m_write_req`.
    // If `gen_interface` is purely for AW, then you need a separate B channel slave interface.
    // Given the structure, it's likely `m_write_req` bundles B channel response ready.
    // If `m_write_req.bready` exists:
    // assign m_write_req.bready = 1'b1;

endmodule