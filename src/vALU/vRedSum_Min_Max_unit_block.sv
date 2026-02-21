// `include "vMinMaxSelector.v"
// `include "vAdd_unit_block.v"
/*
(* keep_hierarchy = "yes" *) module vRedSum_min_max_unit_block #(
	parameter REQ_DATA_WIDTH  	= 32,
	parameter RESP_DATA_WIDTH 	= 64,
	parameter OPSEL_WIDTH     	= 2 ,
	parameter SEW_WIDTH       	= 2 ,
	parameter MIN_MAX_ENABLE  	= 1 ,
	parameter ENABLE_64_BIT 	= 1
) (
	input                             clk,
	input                             rst,
	input      [REQ_DATA_WIDTH*2-1:0] vec0,
	input                             en,
	input      [       SEW_WIDTH-1:0] sew,
	input      [     OPSEL_WIDTH-1:0] opSel,
	output reg [ RESP_DATA_WIDTH-1:0] out_vec
);

	wire [ RESP_DATA_WIDTH-1:0] op_out;
	wire [RESP_DATA_WIDTH+16:0] result;
	wire [ RESP_DATA_WIDTH-1:0]	minMax_result;

	if (ENABLE_64_BIT | REQ_DATA_WIDTH < 64) begin
		vAdd_unit_block #(.ENABLE_64_BIT(ENABLE_64_BIT)
		)	vAdd_unit0 (
			.clk   	(clk 									),
			.rst   	(rst 									),
			.vec0  	(vec0[REQ_DATA_WIDTH-1:0]				),
			.vec1  	(vec0[REQ_DATA_WIDTH*2-1:REQ_DATA_WIDTH]),
			.carry	('h0 									),
			.sew   	(sew									),
			.opSel 	('h0 									),
			.result	(result									)
		);
	end else begin
		vAdd_unit_block #(.ENABLE_64_BIT(ENABLE_64_BIT)
		)	vAdd_unit0 (
			.clk   	(clk 					),
			.rst   	(rst 					),
			.vec0  	({32'b0,vec0[31:0]}		),
			.vec1  	({32'b0,vec0[91:64]}	),
			.carry	('h0 					),
			.sew   	(sew					),
			.opSel 	('h0 					),
			.result	(result					)
		);
	end

	generate
		if(MIN_MAX_ENABLE) begin
			if (ENABLE_64_BIT | REQ_DATA_WIDTH < 64) begin
				vMinMaxSelector #(.ENABLE_64_BIT(ENABLE_64_BIT)
				) vMinMaxSelector0 (
					.vec0			(vec0[REQ_DATA_WIDTH-1:0]				),
					.vec1			(vec0[REQ_DATA_WIDTH*2-1:REQ_DATA_WIDTH]),
					.sub_result 	(result 								),
					.sew 			(sew 									),
					.minMax_sel 	(opSel[0] 								),
					.minMax_result 	(minMax_result 							),
					.equal			(										),
					.lt 			(										)
				);
			end else begin
				vMinMaxSelector #(.ENABLE_64_BIT(ENABLE_64_BIT)
				) vMinMaxSelector0 (
					.vec0			({32'b0,vec0[31:0]}		),
					.vec1			({32'b0,vec0[91:64]}	),
					.sub_result 	(result 				),
					.sew 			(sew 					),
					.minMax_sel 	(opSel[0] 				),
					.minMax_result 	(minMax_result 			),
					.equal			(						),
					.lt 			(						)
				);
			end
		end
	endgenerate

	generate
		if (REQ_DATA_WIDTH >= 64) begin
			assign op_out	= {result[78:71],result[68:61],result[58:51],result[48:41],result[38:31],result[28:21],result[18:11],result[8:1]};
		end else if (REQ_DATA_WIDTH >= 32) begin
			assign op_out	= {32'h0,result[38:31],result[28:21],result[18:11],result[8:1]};
		end else if (REQ_DATA_WIDTH >= 16) begin
			assign op_out	= {48'h0,result[18:11],result[8:1]};
		end else begin
			assign op_out	= {56'h0,result[8:1]};
		end

		always @(posedge clk) begin
			if (rst) begin
				out_vec <= 'h0;
			end else begin
				if (ENABLE_64_BIT | REQ_DATA_WIDTH < 64) begin
					out_vec <= en ? (opSel[1] ? minMax_result[REQ_DATA_WIDTH-1:0] : op_out[REQ_DATA_WIDTH-1:0]) : vec0[REQ_DATA_WIDTH-1:0];
				end else begin
					out_vec <= en ? (opSel[1] ? minMax_result[(REQ_DATA_WIDTH/2) - 1:0] : op_out[(REQ_DATA_WIDTH/2):0]) : vec0[(REQ_DATA_WIDTH/2):0];
				end
			end
		end
	endgenerate


endmodule
*/
(* keep_hierarchy = "yes" *)
module vRedSum_min_max_unit_block #(
    parameter REQ_DATA_WIDTH  = 32,
    parameter RESP_DATA_WIDTH = 64,
    parameter OPSEL_WIDTH     = 2,
    parameter SEW_WIDTH       = 2,
    parameter MIN_MAX_ENABLE  = 1,
    parameter ENABLE_64_BIT   = 1
) (
    input  logic                          clk,
    input  logic                          rst,
    input  logic [REQ_DATA_WIDTH * 2 - 1:0] vec0,
    input  logic                          en,
    input  logic [SEW_WIDTH - 1:0]        sew,
    input  logic [OPSEL_WIDTH - 1:0]      opSel,
    output reg   [RESP_DATA_WIDTH - 1:0]  out_vec
);
    
    // ... (wires and stage 1 registers are the same) ...
    wire [RESP_DATA_WIDTH - 1:0] op_out;
    wire [RESP_DATA_WIDTH + 16:0] result;
    wire [RESP_DATA_WIDTH - 1:0] minMax_result;

    reg [REQ_DATA_WIDTH * 2 - 1:0] s1_vec0;
    reg                            s1_en;
    reg [SEW_WIDTH - 1:0]          s1_sew;
    reg [OPSEL_WIDTH - 1:0]        s1_opSel;
    reg [REQ_DATA_WIDTH - 1:0]     s1_fallback_vec;

    // Instantiate the new, simplified 2-stage adder
    if (ENABLE_64_BIT | REQ_DATA_WIDTH < 64) begin
        vAdd_unit_block_2stage_simple #(.ENABLE_64_BIT(ENABLE_64_BIT)) vAdd_unit0 (
            .clk   (clk),
            .rst   (rst),
            .vec0  (vec0[REQ_DATA_WIDTH - 1:0]),
            .vec1  (vec0[REQ_DATA_WIDTH * 2 - 1:REQ_DATA_WIDTH]),
            .sew   (sew),
            .result(result)
        );
    end else begin
        vAdd_unit_block_2stage_simple #(.ENABLE_64_BIT(ENABLE_64_BIT)) vAdd_unit0 (
            .clk   (clk),
            .rst   (rst),
            .vec0  ({32'b0, vec0[31:0]}),
            .vec1  ({32'b0, vec0[91:64]}),
            .sew   (sew),
            .result(result)
        );
    end

    // ... (rest of the module logic remains unchanged) ...
    always @(posedge clk) begin
        if (rst) begin
            s1_vec0         <= '0;
            s1_en           <= 1'b0;
            s1_sew          <= '0;
            s1_opSel        <= '0;
            s1_fallback_vec <= '0;
        end else begin
            s1_vec0  <= vec0;
            s1_en    <= en;
            s1_sew   <= sew;
            s1_opSel <= opSel;
            if (ENABLE_64_BIT | REQ_DATA_WIDTH < 64) begin
                s1_fallback_vec <= vec0[REQ_DATA_WIDTH - 1:0];
            end else begin
                s1_fallback_vec <= vec0[(REQ_DATA_WIDTH / 2):0];
            end
        end
    end

    generate
        if (MIN_MAX_ENABLE) begin
            if (ENABLE_64_BIT | REQ_DATA_WIDTH < 64) begin
                vMinMaxSelector #(.ENABLE_64_BIT(ENABLE_64_BIT)) vMinMaxSelector0 (
                    .vec0         (s1_vec0[REQ_DATA_WIDTH - 1:0]),
                    .vec1         (s1_vec0[REQ_DATA_WIDTH * 2 - 1:REQ_DATA_WIDTH]),
                    .sub_result   (result),
                    .sew          (s1_sew),
                    .minMax_sel   (s1_opSel[0]),
                    .minMax_result(minMax_result),
                    .equal        (),
                    .lt           ()
                );
            end else begin
                vMinMaxSelector #(.ENABLE_64_BIT(ENABLE_64_BIT)) vMinMaxSelector0 (
                    .vec0         ({32'b0, s1_vec0[31:0]}),
                    .vec1         ({32'b0, s1_vec0[91:64]}),
                    .sub_result   (result),
                    .sew          (s1_sew),
                    .minMax_sel   (s1_opSel[0]),
                    .minMax_result(minMax_result),
                    .equal        (),
                    .lt           ()
                );
            end
        end
    endgenerate

    generate
        if (REQ_DATA_WIDTH >= 64) begin
            assign op_out = {result[78:71], result[68:61], result[58:51], result[48:41], result[38:31], result[28:21], result[18:11], result[8:1]};
        end else if (REQ_DATA_WIDTH >= 32) begin
            assign op_out = {32'h0, result[38:31], result[28:21], result[18:11], result[8:1]};
        end else if (REQ_DATA_WIDTH >= 16) begin
            assign op_out = {48'h0, result[18:11], result[8:1]};
        end else begin
            assign op_out = {56'h0, result[8:1]};
        end

        always @(posedge clk) begin
            if (rst) begin
                out_vec <= 'h0;
            end else begin
                if (ENABLE_64_BIT | REQ_DATA_WIDTH < 64) begin
                    out_vec <= s1_en ? (s1_opSel[1] ? minMax_result[REQ_DATA_WIDTH - 1:0] : op_out[REQ_DATA_WIDTH - 1:0]) : s1_fallback_vec;
                end else begin
                    out_vec <= s1_en ? (s1_opSel[1] ? minMax_result[(REQ_DATA_WIDTH / 2) - 1:0] : op_out[(REQ_DATA_WIDTH / 2):0]) : s1_fallback_vec;
                end
            end
        end
    endgenerate

endmodule