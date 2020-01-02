module IF_ID(
	clk_i,
	PC_i,
	Instruction_i,
	IF_ID_write_i, 
	IF_ID_flush_i,
	stall_i,

	PC_o,
	Instruction_o
);

input			clk_i;
input	[31:0]	PC_i;
input	[31:0]	Instruction_i;
input			IF_ID_write_i;
input			IF_ID_flush_i;
input			stall_i;

output	reg [31:0]	PC_o;
output	reg [31:0]	Instruction_o;

reg 	[31:0]	PC_t;
reg 	[31:0]	Instruction_t;

always @(posedge clk_i or negedge clk_i) begin
	if (~stall_i) begin
		if (clk_i == 1 && IF_ID_write_i == 1) begin
			PC_t = PC_i;
			if (IF_ID_flush_i == 1)
				Instruction_t = 32'b0;
			else
				Instruction_t = Instruction_i;
		end
		else if (clk_i == 0) begin
			PC_o 			= PC_t;
			Instruction_o	= Instruction_t;
		end
	end
end

endmodule