module EX_MEM(
	clk_i,
	Control_i,
	Instr_i,
	ALU_i,
	RS2data_i,
	RDaddr_i,
	stall_i,

    Instr_o,
	MemRead_o,
	MemWrite_o,
	Branch_o,
    Control_o, // MemtoReg_o, RegWrite_o
	ALU_o,
	RS2data_o,
	RDaddr_o,
);

input			clk_i;
input	[4:0]	Control_i,
				RDaddr_i;
input	[31:0]	Instr_i,
				ALU_i,
				RS2data_i;
input			stall_i;

reg 			MemRead_t;
reg 			MemWrite_t;
reg 			Branch_t;
reg 	[1:0]	Control_t;
reg 	[4:0]	RDaddr_t;
reg 	[31:0]	Instr_t,
				ALU_t,
				RS2data_t;

output	reg				MemRead_o;
output	reg				MemWrite_o;
output	reg				Branch_o;
output	reg 	[1:0]	Control_o;
output	reg 	[4:0]	RDaddr_o;
output	reg 	[31:0]	Instr_o,
						ALU_o,
						RS2data_o;

always @(posedge clk_i or negedge clk_i) begin
	if (~stall_i) begin
		if (clk_i) begin
			Instr_t		= Instr_i;
			MemRead_t	= Control_i[2];
			MemWrite_t	= Control_i[1];
			Branch_t	= Control_i[0];
			Control_t	= Control_i[4:3];
			ALU_t		= ALU_i;
			RS2data_t	= RS2data_i;
			RDaddr_t	= RDaddr_i;
		end
		else begin
			Instr_o		= Instr_t;
			MemRead_o	= MemRead_t;
			MemWrite_o	= MemWrite_t;
			Branch_o	= Branch_t;
			Control_o	= Control_t;
			ALU_o		= ALU_t;
			RS2data_o	= RS2data_t;
			RDaddr_o	= RDaddr_t;
		end
	end
end

endmodule