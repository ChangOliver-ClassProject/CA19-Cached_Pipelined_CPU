module MEM_WB(
	clk_i,
	Control_i,
	Instruction_i,
	Memory_i,
	ALU_i,
	RDaddr_i,
	stall_i,

    Instruction_o,
	RegWrite_o,
    MemtoReg_o,
    Memory_o,
	ALU_o,
	RDaddr_o
);

input			clk_i;
input	[1:0]	Control_i;
input 	[4:0]	RDaddr_i;
input	[31:0]	Instruction_i,
				Memory_i,
				ALU_i;
input			stall_i;

output	reg			RegWrite_o;
output	reg	 		MemtoReg_o;
output	reg [4:0]	RDaddr_o;
output	reg [31:0]	Instruction_o,
					Memory_o,
					ALU_o;


reg 			RegWrite_t;
reg 			MemtoReg_t;
reg 	[4:0]	RDaddr_t;
reg 	[31:0]	Instruction_t,
				Memory_t,
				ALU_t;

always @(posedge clk_i or negedge clk_i) begin
	if (~stall_i) begin
		if (clk_i == 1) begin
			RegWrite_t 		= Control_i[1];
			MemtoReg_t 		= Control_i[0];
			RDaddr_t		= RDaddr_i;
			Instruction_t 	= Instruction_i;
			Memory_t 		= Memory_i;
			ALU_t			= ALU_i;
		end
		else begin 
			RegWrite_o 		= RegWrite_t;
			MemtoReg_o 		= MemtoReg_t;
			RDaddr_o		= RDaddr_t;
			Instruction_o 	= Instruction_t;
			Memory_o		= Memory_t;
			ALU_o 			= ALU_t;
		end
	end
end

endmodule