module ID_EX(
	clk_i,
	Control_i,
	Instruction_i,
	RS1_i,
	RS2_i,
	sign_extended_i,
	RS1addr_i,
	RS2addr_i,
	RDaddr_i,
	stall_i,
	
	ALUOp_o,
    ALUSrc_o,
    Control_o, // RegWrite_o, MemtoReg_o, MemRead_o, MemWrite_o, Branch_o
	Instruction_o,
	RS1_o,
	RS2_o,
	sign_extended_o,
	RS1addr_o,
	RS2addr_o,
	RDaddr_o
);

input				clk_i, stall_i;
input		[7:0]	Control_i;
input		[31:0]	Instruction_i,
					RS1_i,
					RS2_i,
					sign_extended_i;
input 		[4:0]	RS1addr_i,
					RS2addr_i,
					RDaddr_i;

reg 		[1:0]	ALUOp_t;
reg					ALUSrc_t;
reg			[4:0]	Control_t;
reg			[31:0]	Instruction_t,
					RS1_t,
					RS2_t,
					sign_extended_t;
reg 		[4:0]	RS1addr_t,
					RS2addr_t,
					RDaddr_t;


output	reg [1:0]	ALUOp_o;
output	reg     	ALUSrc_o;
output	reg [4:0]   Control_o; // RegWrite_o, MemtoReg_o, MemRead_o, MemWrite_o, Branch_o
output	reg [31:0]	Instruction_o,
					RS1_o,
					RS2_o,
					sign_extended_o;
output	reg [4:0]	RS1addr_o,
					RS2addr_o,
					RDaddr_o;


always @(posedge clk_i or negedge clk_i) begin
	if (~stall_i) begin
		if (clk_i) begin
			ALUOp_t 		= Control_i[7:6];
			ALUSrc_t 		= Control_i[5];
			Control_t 		= Control_i[4:0];
			Instruction_t 	= Instruction_i;
			RS1_t 			= RS1_i;
			RS2_t 			= RS2_i;
			sign_extended_t = sign_extended_i;
			RS1addr_t 		= RS1addr_i;
			RS2addr_t 		= RS2addr_i;
			RDaddr_t 		= RDaddr_i;
		end
		else begin
			ALUOp_o 				= ALUOp_t;
			ALUSrc_o 				= ALUSrc_t;
			Control_o 				= Control_t;
			Instruction_o 			= Instruction_t;
			RS1_o 					= RS1_t;
			RS2_o 					= RS2_t; //if ALUOp_t = 00 then select sign_extended
			sign_extended_o			= sign_extended_t;
			RS1addr_o 				= RS1addr_t;
			RS2addr_o 				= RS2addr_t;
			RDaddr_o 				= RDaddr_t;
		end
	end
end

endmodule