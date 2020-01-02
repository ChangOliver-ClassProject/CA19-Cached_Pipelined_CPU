module Control
(
	Op_i,
	is_equal_i,
	Control_o,
	flush_o,
);

input 			[6:0]	Op_i;
input	 				is_equal_i;
output 	reg 	[7:0]	Control_o;
output  reg 			flush_o;
reg 			[1:0]	ALUOp_o;
reg 					ALUSrc_o, RegWrite_o, MemtoReg_o, MemRead_o, MemWrite_o, Branch_o;

always@ (*)
begin
	case (Op_i)
		7'b0110011: 				//R-format
		begin
			ALUOp_o    = 2'b10;
			ALUSrc_o   = 0;
			RegWrite_o = 1;
			MemtoReg_o = 0;
			MemRead_o = 0;
			MemWrite_o = 0;
			Branch_o = 0;
		end
		7'b0010011: 				//I-format(addi)
		begin
			ALUOp_o    = 2'b00;
			ALUSrc_o   = 1;
			RegWrite_o = 1;
			MemtoReg_o = 0;
			MemRead_o = 0;
			MemWrite_o = 0;
			Branch_o = 0;
		end
		7'b0000011:					//ld
		begin
			ALUOp_o    = 2'b00;
			ALUSrc_o   = 1;
			RegWrite_o = 1;
			MemtoReg_o = 1;
			MemRead_o = 1;
			MemWrite_o = 0;
			Branch_o = 0;
		end
		7'b0100011:					//sd
		begin
			ALUOp_o    = 2'b00;
			ALUSrc_o   = 1;
			RegWrite_o = 0;
			MemtoReg_o = 0;			// X
			MemRead_o = 0;
			MemWrite_o = 1;
			Branch_o = 0;
		end
		7'b1100011:					//beq
		begin
			ALUOp_o    = 2'b01;
			ALUSrc_o   = 0;
			RegWrite_o = 0;
			MemtoReg_o = 0;			// X
			MemRead_o = 0;
			MemWrite_o = 0;
			Branch_o = 1;
		end
		7'b0000000:
		begin
			ALUOp_o    = 2'b00;
			ALUSrc_o   = 0;
			RegWrite_o = 0;
			MemtoReg_o = 0;			// X
			MemRead_o = 0;
			MemWrite_o = 0;
			Branch_o = 0;
		end
	endcase
	Control_o = {ALUOp_o, ALUSrc_o, RegWrite_o, MemtoReg_o, MemRead_o, MemWrite_o, Branch_o}; 
	flush_o = Branch_o & is_equal_i;
	if (flush_o)
		Branch_o = 0;
end

endmodule