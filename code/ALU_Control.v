module ALU_Control
(
	funct_i,
    ALUOp_i,
    ALUCtrl_o
);

input  		[1:0] ALUOp_i;
input  		[9:0] funct_i;
output reg  [2:0] ALUCtrl_o;

always@ (*)
begin
	case(ALUOp_i)
		2'b10: 	 	 	 			                //R-format
			case(funct_i)
				10'b0000000110: ALUCtrl_o = 3'b001; //or
				10'b0000000111: ALUCtrl_o = 3'b000; //and
				10'b0000000000: ALUCtrl_o = 3'b010; //add
				10'b0100000000: ALUCtrl_o = 3'b110; //sub
				10'b0000001000: ALUCtrl_o = 3'b011; //mul
			endcase
		2'b00:                                      //I-format 
								ALUCtrl_o = 3'b010; //addi
		2'b01:
								ALUCtrl_o = 3'b110; //branch
	endcase
end

endmodule