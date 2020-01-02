module Sign_Extend
(
	data_i,
	data_o
);

input  			[31:0] 	data_i;
output 	reg 	[31:0] 	data_o;


always@ (*)
begin
	case (data_i[6:0])
		7'b0010011: data_o = {{20{data_i[31]}}, data_i[31:20]};											//I-format(addi)		
		7'b0000011:	data_o = {{20{data_i[31]}}, data_i[31:20]};											//ld		
		7'b0100011:	data_o = {{20{data_i[31]}}, data_i[31:25], data_i[11:7]};							//sd
		7'b1100011: data_o = {{20{data_i[31]}}, data_i[31], data_i[7], data_i[30:25], data_i[11:8]};	//beq
	endcase
end

endmodule


