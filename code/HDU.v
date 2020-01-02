module HDU (
	ID_Instruction_i, 
	EX_RDaddr_i,
	EX_Control_i,

	select_o,
	PCWrite_o,
	IF_ID_write_o
);

input 	[31:0] ID_Instruction_i;
input	[4:0]  EX_RDaddr_i;
input	[4:0]  EX_Control_i;

output  reg select_o;
output  reg PCWrite_o;
output  reg IF_ID_write_o;

always@(ID_Instruction_i or EX_RDaddr_i or EX_Control_i) begin
	if (EX_Control_i[2] && ((EX_RDaddr_i == ID_Instruction_i[19:15]) || (EX_RDaddr_i == ID_Instruction_i[24:20]))) begin
		select_o = 1;
		PCWrite_o = 0;
		IF_ID_write_o = 0;
	end 
	else begin
		select_o = 0;
		PCWrite_o = 1;
		IF_ID_write_o = 1;
	end
end

endmodule