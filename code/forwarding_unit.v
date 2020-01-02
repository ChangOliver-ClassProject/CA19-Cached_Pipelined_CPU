module forwarding_unit(
	EX_RS1addr_i,
	EX_RS2addr_i,
	MEM_RDaddr_i,
	WB_RDaddr_i,
	MEM_RegWrite_i,
	WB_RegWrite_i,

	ForwardA_o,
	ForwardB_o
);

input		[4:0]	EX_RS1addr_i,
					EX_RS2addr_i,
					MEM_RDaddr_i,
					WB_RDaddr_i;
input 				MEM_RegWrite_i,
					WB_RegWrite_i;

output	reg [1:0]	ForwardA_o,
					ForwardB_o;	

always @(*) begin
	if (MEM_RegWrite_i && (MEM_RDaddr_i != 5'b0) && (MEM_RDaddr_i == EX_RS1addr_i))
		ForwardA_o = 2'b10;
	else if (WB_RegWrite_i && (WB_RDaddr_i != 5'b0) && (WB_RDaddr_i == EX_RS1addr_i))
		ForwardA_o = 2'b01;
	else
		ForwardA_o = 2'b00;

	if (MEM_RegWrite_i && (MEM_RDaddr_i != 5'b0) && (MEM_RDaddr_i == EX_RS2addr_i))
		ForwardB_o = 2'b10;
	else if (WB_RegWrite_i && (WB_RDaddr_i != 5'b0) && (WB_RDaddr_i == EX_RS2addr_i))
		ForwardB_o = 2'b01;
	else
		ForwardB_o = 2'b00;
end

endmodule
