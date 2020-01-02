module ALU
(
	data1_i,
    data2_i,
    ALUCtrl_i,
    data_o,
    Zero_o
);

input      [31:0]   data1_i, data2_i;
input      [2:0]    ALUCtrl_i;
output reg [31:0]   data_o;
output reg          Zero_o;

always@ (*)
begin
	case(ALUCtrl_i)
		3'b001: data_o = data1_i | data2_i;     //or
		3'b000: data_o = data1_i & data2_i;     //and			
		3'b010: data_o = data1_i + data2_i;     //add, addi
		3'b110: data_o = data1_i - data2_i;     //sub			
		3'b011: data_o = data1_i * data2_i;     //mul
	endcase

	Zero_o = (data_o) ? 0 : 1;
end

endmodule