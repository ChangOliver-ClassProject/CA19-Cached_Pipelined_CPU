module isEqual(
	RS1_i,
	RS2_i,
	is_equal_o
);

input 	[31:0]	RS1_i;
input 	[31:0]  RS2_i;
output 	reg		is_equal_o;

always@(RS1_i or RS2_i)
	is_equal_o = (RS1_i == RS2_i)? 1 : 0;

endmodule

