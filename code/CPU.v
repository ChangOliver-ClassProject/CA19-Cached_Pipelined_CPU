module CPU
(
	clk_i,
	rst_i,
	start_i,
   
	mem_data_i, 
	mem_ack_i, 	
	mem_data_o, 
	mem_addr_o, 	
	mem_enable_o, 
	mem_write_o
);

//input
input clk_i;
input rst_i;
input start_i;

//
// to Data Memory interface		
//
input	[256-1:0]	mem_data_i; 
input				mem_ack_i; 	
output	[256-1:0]	mem_data_o; 
output	[32-1:0]	mem_addr_o; 	
output				mem_enable_o; 
output				mem_write_o; 

//
// add your project1 here!
//
wire        [31:0]  IF_PC_four;
wire        [31:0]  IF_PC_in;
wire        [31:0]  IF_PC_out;
wire        [31:0]  IF_Instr;

wire        [31:0]  ID_PC_branch;
wire        [31:0]  ID_Instr;
wire        [7:0]   ID_Control;
wire        [7:0]   ID_Mux_Control;
wire        [31:0]  ID_RS1data;
wire        [31:0]  ID_RS2data;
wire        [31:0]  ID_PC;
wire        [31:0]  ID_SignExtendImm;
wire                ID_Flush;
wire                ID_isEqual;

wire                HDU_PC;
wire                HDU_IF_ID;
wire                HDU_select;

wire        [1:0]   EX_ALUOp;
wire                EX_ALUSrc;
wire        [4:0]   EX_Control;
wire        [2:0]   EX_ALUControl;
wire        [31:0]  EX_Instr;
wire        [31:0]  EX_RS1;
wire        [31:0]  EX_RS2;
wire        [31:0]  EX_Mux1;
wire        [31:0]  EX_Mux2;
wire        [4:0]   EX_RS1addr;
wire        [4:0]   EX_RS2addr;
wire        [4:0]   EX_RDaddr;
wire        [31:0]  EX_SignExtendImm;
wire        [31:0]  EX_ALURS2;
wire        [31:0]  EX_ALU;
wire                EX_Zero;
wire        [1:0]   EX_FU1;
wire        [1:0]   EX_FU2;

wire        [31:0]  MEM_ALU;
wire                MEM_Branch;
wire        [31:0]  MEM_Mux2;
wire        [31:0]  MEM_Data_Memory_out;
wire        [31:0]  MEM_Instr;
wire                MEM_MemWrite;
wire                MEM_MemRead;
wire        [1:0]   MEM_Control;
wire        [4:0]   MEM_RDaddr;

wire 	 	 	    Stall;

wire                WB_RegWrite;
wire                WB_MemtoReg;
wire        [4:0]   WB_RDaddr;
wire        [31:0]  WB_Instr;
wire        [31:0]  WB_Memory;
wire        [31:0]  WB_ALU;
wire        [31:0]  WB_Mux;

reg                 ID_Flush_reg = 1'b0;
reg                 PCWrite_reg = 1'b1;
reg                 IF_ID_write_reg = 1'b1;
reg                 Mux_Control_reg = 1'b0;
reg                 RegWrite_reg = 1'b0;
reg                 stall_reg = 1'b0;

always @(ID_Flush)
    ID_Flush_reg = ID_Flush;

always @(HDU_PC)
    PCWrite_reg = HDU_PC;

always @(HDU_IF_ID)
    IF_ID_write_reg = HDU_IF_ID;

always @(HDU_select)
    Mux_Control_reg = HDU_select;

always @(WB_RegWrite)
    RegWrite_reg = WB_RegWrite;

always @(Stall)
    stall_reg = Stall;

Mux32_2 MUX_PC(
    .data1_i        (IF_PC_four),
    .data2_i        (ID_PC_branch),
    .select_i       (ID_Flush_reg),
    .data_o         (IF_PC_in)
);

PC PC
(
	.clk_i(clk_i),
	.rst_i(rst_i),
	.start_i(start_i),
	.stall_i(stall_reg),
	.PCWrite_i(PCWrite_reg),
	.pc_i(IF_PC_in),
	.pc_o(IF_PC_out)
);

Instruction_Memory Instruction_Memory(
	.addr_i(IF_PC_out), 
	.instr_o(IF_Instr)
);

parameter IF_PCImm = {{29{1'b0}}, 3'b100};
Adder Add_PC(
    .data1_i    (IF_PC_out),
    .data2_i    (IF_PCImm),
    .data_o     (IF_PC_four)
);

IF_ID Pipe_IF_ID(
    .clk_i          (clk_i),
    .PC_i           (IF_PC_out),
    .Instruction_i  (IF_Instr),
    .IF_ID_write_i  (IF_ID_write_reg), 
    .IF_ID_flush_i  (ID_Flush_reg),
	.stall_i		(stall_reg),
    .PC_o           (ID_PC),
    .Instruction_o  (ID_Instr)
);

Control Control(
    .Op_i       (ID_Instr[6:0]),
    .is_equal_i (ID_isEqual),
    .Control_o  (ID_Control),   // MUX_Control.data1_i
    .flush_o    (ID_Flush)
);

Mux8_2 MUX_Control(
    .data1_i    (ID_Control), // Control.Control_o
    .data2_i    (8'b0),
    .select_i   (Mux_Control_reg),
    .data_o     (ID_Mux_Control)
);

HDU HDU(
    .ID_Instruction_i   (ID_Instr), 
    .EX_RDaddr_i        (EX_RDaddr),
    .EX_Control_i       (EX_Control),
    .select_o           (HDU_select),
    .PCWrite_o          (HDU_PC),
    .IF_ID_write_o      (HDU_IF_ID)
);

Adder Add_Branch(
    .data1_i    (ID_SignExtendImm << 1),
    .data2_i    (ID_PC),
    .data_o     (ID_PC_branch)
);

Registers Registers(
    .clk_i       (clk_i),
    .RS1addr_i   (ID_Instr[19:15]),
    .RS2addr_i   (ID_Instr[24:20]),
    .RDaddr_i    (WB_Instr[11:7]),
    .RDdata_i    (WB_Mux),
    .RegWrite_i  (RegWrite_reg), 
    .RS1data_o   (ID_RS1data), 
    .RS2data_o   (ID_RS2data) 
);

isEqual isEqual(
    .RS1_i      (ID_RS1data),
    .RS2_i      (ID_RS2data),
    .is_equal_o (ID_isEqual)
);

Sign_Extend Sign_Extend(
    .data_i     (ID_Instr),
    .data_o     (ID_SignExtendImm)
);

ID_EX Pipe_ID_EX(
    .clk_i              (clk_i),
    .Control_i          (ID_Mux_Control),
    .Instruction_i      (ID_Instr),
    .RS1_i              (ID_RS1data),
    .RS2_i              (ID_RS2data),
    .sign_extended_i    (ID_SignExtendImm),
    .RS1addr_i          (ID_Instr[19:15]),
    .RS2addr_i          (ID_Instr[24:20]),
    .RDaddr_i           (ID_Instr[11:7]),
	.stall_i			(stall_reg),

    .ALUOp_o            (EX_ALUOp),
    .ALUSrc_o           (EX_ALUSrc),
    .Control_o          (EX_Control), // RegWrite_o, MemtoReg_o, MemRead_o, MemWrite_o, Branch_o
    .Instruction_o      (EX_Instr),
    .RS1_o              (EX_RS1),
    .RS2_o              (EX_RS2),
    .sign_extended_o    (EX_SignExtendImm),
    .RS1addr_o          (EX_RS1addr),
    .RS2addr_o          (EX_RS2addr),
    .RDaddr_o           (EX_RDaddr)
);

Mux32_3 MUX_RS1(
    .data1_i    (EX_RS1),
    .data2_i    (WB_Mux),
    .data3_i    (MEM_ALU), 
    .select_i   (EX_FU1), // from FU
    .data_o     (EX_Mux1)
);

Mux32_3 MUX_RS2(
    .data1_i    (EX_RS2),
    .data2_i    (WB_Mux),
    .data3_i    (MEM_ALU),
    .select_i   (EX_FU2), // from FU
    .data_o     (EX_Mux2)
);

Mux32_2 Mux_ALUSrc(
    .data1_i    (EX_Mux2),
    .data2_i    (EX_SignExtendImm),
    .select_i   (EX_ALUSrc),
    .data_o     (EX_ALURS2)    
);

ALU ALU(
    .data1_i    (EX_Mux1),
    .data2_i    (EX_ALURS2),
    .ALUCtrl_i  (EX_ALUControl),
    .data_o     (EX_ALU),
    .Zero_o     (EX_Zero)
);

forwarding_unit FU(
    .EX_RS1addr_i   (EX_RS1addr),
    .EX_RS2addr_i   (EX_RS2addr),
    .MEM_RDaddr_i   (MEM_RDaddr),
    .WB_RDaddr_i    (WB_RDaddr),
    .MEM_RegWrite_i (MEM_Control[1]),
    .WB_RegWrite_i  (RegWrite_reg),
    .ForwardA_o     (EX_FU1),
    .ForwardB_o     (EX_FU2)
);

ALU_Control ALU_Control(
    .funct_i    ({EX_Instr[31:25], EX_Instr[14:12]}),
    .ALUOp_i    (EX_ALUOp),
    .ALUCtrl_o  (EX_ALUControl)
);

EX_MEM Pipe_EX_MEM(
    .clk_i      (clk_i),
    .Control_i  (EX_Control),
    .Instr_i    (EX_Instr),
    .ALU_i      (EX_ALU),
    .RS2data_i  (EX_Mux2),
    .RDaddr_i   (EX_RDaddr),
	.stall_i	(stall_reg),

    .Instr_o    (MEM_Instr),
    .MemRead_o  (MEM_MemRead),
    .MemWrite_o (MEM_MemWrite),
    .Branch_o   (MEM_Branch),
    .Control_o  (MEM_Control), // RegWrite_o, MemtoReg_o
    .ALU_o      (MEM_ALU),
    .RS2data_o  (MEM_Mux2),
    .RDaddr_o   (MEM_RDaddr)
);


MEM_WB Pipe_MEM_WB(
    .clk_i          (clk_i),
    .Control_i      (MEM_Control),
    .Instruction_i  (MEM_Instr),
    .Memory_i       (MEM_Data_Memory_out),
    .ALU_i          (MEM_ALU),
    .RDaddr_i       (MEM_RDaddr),
	.stall_i		(stall_reg),

    .Instruction_o  (WB_Instr),
    .RegWrite_o     (WB_RegWrite),
    .MemtoReg_o     (WB_MemtoReg),
    .Memory_o       (WB_Memory),
    .ALU_o          (WB_ALU),
    .RDaddr_o       (WB_RDaddr)
);

Mux32_2 MUX_Output(
    .data1_i    (WB_ALU),
    .data2_i    (WB_Memory),
    .select_i   (WB_MemtoReg),
    .data_o     (WB_Mux)
);

//data cache
dcache_top dcache(
    // System clock, reset and stall
	.clk_i(clk_i), 
	.rst_i(rst_i),
	
	// to Data Memory interface		
	.mem_data_i(mem_data_i), 
	.mem_ack_i(mem_ack_i), 	
	.mem_data_o(mem_data_o), 
	.mem_addr_o(mem_addr_o), 	
	.mem_enable_o(mem_enable_o), 
	.mem_write_o(mem_write_o), 
	
	// to CPU interface	
	.p1_data_i(MEM_Mux2), 
	.p1_addr_i(MEM_ALU), 	
	.p1_MemRead_i(MEM_MemRead), 
	.p1_MemWrite_i(MEM_MemWrite), 
	.p1_data_o(MEM_Data_Memory_out), 
	.p1_stall_o(Stall)
);

endmodule
