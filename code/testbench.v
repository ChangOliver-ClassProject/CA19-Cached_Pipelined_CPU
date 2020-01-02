`define CYCLE_TIME 50

module TestBench;

reg                Clk;
reg                Start;
reg                Reset;
integer            i, outfile, outfile2, counter;
reg                flag;
reg        [26:0]  address;
reg        [23:0]  tag;
reg        [4:0]   index;

wire    [256-1:0]  mem_cpu_data; 
wire               mem_cpu_ack;     
wire    [256-1:0]  cpu_mem_data; 
wire    [32-1:0]   cpu_mem_addr;     
wire               cpu_mem_enable; 
wire               cpu_mem_write; 

always #(`CYCLE_TIME/2) Clk = ~Clk;    

CPU CPU(
    .clk_i  (Clk),
    .rst_i  (Reset),
    .start_i(Start),
    
    .mem_data_i(mem_cpu_data), 
    .mem_ack_i(mem_cpu_ack),     
    .mem_data_o(cpu_mem_data), 
    .mem_addr_o(cpu_mem_addr),     
    .mem_enable_o(cpu_mem_enable), 
    .mem_write_o(cpu_mem_write)
);

Data_Memory Data_Memory(
    .clk_i    (Clk),
    .rst_i    (Reset),
    .addr_i   (cpu_mem_addr),
    .data_i   (cpu_mem_data),
    .enable_i (cpu_mem_enable),
    .write_i  (cpu_mem_write),
    .ack_o    (mem_cpu_ack),
    .data_o   (mem_cpu_data)
);
  
initial begin
    $dumpfile("CPU.vcd");
    $dumpvars;
    counter = 0;
    
    // initialize instruction memory (1KB)
    for(i=0; i<255; i=i+1) begin
        CPU.Instruction_Memory.memory[i] = 32'b0;
    end
    
    // initialize data memory    (16KB)
    for(i=0; i<512; i=i+1) begin
        Data_Memory.memory[i] = 256'b0;
    end
        
    // initialize cache memory    (1KB)
    for(i=0; i<32; i=i+1) begin
        CPU.dcache.dcache_tag_sram.memory[i] = 24'b0;
        CPU.dcache.dcache_data_sram.memory[i] = 256'b0;
    end
    
    // initialize Register File
    for(i=0; i<32; i=i+1) begin
        CPU.Registers.register[i] = 32'b0;
    end

 // initialize pipeline registers
    CPU.Control.ALUOp_o = 2'b0;
    CPU.Control.ALUSrc_o = 0;
    CPU.Control.RegWrite_o = 0;
    CPU.Control.MemtoReg_o = 0;
    CPU.Control.MemRead_o = 0;
    CPU.Control.MemWrite_o = 0;
    CPU.Control.Branch_o = 0;

    CPU.Pipe_IF_ID.PC_t = 32'b0;
    CPU.Pipe_IF_ID.Instruction_t = 32'b0;

    CPU.Pipe_ID_EX.ALUOp_t = 2'b0;
    CPU.Pipe_ID_EX.ALUSrc_t = 0;
    CPU.Pipe_ID_EX.Control_t = 5'b0;
    CPU.Pipe_ID_EX.Instruction_t = 32'b0;
    CPU.Pipe_ID_EX.RS1_t = 0;
    CPU.Pipe_ID_EX.RS2_t = 0;
    CPU.Pipe_ID_EX.sign_extended_t = 0;
    CPU.Pipe_ID_EX.RS1addr_t = 5'b0;
    CPU.Pipe_ID_EX.RS2addr_t = 5'b0;
    CPU.Pipe_ID_EX.RDaddr_t = 5'b0;

    CPU.Pipe_EX_MEM.MemRead_t = 0;
    CPU.Pipe_EX_MEM.MemWrite_t = 0;
    CPU.Pipe_EX_MEM.Branch_t = 0;
    CPU.Pipe_EX_MEM.Control_t = 2'b0;
    CPU.Pipe_EX_MEM.RDaddr_t = 5'b0;
    CPU.Pipe_EX_MEM.Instr_t = 32'b0;
    CPU.Pipe_EX_MEM.ALU_t = 32'b0;
    CPU.Pipe_EX_MEM.RS2data_t = 32'b0;

    CPU.Pipe_MEM_WB.RegWrite_t = 0;
    CPU.Pipe_MEM_WB.MemtoReg_t = 0;
    CPU.Pipe_MEM_WB.RDaddr_t = 5'b0;
    CPU.Pipe_MEM_WB.Instruction_t = 32'b0;
    CPU.Pipe_MEM_WB.Memory_t = 32'b0;
    CPU.Pipe_MEM_WB.ALU_t = 32'b0;
    
    // Load instructions into instruction memory
    $readmemb("../testdata/instruction.txt", CPU.Instruction_Memory.memory);
    
    // Open output file
    outfile = $fopen("../testdata/output.txt") | 1;
    outfile2 = $fopen("../testdata/cache.txt") | 1;
    
    
    // Set Input n into data memory at 0x00
    Data_Memory.memory[0] = 256'h5;        // n = 5 for example
    
    Clk = 0;
    Reset = 0;
    Start = 0;
    
    #(`CYCLE_TIME/4) 
    Reset = 1;
    Start = 1;

    
end
  
always@(posedge Clk) begin
    if(counter == 150) begin    // store cache to memory
        $fdisplay(outfile, "Flush Cache! \n");
        for(i=0; i<32; i=i+1) begin
            tag = CPU.dcache.dcache_tag_sram.memory[i];
            index = i;
            address = {tag[21:0], index};
            Data_Memory.memory[address] = CPU.dcache.dcache_data_sram.memory[i];
        end 
    end
    if(counter > 150) begin    // stop 
        $finish;
    end
        
    // print PC 
    $fdisplay(outfile, "cycle = %0d, Start = %b\nPC = %d", counter, Start, CPU.PC.pc_o);
    // print Registers
    $fdisplay(outfile, "Registers");
    $fdisplay(outfile, "x0 = %h, x8  = %h, x16 = %h, x24 = %h", CPU.Registers.register[0], CPU.Registers.register[8] , CPU.Registers.register[16], CPU.Registers.register[24]);
    $fdisplay(outfile, "x1 = %h, x9  = %h, x17 = %h, x25 = %h", CPU.Registers.register[1], CPU.Registers.register[9] , CPU.Registers.register[17], CPU.Registers.register[25]);
    $fdisplay(outfile, "x2 = %h, x10 = %h, x18 = %h, x26 = %h", CPU.Registers.register[2], CPU.Registers.register[10], CPU.Registers.register[18], CPU.Registers.register[26]);
    $fdisplay(outfile, "x3 = %h, x11 = %h, x19 = %h, x27 = %h", CPU.Registers.register[3], CPU.Registers.register[11], CPU.Registers.register[19], CPU.Registers.register[27]);
    $fdisplay(outfile, "x4 = %h, x12 = %h, x20 = %h, x28 = %h", CPU.Registers.register[4], CPU.Registers.register[12], CPU.Registers.register[20], CPU.Registers.register[28]);
    $fdisplay(outfile, "x5 = %h, x13 = %h, x21 = %h, x29 = %h", CPU.Registers.register[5], CPU.Registers.register[13], CPU.Registers.register[21], CPU.Registers.register[29]);
    $fdisplay(outfile, "x6 = %h, x14 = %h, x22 = %h, x30 = %h", CPU.Registers.register[6], CPU.Registers.register[14], CPU.Registers.register[22], CPU.Registers.register[30]);
    $fdisplay(outfile, "x7 = %h, x15 = %h, x23 = %h, x31 = %h", CPU.Registers.register[7], CPU.Registers.register[15], CPU.Registers.register[23], CPU.Registers.register[31]);

    // print Data Memory
    $fdisplay(outfile, "Data Memory: 0x0000 = %h", Data_Memory.memory[0]);
    $fdisplay(outfile, "Data Memory: 0x0020 = %h", Data_Memory.memory[1]);
    $fdisplay(outfile, "Data Memory: 0x0040 = %h", Data_Memory.memory[2]);
    $fdisplay(outfile, "Data Memory: 0x0060 = %h", Data_Memory.memory[3]);
    $fdisplay(outfile, "Data Memory: 0x0080 = %h", Data_Memory.memory[4]);
    $fdisplay(outfile, "Data Memory: 0x00A0 = %h", Data_Memory.memory[5]);
    $fdisplay(outfile, "Data Memory: 0x00C0 = %h", Data_Memory.memory[6]);
    $fdisplay(outfile, "Data Memory: 0x00E0 = %h", Data_Memory.memory[7]);
    $fdisplay(outfile, "Data Memory: 0x0400 = %h", Data_Memory.memory[32]);
    
    $fdisplay(outfile, "\n");
    // print Data Cache Status
    if(CPU.dcache.p1_stall_o && CPU.dcache.state == 0) begin
        if(CPU.dcache.sram_dirty) begin
            if(CPU.dcache.p1_MemWrite_i) 
                $fdisplay(outfile2, "Cycle: %d, Write Miss, Address: %h, Write Data: %h (Write Back!)", counter, CPU.dcache.p1_addr_i, CPU.dcache.p1_data_i);
            else if(CPU.dcache.p1_MemRead_i) 
                $fdisplay(outfile2, "Cycle: %d, Read Miss , Address: %h, Read Data : %h (Write Back!)", counter, CPU.dcache.p1_addr_i, CPU.dcache.p1_data_o);
        end
        else begin
            if(CPU.dcache.p1_MemWrite_i) 
                $fdisplay(outfile2, "Cycle: %d, Write Miss, Address: %h, Write Data: %h", counter, CPU.dcache.p1_addr_i, CPU.dcache.p1_data_i);
            else if(CPU.dcache.p1_MemRead_i) 
                $fdisplay(outfile2, "Cycle: %d, Read Miss , Address: %h, Read Data : %h", counter, CPU.dcache.p1_addr_i, CPU.dcache.p1_data_o);
        end
        flag = 1'b1;
    end
    else if(!CPU.dcache.p1_stall_o) begin
        if(!flag) begin
            if(CPU.dcache.p1_MemWrite_i) 
                $fdisplay(outfile2, "Cycle: %d, Write Hit , Address: %h, Write Data: %h", counter, CPU.dcache.p1_addr_i, CPU.dcache.p1_data_i);
            else if(CPU.dcache.p1_MemRead_i) 
                $fdisplay(outfile2, "Cycle: %d, Read Hit  , Address: %h, Read Data : %h", counter, CPU.dcache.p1_addr_i, CPU.dcache.p1_data_o);
        end
        flag = 1'b0;
    end
        
    
    counter = counter + 1;
end

  
endmodule
