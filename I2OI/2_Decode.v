module decode 
(
	input [31:0] inst, writedata, 
	input clk, 

	output [31:0] data1, data2, ImmGen, 
	output alusrc, memread, memwrite, memtoreg, branch, 
	output [1:0] aluop, 
	output [9:0] funct
);

	wire branch, memread, memtoreg, MemWrite, alusrc, regwrite;
	wire [1:0] aluop; 
	wire [4:0] writereg, rs1, rs2, rd;
	wire [6:0] opcode;
	wire [9:0] funct;
	wire [31:0] ImmGen;

	assign opcode = inst[6:0];
	assign rs1    = inst[19:15];
	assign rs2    = inst[24:20];
	assign rd     = inst[11:7];
	assign funct = {inst[31:25],inst[14:12]};

	ControlUnit control (opcode, inst, alusrc, memtoreg, regwrite, memread, memwrite, branch, aluop, ImmGen);

	Register_Bank Registers (clk, regwrite, rs1, rs2, rd, writedata, data1, data2); 

endmodule

