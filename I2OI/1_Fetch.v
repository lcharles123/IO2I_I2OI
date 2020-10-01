module Fetch 
(	
	input zero, rst, clk, branch, 
	input [31:0] sigext, 
	
	output [31:0] inst
);

	wire [31:0] PC, PC_4, newPC;

	assign PC_4 = PC + 4; // pc+4  Adder
	assign newPC = (branch & zero) ? PC_4 + sigext : PC_4; // desvia se for um branch beq for true

	PC program_counter(newPC, clk, rst, PC);

	reg [31:0] inst_mem [0:31];

	assign inst = inst_mem[PC[31:2]];

	initial
	begin
		// Exemplos
		inst_mem[0] <= 32'h00000000; // nop
		inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok
		inst_mem[2] <= 32'h00210233; // add  x4, x2, x2  ok
		//inst_mem[1] <= 32'h00202223; // sw x2, 8(x0) ok
		//inst_mem[1] <= 32'h0050a423; // sw x5, 8(x1) ok
		//inst_mem[2] <= 32'h0000a003; // lw x1, x0(0) ok
		//inst_mem[1] <= 32'hfff00113; // addi x2,x0,-1 ok
		//inst_mem[2] <= 32'h00318133; // add x2, x3, x3 ok
		//inst_mem[3] <= 32'h40328133; // sub x2, x5, x3 ok
	end

endmodule



module PC 
(
	input [31:0] pc_in, 
	input clk, rst, 
	
	output reg [31:0] pc_out
);

	always @(posedge clk) 
	begin
		pc_out <= pc_in;
		if (~rst)
			pc_out <= 0;
	end

endmodule



