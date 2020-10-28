/** CORE **/
module pipemips 
(
	input clk, rst, stall,
  	output alusrc
  	
);
  	//Variaveis Fetch
	wire [31:0] d_inst, d_pc;	
	wire pc_src, stall, m_addres;
  
  	//Variaveis Decode
  	wire clk, rst, regwrite;
    wire [31:0] inst, pc, writedata;
    wire [4:0] muxRegDst;
  	wire alusrc;

	Fetch fetch (clk, rst, stall,  pc_src, m_addres, // input do fetch
                 
					 d_inst, d_pc); //output do fetch
  
  
  	
  Decode decode (clk, rst, regwrite, d_inst, d_pc, writedata, muxRegDst,
                 
                 alusrc); //output do fetch
	
					//i_rd1, i_rd2, sig_ext, i_pc, i_inst1, i_inst2, i_aluop, i_alusrc, i_regdst, 
					//i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch);
endmodule

/** FETCH **/
module Fetch // se stall == 1, insere nop
(
	input clk,rst, stall,  pc_src, 
	input [31:0] add_res, 
	
	output [31:0] d_inst, d_pc
);
  
	wire [31:0] pc, new_pc, pc_4;
	wire [31:0] inst;

	assign pc_4 = (stall) ? pc : pc + 4;
	assign new_pc = (pc_src) ? add_res : pc_4; //decide se pc vem do fluxo normal ou de endereco de branch

	PC program_counter(new_pc, clk, rst, 
						pc);

	reg [31:0] inst_mem [0:31];

	assign inst = (stall) ? 32'b0 : inst_mem[pc[31:2]]; // insere nop caso stall
	
	/** IFID **/
	IFID IFID (clk, rst, pc_4, inst, 
					d_pc, d_inst);

	initial 
	begin

		//$readmemb("tb/inst.mem",inst_mem, 0, 31); //carrega de arquivo
			
		inst_mem[0] <= 32'b000000_00000_00000_00000_00000_000000; // nop 
		inst_mem[1] <= 32'b001000_00000_01010_0000000000000101; // addi $t2,$zero,5
		inst_mem[2] <= 32'b001000_00000_01011_0000000000000111; // addi $t3,$zero,7
		inst_mem[3] <= 32'b001000_00000_01100_0000000000000010; // addi $t4,$zero,2
		inst_mem[4] <= 32'b001000_00000_01101_0000000000000000; // addi $t5,$zero,0
		inst_mem[5] <= 32'b000000_01010_01011_01010_00000_100000; // add $t2,$t2,$t3
		inst_mem[6] <= 32'b000000_01011_01100_01011_00000_100000; // add $t3,$t3,$t4
		inst_mem[7] <= 32'b000000_01100_01100_01100_00000_100000; // add $t4,$t4,$t4
		inst_mem[8] <= 32'b000000_01101_01010_01101_00000_100000; // add $t5,$t5,$t2
		inst_mem[9] <= 32'b000000_01101_01010_01101_00000_100000; // add $t5,$t5,$t2 rep
		inst_mem[10] <= 32'b000000_01101_01010_01101_00000_100000; // add $t5,$t5,$t2 rep
		inst_mem[11] <= 32'b000000_01101_01010_01101_00000_100000; // add $t5,$t5,$t2 rep
		inst_mem[12] <= 32'b000000_01101_01010_01101_00000_100000; // add $t5,$t5,$t2 rep
		inst_mem[13] <= 32'b000000_01101_01010_01101_00000_100000; // add $t5,$t5,$t2 rep
		inst_mem[14] <= 32'b000000_01101_01010_01101_00000_100000; // add $t5,$t5,$t2 rep
		inst_mem[15] <= 32'b000000_01101_01010_01101_00000_100000; // add $t5,$t5,$t2 rep
		inst_mem[16] <= 32'b000000_01101_01010_01101_00000_100000; // add $t5,$t5,$t2 rep
		inst_mem[17] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[18] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[19] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[20] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[21] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[22] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[23] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[24] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[25] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[26] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[27] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[28] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[29] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[30] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep
		inst_mem[31] <= 32'b00000001101010100110100000100000; // add $t5,$t5,$t2 rep

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
		if (!rst)
			pc_out <= 0;
	end
	
endmodule


module IFID 
(
	input clk, f_rst, 
	input [31:0] f_pc, f_inst, 
	
	output reg [31:0] d_pc, d_inst
);

	always @(posedge clk) 
	begin
		if (!f_rst) 
		begin
			d_inst <= 0;
			d_pc   <= 0;
		end
		else 
		begin
			d_inst <= f_inst;
			d_pc   <= f_pc;
		end
	end
endmodule


// DECODE
module Decode 
(
	input clk, rst, 
    input regwrite, 
  	input [31:0] inst, pc, writedata, 
	input [4:0] muxRegDst, 
  
  	output alusrc
);
  
	wire [31:0] data1, data2, sig_ext; 
	wire [4:0] rA, rB, rC; 
	wire [5:0] opcode;
	wire [1:0] aluop;
	wire branch, memread, memtoreg, memwrite, regdst, alusrc, regwrite, regwrite_out;

	assign opcode = inst[31:26]; //opcode
	assign rA = inst[25:21];    //rs 
	assign rB = inst[20:16];    //rt 
	assign rC = inst[15:11];    //rd destino, caso seja tipo R

	assign sig_ext = (inst[15]) ? {16'd1,inst[15:0]} : {16'd0,inst[15:0]}; // extensor de sinal coloca 1 se negativo, 0 caso contrario

	Control control (opcode, //entrada e saidas dos sinais de controle, saidas serao ligadas no modulo superior
					
					regdst, alusrc, memtoreg, regwrite_out, memread, memwrite, branch, aluop);


	Register_Bank Registers (clk, regwrite, rA, rB, muxRegDst, writedata, 
			                   data1, data2);
	
	//Reorder_Buffer();

	// IDI
	IDI idi (clk, rst, regwrite_out, memtoreg, branch, memwrite, memread, regdst, alusrc, aluop, pc, data1, data2, sig_ext, rt, rC,
			 e_regwrite, e_memtoreg, e_branch, e_memwrite, e_memread, e_regdst, e_alusrc, e_aluop, e_pc, e_rd1, e_rd2, e_sigext, e_inst1, e_inst2);


endmodule

module IDI 
(
	input clk, rst, d_regwrite, d_memtoreg, d_branch, d_memwrite, d_memread, d_regdst, d_alusrc, 
	input [1:0] d_aluop, 
	input [31:0] d_pc, d_rd1, d_rd2, d_sigext, 
	input [4:0] d_inst1, d_inst2, 
	
	output reg e_regwrite, e_memtoreg, e_branch, e_memwrite, e_memread, e_regdst, e_alusrc, 
	output reg [1:0] e_aluop, 
	output reg [31:0] e_pc, e_rd1, e_rd2, e_sigext, 
	output reg [4:0] e_inst1, e_inst2
);

	always @(posedge clk) 
	begin
		if (!rst) 
		begin
			e_regwrite <= 0;
			e_memtoreg <= 0;
			e_branch   <= 0;
			e_memwrite <= 0;
			e_memread  <= 0;
			e_regdst   <= 0;
			e_aluop    <= 0;
			e_alusrc   <= 0;
			e_pc       <= 0;
			e_rd1      <= 0;
			e_rd2      <= 0;
			e_sigext   <= 0;
			e_inst1    <= 0;
			e_inst2    <= 0;
		end
		else 
		begin
			e_regwrite <= d_regwrite;
			e_memtoreg <= d_memtoreg;
			e_branch   <= d_branch;
			e_memwrite <= d_memwrite;
			e_memread  <= d_memread;
			e_regdst   <= d_regdst;
			e_aluop    <= d_aluop;
			e_alusrc   <= d_alusrc;
			e_pc       <= d_pc;
			e_rd1      <= d_rd1;
			e_rd2      <= d_rd2;
			e_sigext   <= d_sigext;
			e_inst1    <= d_inst1;
			e_inst2    <= d_inst2;
		end
	end
endmodule

// CONTROL
module Control 
(
	input [5:0] opcode, 
	
	output reg regdst, alusrc, memtoreg, regwrite_out, memread, memwrite, branch, 
	output reg [1:0] aluop
);

	always @(opcode) 
	begin
		case(opcode) 
			6'b000000: 
			begin // R type
				regdst <= 1 ;
				alusrc <= 0 ;
				memtoreg <= 0 ;
				regwrite_out <= 1 ;
				memread <= 0 ;
				memwrite <= 0 ;
				branch <= 0 ;
				aluop <= 2 ;
			end
			6'b000100: 
			begin // beq
				regdst <= 0 ;
				alusrc <= 0 ;
				memtoreg <= 0 ;
				regwrite_out <= 0 ;
				memread <= 0 ;
				memwrite <= 0 ;
				branch <= 1 ;
				aluop <= 1 ;
			end
			6'b001000: 
			begin // addi
				regdst <= 0 ;
				alusrc <= 1 ;
				memtoreg <= 0 ;
				regwrite_out <= 1 ;
				memread <= 0 ;
				memwrite <= 0 ;
				branch <= 0 ;
				aluop <= 0 ;
			end
			6'b100011: 
			begin // lw
				regdst <= 0 ;
				alusrc <= 1 ;
				memtoreg <= 1 ;
				regwrite_out <= 1 ;
				memread <= 1 ;
				memwrite <= 0 ;
				branch <= 0 ;
				aluop <= 0 ;
			end
			6'b101011: 
			begin // sw
				regdst <= 0 ;
				alusrc <= 1 ;
				memtoreg <= 0 ;
				regwrite_out <= 0 ;
				memread <= 0 ;
				memwrite <= 1 ;
				branch <= 0 ;
				aluop <= 0 ;
			end
			default: 
			begin //nop
				regdst <= 0 ;
				alusrc <= 0 ;
				memtoreg <= 0 ;
				regwrite_out <= 0 ;
				memread <= 0 ;
				memwrite <= 0 ;
				branch <= 0 ;
				aluop <= 0 ;
			end
		endcase
	end

endmodule 

module Register_Bank (input clk, regwrite, input [4:0] read1, read2, writereg, input [31:0] writedata, output [31:0] data1, data2);

  integer i;
  reg [31:0] memory [0:31]; // 32 registers de 32 bits cada

  // Inicializa a memória
  initial
    for (i = 0; i <= 31; i++) 
      memory[i] <= i;
 
  assign data1 = (regwrite && read1==writereg) ? writedata : memory[read1];
  assign data2 = (regwrite && read2==writereg) ? writedata : memory[read2];
	
  always @(posedge clk)
    if (regwrite)
      memory[writereg] <= writedata;
  
endmodule


