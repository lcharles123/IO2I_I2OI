/** CORE **/
module pipemips 
(
	input clk, rst, stall, 
	
	output [31:0] reg_writedata
);

	wire [31:0] d_inst, d_pc, 
	i_pc, i_rd1, i_rd2, i_sig_ext,
	e_pc, e_rd1, e_rd2, e_sig_ext,
	 sig_ext, write_data, 
	m_addres, add_res, m_alures, m_readdata, 
	w_readData, w_alures, reg_writedata;
	
	wire i_regwrite, i_memtoreg, i_branch, i_memwrite, i_memread, i_regdst, i_alusrc, 
	e_regwrite, e_memtoreg, e_branch, e_memwrite, e_memread, e_regdst, e_alusrc, 
	m_regwrite, m_memtoreg, m_zero, m_memread, m_memwrite, 
	w_regwrite, w_memtoreg, m_branch;
	
	wire [1:0] i_aluop, e_aluop;
	wire [4:0] i_inst1, i_inst2, i_muxRegDst,  
	e_inst1, e_inst2, e_muxRegDst,
	m_muxRegDst, 
	w_muxRegDst;
	
	wire pc_src, stall;


	Fetch fetch (clk, rst, stall,  pc_src,  // input do fetch
					
					m_addres, d_inst, d_pc); //output do fetch


	Decode decode (clk, rst, w_regwrite, d_inst, d_pc, reg_writedata, w_muxRegDst,
	
					i_rd1, i_rd2, sig_ext, i_pc, i_inst1, i_inst2, i_aluop, i_alusrc, i_regdst, 
					i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch);

	/*Issue issue (clk, rst, 
					i_rd1, i_rd2, i_sig_ext, i_pc, i_inst1, i_inst2, i_aluop, i_alusrc, i_regdst, 
					i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch,
					
					e_rd1, e_rd2, e_sig_ext, e_pc, e_inst1, e_inst2, e_aluop, e_alusrc, e_regdst, 
					e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch
					
					);*/

	/*Execute execute (clk, rst, e_alusrc, e_regdst, e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch, i_rd1, e_rd2, e_sig_ext, e_pc, e_inst1, e_inst2, e_aluop, 
	
						m_alures, m_addres, write_data, m_muxRegDst, m_branch, m_zero, m_regwrite, m_memtoreg, m_memread, m_memwrite);
*/

	/*Memory memory (clk, rst, m_branch, m_zero, m_regwrite, m_memtoreg, m_memread, m_memwrite, m_alures, write_data, m_muxRegDst,
		
						w_readData, w_alures, w_memtoreg, w_regwrite, pc_src, w_muxRegDst);
*/

	/*Writeback writeback (w_readData, w_alures, w_memtoreg, 
	
							reg_writedata);*/

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


//OK ATE AQUI
//ate aqui estamos trazendo os sinais de instrucao e PC
//mas exige o endereco do branch e o sinal de stall


/** DECODE **/
module Decode 
(
	input clk, rst, regwrite, 
	input [31:0] inst, pc, writedata, 
	input [4:0] muxRegDst, 
	
	output [31:0] e_rd1, e_rd2, e_sigext, e_pc, 
	output [4:0] e_inst1, e_inst2, 
	output [1:0] e_aluop, 
	output e_alusrc, e_regdst, e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch
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


	PRF Registers (clk, regwrite, rA, rB, muxRegDst, writedata,  //
					
					data1, data2);
	
	//Reorder_Buffer();







	/** IDI **/
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

/** ISSUE **/ //MODULO INCOMPLETO
//module Issue 
//(


/*Issue issue (clk, rst, 
					i_rd1, i_rd2, sig_ext, i_pc,
					 i_inst1, i_inst2,  i_alusrc, i_regdst, 
					i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch,
					
					e_rd1, e_rd2, sig_ext, e_pc, e_inst1, e_inst2, e_aluop, e_alusrc, e_regdst, 
					e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch
					
					);*/

	/*input clk, rst,  
	input [31:0] i_rd1, i_rd2, i_sigext, i_pc,
	input [4:0] i_inst1, i_inst2, 
	input [1:0] i_aluop,
	input  i_alusrc, i_regdst, i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch,
	
	output [31:0] e_rd1, e_rd2, e_sigext, e_pc, 
	output [4:0] e_inst1, e_inst2, 
	output [1:0] e_aluop, 
	output e_alusrc, e_regdst, e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch
);

*/

	/** IEX **/
	/*IEX iex (clk, rst, 
					i_rd1, i_rd2, i_sigext, i_pc,
					 i_inst1, i_inst2, i_aluop, i_alusrc, i_regdst, 
					i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch,
					
					e_rd1, e_rd2, e_sigext, e_pc, e_inst1, e_inst2, e_aluop, e_alusrc, e_regdst, 
					e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch
					
					);


endmodule*/


/** IEX **/
/*module IEX 
(
	input clk, rst,  
	input [31:0] i_rd1, i_rd2, i_sigext, i_pc,
	input [4:0] i_inst1, i_inst2, 
	input [1:0] i_aluop,
	input  i_alusrc, i_regdst, i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch,
	
	output reg [31:0] e_rd1, e_rd2, e_sigext, e_pc, 
	output reg [4:0] e_inst1, e_inst2, 
	output reg [1:0] e_aluop, 
	output reg e_alusrc, e_regdst, e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch
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
			e_regwrite <= i_regwrite;
			e_memtoreg <= i_memtoreg;
			e_branch   <= i_branch;
			e_memwrite <= i_memwrite;
			e_memread  <= i_memread;
			e_regdst   <= i_regdst;
			e_aluop    <= i_aluop;
			e_alusrc   <= i_alusrc;
			e_pc       <= i_pc;
			e_rd1      <= i_rd1;
			e_rd2      <= i_rd2;
			e_sigext   <= i_sigext;
			e_inst1    <= i_inst1;
			e_inst2    <= i_inst2;
		end
	end
endmodule

*/


/** EXECUTE **/
module Execute 
(
	input clk, rst, e_alusrc, e_regdst, e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch, 
	input [31:0] e_in1, e_in2, e_sigext, e_pc, 
	input [4:0] e_inst20_16, e_inst15_11, 
	input [1:0] e_aluop, 
	
	output [31:0] m_alures, m_addres, m_rd2, 
	output [4:0] m_muxRegDst, 
	output m_branch, m_zero, m_regwrite, m_memtoreg, m_memread, m_memwrite
);

	wire [31:0] alu_B, e_addres, e_aluout;
	wire [3:0] aluctrl;
	wire [4:0] e_muxRegDst;
	wire e_zero;

	Add Add (e_pc, e_sigext,
				e_addres);

	assign alu_B = (e_alusrc) ? e_sigext : e_in2 ;

	//Unidade Lógico Aritimética
	ALU alu (aluctrl, e_in1, alu_B, 
				e_aluout, e_zero);

	Alucontrol alucontrol (e_aluop, e_sigext[5:0], 
								aluctrl);

	assign e_muxRegDst = (e_regdst) ? e_inst15_11 : e_inst20_16;

	/** EXMEM **/
	EXMEM exmem (clk, rst, e_regwrite, e_memtoreg, e_branch, e_zero, e_memread, e_memwrite, e_addres, e_aluout, alu_B, e_muxRegDst, 
					m_regwrite, m_memtoreg, m_zero, m_memread, m_memwrite, m_branch, m_addres, m_alures, m_rd2, m_muxRegDst);

endmodule



module Add 
(
	input [31:0] pc, shiftleft2, 
	
	output [31:0] add_result
);
	assign add_result = pc + (shiftleft2 << 2);
endmodule


module ALU 
(
	input [3:0] alucontrol, 
	input [31:0] A, B, 
	
	output [31:0] aluout, 
	output zero
);

	reg [31:0] aluout;

	// Zero recebe um valor lógico caso aluout seja igual a zero.
	assign zero = (aluout == 0); 

	always @(alucontrol, A, B) 
	begin
	//verifica qual o valor do controle para determinar o que fazer com a saída
		case (alucontrol)
			0: aluout <= A & B; // AND
			1: aluout <= A | B; // OR
			2: aluout <= A + B; // ADD
			6: aluout <= A - B; // SUB
			7: aluout <= A < B ? 32'd1:32'd0; //SLT
			12: aluout <= ~(A | B); // NOR
			default: aluout <= 0; //default 0, Nada acontece;
		endcase
	end
endmodule


module EXMEM 
(
	input clk, rst, e_regwrite, e_memtoreg, e_branch, e_zero, e_memread, e_memwrite, 
	input [31:0] e_addres, e_alures, e_rd2, 
	input [4:0] e_muxRegDst, 
	
	output reg m_regwrite, m_memtoreg, m_zero, m_memread, m_memwrite, m_branch, 
	output reg [31:0] m_addres, m_alures, m_rd2, 
	output reg [4:0] m_muxRegDst
);

	always @(posedge clk) 
	begin
		if (!rst) 
		begin
			m_regwrite  <= 0;
			m_memtoreg  <= 0;
			m_addres    <= 0;
			m_zero      <= 0;
			m_alures    <= 0;
			m_rd2       <= 0;
			m_muxRegDst <= 0;
			m_memread   <= 0;
			m_memwrite  <= 0;
			m_branch    <= 0;
		end
		else 
		begin
			m_regwrite  <= e_regwrite;
			m_memtoreg  <= e_memtoreg;
			m_addres    <= e_addres;
			m_zero      <= e_zero;
			m_alures    <= e_alures;
			m_rd2       <= e_rd2;
			m_muxRegDst <= e_muxRegDst;
			m_memread   <= e_memread;
			m_memwrite  <= e_memwrite;
			m_branch    <= e_branch;
		end
	end
endmodule

/** MEM **/
module Memory 
(
	input clk, rst, m_branch, m_zero, m_regwrite, m_memtoreg, m_memread, m_memwrite, 
	input [31:0] m_alures, writedata, 
	input [4:0] m_muxRegDst, 
	
	output [31:0] w_readdata, w_alures, 
	output w_memtoreg, w_regwrite, pc_src, 
	output [4:0] w_muxRegDst
);

	wire [31:0] m_readdata;
	reg [31:0] memory [0:127]; 

	//popular memoria
	initial $readmemh("tb/dados.mem", memory, 0, 31);
	
	/*
	memory[0] <= 32'd1; 
	memory[1] <= 32'd2; 
	memory[2] <= 32'd3; 
	memory[3] <= 32'd4; 
	memory[4] <= 32'd5; 
	memory[5] <= 32'd6; 
	memory[6] <= 32'd7; 
	memory[7] <= 32'd8; 
	memory[8] <= 32'd9; 
	*/
	
	assign pc_src = (m_zero & m_branch) ? 1 : 0; 

	assign m_readdata = (m_memread) ? memory[m_alures[31:2]] : 0;

	always @(posedge clk) 
	begin
		if (m_memwrite)
			memory[m_alures[31:2]] = writedata;
	end

	/** MEMWB **/
	MEMWB MEMWB (clk, rst, m_regwrite, m_memtoreg, m_readdata, m_alures, m_muxRegDst, 
					
					w_readdata, w_alures, w_muxRegDst, w_regwrite, w_memtoreg); 

endmodule


module MEMWB 
(
	input clk, rst, m_regwrite, m_memtoreg, 
	input [31:0] m_readData, m_alures, 
	input [4:0] m_muxRegDst,
	
	output reg [31:0] w_readData, w_alures, 
	output reg [4:0] w_muxRegDst, 
	output reg w_regwrite, w_memtoreg
);
	always @(posedge clk) 
	begin
		if (!rst) 
		begin
			w_readData  <= 0;
			w_alures    <= 0;
			w_regwrite  <= 0;
			w_memtoreg  <= 0;
			w_muxRegDst <= 0;
		end
		else 
		begin
			w_readData  <= m_readData;
			w_alures    <= m_alures;
			w_regwrite  <= m_regwrite;
			w_memtoreg  <= m_memtoreg;
			w_muxRegDst <= m_muxRegDst;
		end
	end
endmodule

/** WRITEBACK **/
module Writeback 
(
	input [31:0] readdata, aluout, 
	input memtoreg, 
	
	output [31:0] write_data
);

    assign write_data = (memtoreg) ? readdata : aluout;

endmodule


/** CONTROL **/
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


module Alucontrol 
(
	input [1:0] aluop, 
	input [5:0] funct, 
	
	output [3:0] alucontrol
);

	reg [3:0] alucontrol;

	always @(aluop or funct) 
	begin
		case (aluop)
			2'b00: alucontrol <= 4'b0010; // ADD para sw e lw
			2'b01: alucontrol <= 4'b0110; // SUB para branch
			default:
			begin
				case (funct)
					32: alucontrol <= 4'b0010; // ADD
					34: alucontrol <= 4'b0110; // SUB
					36: alucontrol <= 4'b0000; // AND
					37: alucontrol <= 4'b0001; // OR
					39: alucontrol <= 4'd12; // NOR ?
					42: alucontrol <= 4'b0111; // SLT
					default: alucontrol <= 4'd15; // Nada acontece ?
				endcase
			end
		endcase
	end
endmodule


/** modulos auxiliares **/
// score board eh uma tabela com 32 linhas que representa o estado dos registradores,
// os estados estao divididos por colunas e incluem 
// pendente: bit/coluna 7 indicando se alguma instrucao escrevera nele, bit [6:5] indicam qual unidade funconal escrevera no registrador dentre 00: ALU; 01:Load; 10: Store; 11: Mul
// bits [4:0] indicam em que lugar no pipeline o dado esta, esses dados podem ser usados para fazer encaminhamento

module Score_Board 
(
	input clk,
	input [5:0] regist, //registrador que sera alterado 2**5 == 32 registradores (regist == 0 eh ignorado)
	input [7:0] valor, //valor de escrita para a linha inteira, colocar o 1 no dado disponivel no estagio correto no SB
						//valor[7] == pendente; valor[6:5] == nome unidade funcional; valor[4:0] == localizacao do dado
	
	output pendente [0:31]; //valor para leitura sera toda a coluna "Pendente", basta usar os enderecos de regA e regB nessa saida
							//para termos o estado, havera apenas essa saida pois nao existira encaminhamentos por enquanto
);	
	reg [0:31] pendente;
	reg [7:0] sb [0:31]; // 32 linhas de 8 bits cada

	assign pendente <= [7]sb; //coluna de 1 bit indicando pendente vai para saida 
	assign [7]sb <= [6:5]sb ~^ [0]sb; //apenas coloc
	//initial sb = 0; //inicializar valores == 0, e valor de regist[0] == pendente colocar valor 
	
	always @(posedge clk)
	begin
		if(regist) //nao atualiza o registrador 0
		begin
			sb[regist] <= valor;
		end
		/** faz o dado andar nas colunas dado disponivel **/
		//shift 0 >> sb[4:0]
		
	end

endmodule


module Physical_Register_File // registradores auxiliares
(
	input clk, regwrite, 				//sinal clock e permissao escrita
	input [4:0] read1, read2, writereg,  //enderecos de leitura e escrita
	input [31:0] writedata, 			//dado para escrita
	
	output [31:0] data1, data2
);

	reg [31:0] memory_tmp [0:31]; // 32 bits cada um dos 32 registers 
	
	// Popular os registradores
	//initial $readmemh("tb/dados.mem", memory, 0, 31);
	integer i;
	initial for(i = 0; i < 32; i = i+1) memory_tmp[i] = 0; 

	assign data1 = (regwrite && read1==writereg) ? writedata : memory_tmp[read1];//faz dado1 ser escrito no inicio do ciclo com a possibilidade de ser lido no final
	assign data2 = (regwrite && read2==writereg) ? writedata : memory_tmp[read2]; //mesma coisa pra dado2

	always @(posedge clk) 
	begin
    	if (regwrite)
        	memory_tmp[writereg] <= writedata;//apenas escreve, caso seja possivel
  	end
  
endmodule

module Architetural_Register_File  //banco de registradores principais da maquina
(
	input clk, regwrite, 				//sinais de controle
	input [4:0] read1, read2, writereg, //enderecos de regs leitura1, leitura2 e escrita
	input [31:0] writedata, 			//dado para escrita
	
	output [31:0] data1, data2			//saida do dado1 e dado2
);

	reg [31:0] memory [0:31]; // 32 registers de 32 bits cada

	// Popular os registradores
	//initial $readmemh("tb/dados.mem", memory, 0, 31);
	integer i;
	initial for(i = 0; i < 32; i = i+1) memory_tmp[i] = 0; 

	assign data1 = (regwrite && read1==writereg) ? writedata : memory[read1]; //faz dado1 ser escrito no inicio do ciclo com a possibilidade de ser lido no final
	assign data2 = (regwrite && read2==writereg) ? writedata : memory[read2]; //mesma coisa com dado2

	always @(posedge clk) 
	begin
    	if (regwrite)
        	memory[writereg] <= writedata; //apenas escreve, caso seja possivel
  	end
  
endmodule




//instancia no decode, ler linha com Preg especifico: 
//decode ira procurar por instrucao pronta, i.e. ROB recebe endereco Preg na variavel dado_in[4:0] com lerEscrever == 00
//dado_out retornara o resultado.

//instancia no WB, escrever apenas:
// lerEscrever == 10 , dado_in == linha da instrucao para entrar na fila, testar se !cheio antes de

module Reorder_Buffer // https://esrd2014.blogspot.com/p/first-in-first-out-buffer.html
(
	// descricao do dado_in e dado_out abaixo
	// State: (Free, Pending, Finished) [9:8] 3 estados para representar
	// S: especulativo  [7]
	// ST: Store bit [6]
	// V: destino PRF valido [5]
	// Preg: end PRF [4:0]  2**5 registradores no PRF 
	input clk, rst, 
	input [1:0] lerEscrever, // 00 ler linha especificada por Preg ; 10 entra dado ; 01 sai dado ; 11 atualizar linha especifica por Preg
	input [9:0] dado_in, 

	output [9:0] dado_out,
	output cheio
);
	//wire [1:0] lerEscrever;
	reg [9:0] dado_out;
	reg [2:0] inicio, fim, contador; // 2**3 == 8, marcar o tamanho da tabela

	reg [9:0] fila [7:0]; //fila possui 10 bits por linha e 8 linhas

    assign cheio = (contador == 3'b111) ? 1'b1 : 1'b0 ; //avisa quando o rob estiver cheio

    always @(posedge clk) 
    begin 
		if(!rst)
		begin
			if (lerEscrever == 2'b10 && contador != 3'b0) // se leitura e fila nao vazia
			begin 
				dado_out = fila[inicio]; // ler inicio da fila
				inicio = inicio + 1;//fila andando
  			end 
  			else if (lerEscrever == 2'b01 && contador < 8) // se escrita e fila nao cheia
  			begin
  				fila[fim] = dado_in;
  				fim = fim + 1;
  			end 
  			else if (lerEscrever == 2'b00)
  			begin
  				;//dado_out <= fila[ fila[4:0] ]; //comparar Preg e retornar a linha correspondente
  			end
  			else if (lerEscrever == 2'b11)
  			begin
  				;//fila[9:8] <= atualizar
  			end
   			
   			if(fim == 8) //se algum contador chegou na posicao 8, volta pra 0
   				fim = 0;
   			if(inicio == 8)
   				inicio = 0;
   			
   			if(inicio > fim)
   				contador = inicio - fim;
   			else if(fim > inicio)
   				contador = fim - inicio;
		end
		else
		begin
			inicio <= 0;
			fim <= 0;
			contador <= 0;
		end	
	end
endmodule


// FSB: escrito no wb e lido e apagado apos o commit
// serve apenas para instrucoes de store e 
module Finished_Store_Buffer(

	input clk,	
	input [1:0] lerEscrever, // 10 apenas commit para memoria quando cheio, 01 apenas escreve nele quando vazio , 11 ou 00 nao muda o estado
	input [1:0] oper, //operacao que gerou o dado, utilidade?
	input [31:0] endereco_in, // endereco de escrita na memoria
	input [31:0] dado_in,	// dado para a escrita
	
	output [31:0] endereco_out, //saidas
	output [31:0] dado_out,
	output vazio //saida do bit de validade
);

	reg [31:0] endereco_out, dado_out; //saidas que serao os elementos de memoria para armazenar o valor
	reg [1:0] oper; //operacao, descobrir para que serve
	reg vazio;

	initial
	begin
		endereco <= 32'b0;
		dado <= 32'b0;
		vazio <= 1'b1; //indica vazio
	end
	
	always @(posedge clk)
	begin
		if(vazio && lerEscrever == 2'b01) // se esta vazio e um dado vai ser escrito nele
		begin
			endereco_out <= endereco_in;
			dado_out <= dado_in;
			vazio <= 1'b0; //muda para cheio
		end
		else if(!vazio && lerEscrever == 2'b10) // se esta cheio e um dado sera escrito na memoria
		begin
			vazio <= 1'b1; //apenas muda bit de validade para vazio
		end
			// 00 ou 11 e vazio == 0, o dado eh valido, entao a saida ja estara ativa
	end
endmodule


module Issue_Queue(

	//ver na net
);

























endmodule
