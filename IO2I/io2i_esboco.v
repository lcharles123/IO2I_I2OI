/** CORE **/
module pipemips 
(
	input clk, rst,
	
	//resultado das operacoes que escrevem no reg: todas menos branch e alu

  
  	output [31:0] x_aluout, 
    output [4:0]  x_register_store_adress,  
   // output [4:0]  L0_register_store_adress, L1_register_store_adress,
   // output [4:0]  S0_register_store_adress,
  	//output [31:0]  L0_load_aluout, L1_load_aluout,
  //  output [31:0]  S0_store_value,
  
 //	output [31:0]  mul3_res, output [4:0] mul3_endRegC,
 	
 	output [31:0] dado_regC, w_novoPC,	
    output [4:0] w_regC,
 	output permitEscrita , w_branchTomado//testar branch
);  
  	wire stall;
  	//Variaveis Fetch
  	wire [31:0] d_inst, d_pc, w_novoPC;	
  
  	//Variaveis Decode
  	wire permitEscrita;
    wire [31:0] inst, pc; //dado para escrita no reg[w_regC]
 	wire [4:0] w_regC;//endereco para escrita nos registradores, regC de inst tipo R e destino do load
  
    //Variaveis Entrada Issue
  	wire [31:0] i_rd1, i_rd2, i_sigext, i_pc;
  	wire [4:0] i_inst1, i_inst2, i_inst3;
  	wire [1:0] i_aluop;
	wire  i_alusrc, i_regdst, i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch;
  
  	//Variaveis Saída Issue
    wire [31:0] e_rd1, e_rd2, e_sigext, e_pc;
    wire [4:0] e_inst1, e_inst2, e_inst3;
  	wire [1:0] e_aluop;
  	wire [2:0] e_select_execute;
    wire e_alusrc, e_regdst, e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch;
  
  	//Variaveis X
    wire [31:0] x_aluout;
  	wire [4:0]  x_register_store_adress;
  	wire x_pipe_ativo, w_regwrite;
 
  	//Variaveis L
    wire[31:0] L0_load_aluout, L1_load_aluout;
  	wire[4:0]  L0_register_store_adress, L1_register_store_adress;
  	wire L0_pipe_ativo, L1_pipe_ativo;
  	
  	//Variáveis S
    wire[31:0] S0_store_value;
    wire[4:0]  S0_register_store_adress;
  	wire S0_pipe_ativo;

  	//Variaveis MUL
    wire[31:0] mul0_res ,	 mul1_res, 	   mul2_res ,	 mul3_res    ;
  	wire[4:0]  mul0_endRegC, mul1_endRegC, mul2_endRegC, mul3_endRegC;
  	wire mul0_pipe_ativo, mul1_pipe_ativo, mul2_pipe_ativo, mul3_pipe_ativo;
  	
  	//variaveis writeback
	wire [31:0] dado_regC, w_novoPC;
	//wire [4:0] endRegC; == w_regC
	wire permitEscrita;

	Fetch fetch (clk, rst, stall,  w_branchTomado, w_novoPC, // input do fetch
            
					 d_inst, d_pc); //output do fetch
  
  
  	
  Decode decode (clk, rst, stall, permitEscrita, d_inst, d_pc, dado_regC, w_regC, //saida das unidades funcionais serao ligadas em writedata e w_regC
                 
                   i_rd1, i_rd2, i_sigext, i_pc, i_inst1, i_inst2, i_inst3, i_aluop, i_alusrc, i_regdst, 
					i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch);
  
  Issue issue (clk, rst, i_rd1, i_rd2, i_sigext, i_pc, i_inst1, i_inst2, i_inst3, i_aluop, i_alusrc, i_regdst, 
				i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch,
																//colocar e inst3 na entrada das unidades funcionais, end regC
               e_rd1, e_rd2, e_sigext, e_pc, e_inst1, e_inst2, e_inst3, e_aluop,  
               e_alusrc, e_regdst, e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch, e_select_execute, stall);
					
  X0 x0 (clk, rst, e_alusrc, e_regdst, e_regwrite, e_branch , e_rd1, e_rd2, e_sigext, e_pc, e_inst2, e_inst3, e_aluop, e_select_execute,

        	x_aluout, w_novoPC, x_register_store_adress, w_branchTomado, w_regwrite, x_pipe_ativo);
					
  L0 l0 (clk, rst, e_inst2, e_rd1, e_sigext, e_select_execute,

           L0_load_aluout, L0_register_store_adress, L0_pipe_ativo);
  
  L1 l1 (clk, rst, L0_load_aluout, L0_register_store_adress, L0_pipe_ativo,

           L1_load_aluout, L1_register_store_adress, L1_pipe_ativo);
           
  
  S0 s0 (clk, rst, e_rd1, e_rd2, e_sigext, e_select_execute,

           S0_store_value, S0_register_store_adress, S0_pipe_ativo);
  
  									
  M0 m0(clk, rst,  e_rd1, e_rd2, e_inst3, e_select_execute,
  			mul0_res, mul0_endRegC, mul0_pipe_ativo);
  M1 m1(clk, rst, mul0_res, mul0_endRegC, mul0_pipe_ativo,
  			mul1_res, mul1_endRegC, mul1_pipe_ativo);
  M2 m2(clk, rst, mul1_res, mul1_endRegC, mul1_pipe_ativo,
  			mul2_res, mul2_endRegC, mul2_pipe_ativo);
  M3 m3(clk, rst, mul2_res, mul2_endRegC, mul2_pipe_ativo,
  			mul3_res, mul3_endRegC, mul3_pipe_ativo);

  WB wb(clk, rst, e_select_execute, x_aluout, x_register_store_adress, x_pipe_ativo, w_regwrite , L1_load_aluout, L1_register_store_adress, L1_pipe_ativo, S0_store_value, S0_register_store_adress, S0_pipe_ativo, mul3_res, mul3_endRegC, mul3_pipe_ativo, 
	
	dado_regC,	w_regC, permitEscrita); //permitEscrita == 0 caso store e branch

  										  
			
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
	//assign new_pc = pc_4;
  
	PC program_counter(new_pc, clk, rst, 
						pc);

	reg [31:0] inst_mem [31:0];
	assign inst = (stall) ? 32'b0 : inst_mem[pc[31:2]]; // insere nop caso stall
	
	/** IFID **/
  IFID IFID (clk, rst, stall, pc_4, inst, 
					d_pc, d_inst);

	initial 
	begin
      //$readmemb("tb/inst.mem",inst_mem, 0, 31); //carrega de arquivo
      /*-------ADD
      	inst_mem[0] <= 32'b000000_00000_00000_00000_00000_000000; // nop 
		inst_mem[1] <= 32'b000101_00000_01010_0000000000000101; // addi $t2,$zero,5
		inst_mem[2] <= 32'b000101_00000_01011_0000000000000111; // addi $t3,$zero,7
		inst_mem[3] <= 32'b000101_00000_01100_0000000000000010; // addi $t4,$zero,2
		inst_mem[4] <= 32'b000101_00000_01101_0000000000000000; // addi $t5,$zero,0
		inst_mem[5] <= 32'b000001_01010_01011_01010_00000_000001; // add $t2,$t2,$t3
        inst_mem[6] <= 32'b000001_01011_01100_01011_00000_000001; // add $t3,$t3,$t4
        inst_mem[7] <= 32'b000001_01100_01100_01100_00000_000001; // add $t4,$t4,$t4

      	inst_mem[0] <= 32'b00000000000000000000000000000000; // nop pc==0
		inst_mem[1] <= 32'b000101_00010_00011_0000000000000011; // addi r2  3 -> r3  pc==4 ...
		inst_mem[2] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[3] <= 32'b000101_00011_00011_0000000000000001; // addi r3  1 -> r3 
		inst_mem[4] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[5] <= 32'b000001_00011_00011_00100_00000000001; // add r3  r3 -> r4 
		inst_mem[6] <= 32'b000110_00010_00101_0000000000000001; // subi r2 1 -> r5 
		inst_mem[7] <= 32'b00000000000000000000000000000000; // nop		
		inst_mem[8] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[9] <= 32'b000001_00101_00011_00110_00000000010; // sub r5 r3 -> r6	 
		inst_mem[10] <= 32'b100000_00000_00000_0000000000000000; // beq r0 r0 -> inst 0
		
		
      // ----- teste load e store
     *//*
		inst_mem[0] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[1] <= 32'b001001_00000_00010_0000000000000010; // lw 2(r0) -> r2 
		//inst_mem[2] <= 32'b00000000000000000000000000000000; // nop
		//inst_mem[3] <= 32'b00000000000000000000000000000000; // nop
		//inst_mem[4] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[2] <= 32'b001001_00000_00011_0000000000000011; // lw 3(r0) -> r3
		//inst_mem[6] <= 32'b00000000000000000000000000000000; // nop
		//inst_mem[7] <= 32'b00000000000000000000000000000000; // nop
		//inst_mem[8] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[3] <= 32'b000001_00010_00011_00100_00000000001; // add r2 r3 -> r4 
		//inst_mem[10] <= 32'b00000000000000000000000000000000; // nop
		//inst_mem[11] <= 32'b00000000000000000000000000000000; // nop
		//inst_mem[12] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[17] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[18] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[19] <= 32'b001000_00000_00100_0000000000000010; // sw 2(r0) <- r4
		inst_mem[20] <= 32'b00000000000000000000000000000000; // nop
		*/
		
		inst_mem[0] <= 32'b000000_00000_00000_00000_00000000000;// nop 	op	dest resposta
		inst_mem[1] <= 32'b000001_01010_01010_01110_00000001000; //mul 12*12 r14  res=144   //nada no 14, precisa dos nops ou
		        
		inst_mem[2] <= 32'b000001_01010_01010_01110_00000001000; //mul 12*12 r14  res=144   //nada no 14, precisa dos nops ou
		                
		inst_mem[3] <= 32'b000001_01010_01010_01110_00000001000; //mul 12*12 r14  res=144   //nada no 14, precisa dos nops ou
		                        
		inst_mem[4] <= 32'b000001_01010_01010_01110_00000001000; //mul 12*12 r14  res=144   //nada no 14, precisa dos nops ou
		                                
		                                
		inst_mem[5] <= 32'b000101_00000_01101_0000000000000000; // addi 0+0 r13  res=0
		
		inst_mem[6] <= 32'b001001_00000_10000_0000000000000010; // lw mem[2]=3  -> r16		
		inst_mem[7] <= 32'b001000_00000_01100_0000000000000111; // sw r12=5 ->  mem[7]                    
		inst_mem[8] <= 32'b00000000000000000000000000000000; // nop		                                
		inst_mem[9] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[10] <= 32'b00000000000000000000000000000000; // nop		
		
		
			/*	                                
		inst_mem[1] <= 32'b000101_00000_01010_0000000000000101; // addi 0+5 r10  res=5
		inst_mem[2] <= 32'b001001_00000_10000_0000000000000010; // lw mem[2]=3  -> r16		
		inst_mem[3] <= 32'b000101_00000_01100_0000000000000010; // addi 0+2 r12  res=2
		inst_mem[4] <= 32'b000101_00000_01101_0000000000000000; // addi 0+0 r13  res=0
		
		inst_mem[5] <= 32'b001001_00000_10000_0000000000000010; // lw mem[2]=3  -> r16		
		inst_mem[6] <= 32'b001000_00000_01100_0000000000000111; // sw r12=5 ->  mem[7]
		inst_mem[7] <= 32'b000101_00000_01101_0000000000000000; // addi 0+0 r13  res=0
		
        inst_mem[8] <= 32'b000001_01010_01010_01110_00000001000; //mul 12*12 r14  res=144   //nada no 14, precisa dos nops ou
        
		inst_mem[9] <= 32'b010000_00000_00000_1000000000000110; // beq 0=0 inst_mem[1]? inst 6, pc == 24
		
		inst_mem[10] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[11] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[12] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[13] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[14] <= 32'b00000000000000000000000000000000; // nop
		*/
		/*
		inst_mem[5] <= 32'b000001_01010_01011_01010_00000000001;// add 5+7 r10  res=12 [16 no 10] ?
        inst_mem[6] <= 32'b000001_01011_01100_01011_00000000001;// add 7+12 r11  res=19 
        inst_mem[7] <= 32'b000001_01100_01100_01100_00000000001;// add 2+2 r12  res=4
		inst_mem[8] <= 32'b00000000000000000000000000000000; // nop
		inst_mem[9] <= 32'b000001_01011_01100_01100_00000000010; // sub 9-4 r12  res=5	ok ate aqui
        inst_mem[10] <= 32'b000001_01010_01011_01101_00000000100; //and 12&9 r13  res=8 //v  // [16 no 13]
        inst_mem[11] <= 32'b000001_01010_01010_01110_00000001000; //mul 12*12 r14  res=144   //nada no 14, precisa dos nops ou funciona com scoreboard
        

        
        /*
		inst_mem[12] <= 32'b000001_01100_01010_01111_00000010000; //slt 5<12 r15 res=1 //v // ok ate aqui
		
		inst_mem[13] <= 32'b110000_01110_01111_0000000000000001; // blt 4<1   nao tomado //esta escrevendo no reg, arrumar isso
		
		inst_mem[14] <= 32'b001001_00000_10000_0000000000000010; // lw mem[2]=3  -> r16		
		inst_mem[15] <= 32'b001000_00000_01100_0000000000000111; // sw r12=5 ->  mem[7]
		
		inst_mem[16] <= 32'b010000_00000_00000_0000000000000001; // beq 0=0 inst_mem[1]? 
		
*/
		
//testar mull
	/*	inst_mem[0] <= 32'b00000000000000000000000000000000; // nop		r2=3 ; r3=4
		inst_mem[1] <= 32'b000001_00010_00011_00100_00000000001; // mul r2 r3 -> r4 
		inst_mem[2] <= 32'b000001_00010_00011_00100_00000000001; // mul r2 r3 -> r4 
		inst_mem[3] <= 32'b000001_00010_00011_00100_00000000001; // mul r2 r3 -> r4 
		inst_mem[4] <= 32'b000001_00010_00011_00100_00000000001; // mul r2 r3 -> r4 
		inst_mem[5] <= 32'b000001_00010_00011_00100_00000000001; // mul r2 r3 -> r4 
		inst_mem[6] <= 32'b000001_00010_00011_00100_00000000001; // mul r2 r3 -> r4 
	
		//*/  
        
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
	input clk, f_rst, stall,
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
      else if(!stall)
		begin
			d_inst <= f_inst;
			d_pc   <= f_pc;
		end
	end
endmodule


// DECODE
module Decode 
(
	input clk, rst, stall, //ok aqui, mas nao funciona no IDI
    input permitEscrita, //
  	input [31:0] inst, pc, writedata, 
    input [4:0] w_regC, //endereco para escrita nos registradores, regC de inst tipo R e destino do load
  
  	output [31:0] e_rd1, e_rd2, e_sigext, e_pc, //saida para issue 
	output [4:0] e_inst1, e_inst2, e_inst3, 
	output [1:0] e_aluop, 
	output e_alusrc, e_regdst, e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch
);
  
	wire [31:0] data1, data2, d_sigext; //extensor de sinal desse modulo
	wire [4:0] rA, rB, rC; 
	wire [5:0] opcode;
	wire [1:0] aluop;
	wire branch, memread, memtoreg, memwrite, regdst, alusrc, permitEscrita, regwrite_out;
	
	
	assign opcode = inst[31:26]; //opcode
	assign rA = inst[25:21];    //rs 
	assign rB = inst[20:16];    //rt 
	assign rC = inst[15:11];    //rd destino, caso seja tipo R

	assign d_sigext = (!rst) ? 32'b0 : {16'b0,inst[15:0]} ; // extensor de sinal extende com zeros e inicializa com 0

	Control control (opcode, //entrada e saidas dos sinais de controle, saidas serao ligadas no modulo superior
					
					regdst, alusrc, memtoreg, regwrite_out, memread, memwrite, branch, aluop);


	Register_Bank Registers (clk, permitEscrita, rA, rB, w_regC, writedata, 
			                   data1, data2);
 
	
//ISSUE QUEUE VAI AQUI

	// IDI
  IDI idi (clk, rst,stall, regwrite_out, memtoreg, branch, memwrite, memread, regdst, alusrc, aluop, pc, data1, data2, d_sigext, rA, rB, rC,
			 e_regwrite, e_memtoreg, e_branch, e_memwrite, e_memread, e_regdst, e_alusrc, e_aluop, e_pc, e_rd1, e_rd2, e_sigext, e_inst1, e_inst2, e_inst3);


endmodule

module IDI 
(
	input clk, rst,stall, d_regwrite, d_memtoreg, d_branch, d_memwrite, d_memread, d_regdst, d_alusrc, 
	input [1:0] d_aluop, 
	input [31:0] d_pc, d_rd1, d_rd2, d_sigext, 
	input [4:0] d_inst1, d_inst2, d_inst3,
	
	output reg e_regwrite, e_memtoreg, e_branch, e_memwrite, e_memread, e_regdst, e_alusrc, 
	output reg [1:0] e_aluop, 
	output reg [31:0] e_pc, e_rd1, e_rd2, e_sigext, 
	output reg [4:0] e_inst1, e_inst2, e_inst3
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
			e_inst3    <= 0;
		end
      else if(!stall)
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
			e_inst3    <= d_inst3;
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
			6'b000001: 
			begin // Tipo R
				regdst <= 1 ;//reg de destino, apenas tipo r usa regC
				alusrc <= 0 ; //se tem imediato, alusrc = 1 , uso no X0
				memtoreg <= 0 ; //caso load
				regwrite_out <= 1 ; //indica inst que escreve no reg; todas exceto branch, store, nop
				memread <= 0 ; //le da memoria, apenas load
				memwrite <= 0 ;//escreve mem, apenas store
				branch <= 0 ; //indica se eh desvio, apenas branch
				aluop <= 2'b10 ; //indica tipo R
			end
			6'b010000: 
			begin // Branch BEQ
				regdst <= 0 ; 
				alusrc <= 0 ;//o branch consiste em comparar o conteudo de regA e regB se sao iguais, saltar para a posicao dada pelo imediato
				memtoreg <= 0 ;
				regwrite_out <= 0 ;
				memread <= 0 ;
				memwrite <= 0 ;
				branch <= 1 ; //esses
				aluop <= 2'b00 ;// dois dizem se BEQ ou BLT , apenas 00 ou 01 ; 10 para tipo R e 11 para timpo IMM
			end
			6'b110000: 
			begin // Branch BLT
				regdst <= 0 ;
				alusrc <= 0 ;
				memtoreg <= 0 ;
				regwrite_out <= 0 ;
				memread <= 0 ;
				memwrite <= 0 ;
				branch <= 1 ;
				aluop <= 2'b01 ;
			end
			6'b000101: 
			begin // IMM
				regdst <= 0 ;
				alusrc <= 1 ;
				memtoreg <= 0 ;
				regwrite_out <= 1 ;
				memread <= 0 ;
				memwrite <= 0 ;
				branch <= 0 ;
				aluop <= 2'b11 ;
			end
			6'b001001: 
			begin // lw
				regdst <= 0 ;
				alusrc <= 1 ;
				memtoreg <= 1 ;
				regwrite_out <= 1 ;
				memread <= 1 ;
				memwrite <= 0 ;
				branch <= 0 ;
				aluop <= 2'b11 ;
			end
			6'b001000: 
			begin // sw
				regdst <= 0 ;
				alusrc <= 1 ;
				memtoreg <= 0 ;
				regwrite_out <= 0 ;
				memread <= 0 ;
				memwrite <= 1 ;
				branch <= 0 ;
				aluop <= 2'b11 ;
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
				aluop <= 2'b00 ;
			end
		endcase
	end


endmodule 


module Register_Bank (input clk, permitEscrita, input [4:0] read1, read2, writereg, input [31:0] writedata, output [31:0] data1, data2);

  integer i;
  reg [31:0] memory [0:31]; // 32 registers de 32 bits cada

  // Inicializa a memória
  initial begin
    for (i = 0; i <= 31; i=i+1) 
      memory[i] <= i;
  end
    
  assign data1 = (permitEscrita && read1==writereg) ? writedata : memory[read1];
  assign data2 = (permitEscrita && read2==writereg) ? writedata : memory[read2];
	
  always @(posedge clk) begin
    if (permitEscrita)
      memory[writereg] <= writedata;
  end
endmodule


module Issue 
  (
	input clk, rst, 
	input [31:0] i_rd1, i_rd2, i_sigext, i_pc,
	input [4:0] i_inst1, i_inst2, i_inst3,
	input [1:0] i_aluop,
	input  i_alusrc, i_regdst, i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch,
	
	output [31:0] e_rd1, e_rd2, e_sigext, e_pc, 
	output [4:0] e_inst1, e_inst2, e_inst3,
    output [1:0] e_aluop, 
	output e_alusrc, e_regdst, e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch,
    output [2:0] e_select_execute,
    output stall
);  

  wire [2:0] select_execute;
  SelectExPipe select_ex_pipe(clk, i_regwrite, i_branch, i_memread, i_memwrite, i_alusrc, i_sigext[4:0], 
                   select_execute); 
    //scoreboard
	Score_Board sb(clk, rst, select_execute,
                 stall);
	
        
  IEX iex (clk, rst, stall,
           i_rd1, i_rd2, i_sigext, i_pc,
           i_inst1, i_inst2, i_inst3,
           i_aluop,
           i_alusrc, i_regdst, i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch, select_execute,

           e_rd1, e_rd2, e_sigext, e_pc, e_inst1, e_inst2,e_inst3, e_aluop, e_alusrc, e_regdst, 
           e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch, e_select_execute
			);
  
endmodule


module SelectExPipe (
  input clk, i_regwrite, i_branch, i_memread, i_memwrite, i_alusrc,
  input [4:0] i_sigext, //Fazendo o papel de Funct, para verificarmos se eh multiplicacao
  output [2:0] select_execute
);
  always @(*) 
    begin
      
                    
      if( (i_regwrite && i_sigext[4:0] != 5'b01000 && !i_memread) || i_branch || (i_regwrite && i_alusrc && !i_memread)) //Envia para o X0, ou seja instrucao Soma/Branch
          select_execute <= 3'b001;
      
     // if( (i_regwrite && i_sigext[4:0] != 5'b01000) || i_branch || (i_regwrite && i_alusrc)) //Envia para o X0, ou seja instrucao Soma/Branch
      //    select_execute <= 3'b001;
      else if (i_memread) // Envia para o L0, instrucao Load
          select_execute <= 3'b010;
      else if(i_memwrite)// Envia para o S0, instrucao Store
         select_execute <= 3'b011;
      else if(i_regwrite && i_sigext == 5'b01000 && !i_alusrc && !i_memread)// Envia para o M0, instrucao MUL
         select_execute <= 3'b100;
      else //Stall
         select_execute <= 3'b000;
      
    end
  
endmodule



module Score_Board 
(
	input clk, rst,
    input [2:0] select_execute,
    output reg stall
);
  
  	reg [3:0] disponibilidade;

  always @(negedge clk  )
	begin  
      if(rst) begin
        disponibilidade = disponibilidade >> 1;
        
        case(select_execute) 
              3'b001 : begin //X0
                if(disponibilidade[0] == 1) begin
            		stall = 1;
                  end else begin
                    disponibilidade = disponibilidade + 1;
                    stall = 0;
                  end
              end
                  
          	  3'b010 : begin //LW
                if(disponibilidade[1] == 1) begin
            		stall = 1;
                  end else begin
                    disponibilidade = disponibilidade + 2;
                    stall = 0;
                  end
              end
          	  3'b011 : begin //SW
                if(disponibilidade[0] == 1) begin
            		stall = 1;
                  end else begin
                    disponibilidade = disponibilidade + 1;
                    stall = 0;
                  end
              end
          		//MUL
          	  3'b100 : disponibilidade = disponibilidade + 8;
        endcase
      end else begin
        disponibilidade <= 4'b0000;
        stall <= 0;
      end
    end
  
endmodule


// IEX 
module IEX 
(
	input clk, rst, stall, 
	input [31:0] i_rd1, i_rd2, i_sigext, i_pc,
	input [4:0] i_inst1, i_inst2, i_inst3,
  	input [1:0] i_aluop, 
	input  i_alusrc, i_regdst, i_regwrite, i_memread, i_memtoreg, i_memwrite, i_branch,
  	input [2:0] select_execute,
	
	output reg [31:0] e_rd1, e_rd2, e_sigext, e_pc, 
	output reg [4:0] e_inst1, e_inst2, e_inst3,
  	output reg [1:0] e_aluop, 
	output reg e_alusrc, e_regdst, e_regwrite, e_memread, e_memtoreg, e_memwrite, e_branch,
  	output reg [2:0] e_select_execute
);

	always @(posedge clk) 
	begin
      if (!rst || stall) 
		begin
			e_regwrite <= 0;
			e_memtoreg <= 0;
			e_branch   <= 0;
			e_memwrite <= 0;
			e_memread  <= 0;
			e_aluop    <= 0;
			e_alusrc   <= 0;
			e_pc       <= 0;
			e_rd1      <= 0;
			e_rd2      <= 0;
			e_sigext   <= 0;
			e_inst1    <= 0;
			e_inst2    <= 0;
			e_inst3    <= 0;
			e_regdst   <= 0;
          	e_select_execute <= 0;
		end
      else if(!stall)
		begin
			e_regwrite <= i_regwrite;
			e_memtoreg <= i_memtoreg;
			e_branch   <= i_branch;
			e_memwrite <= i_memwrite;
			e_memread  <= i_memread;
			e_aluop    <= i_aluop;
			e_alusrc   <= i_alusrc;
			e_pc       <= i_pc;
			e_rd1      <= i_rd1;
			e_rd2      <= i_rd2;
			e_sigext   <= i_sigext;
			e_inst1    <= i_inst1;
			e_inst2    <= i_inst2;
			e_inst3    <= i_inst3;
			e_regdst   <= i_regdst;
            e_select_execute <= select_execute;
		end
	end
endmodule
  

module X0 
  (
    input clk, rst, e_alusrc, e_regdst, e_regwrite, i_branch, //se eh branch
	input [31:0] e_in1, e_in2, e_sigext, e_pc, 
    input [4:0] e_inst2, e_rC, 
    input [1:0] e_aluop, 
    input [2:0] e_select_execute,
    
    output [31:0] x_aluout, w_novoPC,
    output [4:0]  x_register_store_adress,
    output w_branchTomado, w_regwrite, x_pipe_ativo
  );
   
  wire [31:0] alu_B, e_novoPC, e_aluout, x_aluout;
  wire [3:0] aluctrl;
  wire e_zero, pipe_ativo, x_branchTomado;
  wire [4:0] endDest;
  wire [5:0] functImm;
  
   always @*
     begin
       if ( e_select_execute == 3'b001)
          pipe_ativo <= 1;
      else
          pipe_ativo <= 0;
     end
  
  Add Add (e_pc, e_sigext, //soma ou subtrai o PC com o offset
                   e_novoPC);

  assign alu_B = (e_alusrc) ? e_sigext : e_in2 ; //se opera com imm, entao aluSrcB == sigext
  assign endDest = (e_regdst) ? e_rC : e_inst2; //seleciona reg escrita
  assign functImm = (!e_alusrc) ? e_sigext[5:0] : 6'b000001 ; //caso tipo IMM, atribui um valor de funct, nesse caso addi
  assign x_branchTomado = (e_zero && i_branch) ? 1 : 0 ; //sim caso seja branch e alu == zero
  
  Alucontrol alucontrol (e_aluop, functImm, //atribuir funct aqui caso nao seja tipo R
                         	aluctrl);
  
  //Unidade Lógico Aritimética
  ALU alu (aluctrl, e_in1, alu_B, 
           e_aluout, e_zero);

  X0W x0w(clk, rst, pipe_ativo, e_regwrite, x_branchTomado, e_aluout, e_novoPC, endDest, 
          x_aluout, w_novoPC, x_register_store_adress, x_pipe_ativo, w_regwrite, w_branchTomado);
  
endmodule

module X0W (
    input clk, rst, pipe_ativo, x_regwrite, x_branchTomado,
  	input [31:0] e_aluout,x_novoPC ,
  	input [4:0] endDest,
  	
	output [31:0] x_aluout,w_novoPC ,
  	output [4:0] x_register_store_adress,
  	output x_pipe_ativo, w_regwrite, w_branchTomado
);
  always @(posedge clk) 
	begin
		if (!rst) 
		  begin
         	x_aluout    			<= 0;
        	x_register_store_adress <= 0;
            x_pipe_ativo 			<= 0;
            w_regwrite				<= 0;
            w_novoPC				<= 0;
            w_branchTomado			<= 0;
		end
		else 
          begin
			x_aluout 				<= e_aluout;
          	x_register_store_adress <= endDest;
            x_pipe_ativo            <= pipe_ativo;
            w_regwrite				<= x_regwrite;
            w_novoPC				<= x_novoPC;
            w_branchTomado			<= x_branchTomado;
        end
    end
endmodule
      
module Add //add para o pc
(
	input [31:0] pc, shiftleft2, 
	
	output [31:0] add_result_out
);
	wire [31:0] add_result;
	
	assign add_result_out = {17'b0, add_result[14:0]};

	always @(*)
		if(shiftleft2[15] == 0) //se eh negativo
		begin
			add_result[14:0] <= pc + (shiftleft2[14:0] << 2);
		end
		else
		begin
			add_result[14:0] <= pc[14:0] - (shiftleft2[14:0] << 2);
		end
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
	assign zero = (aluout == 0); // v para Branch tomado

	always @(alucontrol, A, B) 
	begin
	//verifica qual o valor do controle para determinar o que fazer com a saída
		case (alucontrol)
			4'b0000 : aluout <= A + B; // add
			4'b0001: aluout <= A - B; // sub
			4'b0011: aluout <= A & B; // and
			4'b1111: aluout <= A < B ? 32'd1 : 32'd0 ; //SLT,
			
			4'b1000: aluout <= A < B ? 32'd0 : 32'd1 ; //BLT
			4'b1100: aluout <= A == B ? 32'd0 : 32'd1 ; //BEQ
			
			default: aluout <= 0; //default 0, Nada acontece;
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

  always @(aluop, funct) 
	begin
		case (aluop) //00 BEQ; 01 BLT
			2'b00: alucontrol <= 4'b1100; // Branch BEQ
			2'b01: alucontrol <= 4'b1000; // BLT
			default:
			begin
				case (funct) //case: funct do tipo r, atribui alucontrol
					1:  alucontrol <= 4'b0000; // ADD
					2:  alucontrol <= 4'b0001; // SUB
					4:  alucontrol <= 4'b0011; // AND
					//8:  alucontrol <= 4'b0111; // MUL
					16: alucontrol <= 4'b1111; // SLT
					default: alucontrol <= 4'd15; // Nada acontece ?
				endcase
			end
		endcase
	end
endmodule
           
module L0 (
  input clk, rst,
  input [4:0] e_rB, //rB endereço que será salvo o valor do load
  input [31:0] e_rd1, e_sigext,
  input [2:0] e_select_execute,
	
  output [31:0] L0_load_aluout,
  output [4:0] L0_register_store_adress,
  output L0_pipe_ativo
);
  wire [31:0] load_aluout;
  wire pipe_ativo;
  
  always @*
     begin
       if (!rst)
          pipe_ativo <= 0;
      else if (e_select_execute == 3'b010)
          pipe_ativo <= 1;
     end

  
  ALU alu (4'b0010, e_rd1, e_sigext, 
           load_aluout, load_zero);
  
  L0L1 l0l1(clk, rst, pipe_ativo, load_aluout, e_rB,
            L0_load_aluout, L0_register_store_adress, L0_pipe_ativo);
endmodule
           
module L0L1 (
    input clk, rst, pipe_ativo,
  	input [31:0] load_aluout, 
  	input [4:0] e_rB,
  	
    output [31:0] L0_load_aluout, 
    output [4:0] L0_register_store_adress,
    output L0_pipe_ativo
);
  always @(posedge clk) 
	begin
		if (!rst) 
		  begin
         	L0_load_aluout 			 <= 0;
        	L0_register_store_adress <= 0;
            L0_pipe_ativo      		 <= 0;
		end
		else 
          begin
			L0_load_aluout 			 <= load_aluout;
          	L0_register_store_adress <= e_rB;
            L0_pipe_ativo      		 <= pipe_ativo;
        end
    end
endmodule
           
module L1 (
  input clk,rst,
  input [31:0] L0_load_aluout,  
  input [4:0] L0_register_store_adress,
  input L0_pipe_ativo,
  
  output [31:0] L1_load_aluout, 
  output [4:0] L1_register_store_adress,
  output L1_pipe_ativo
);
  
  Memory memory (clk, 0, memory_addres, writedata,
                 data_out);
  
  L1W l1w(clk, rst, L0_pipe_ativo, L0_load_aluout, L0_register_store_adress,
          L1_load_aluout, L1_register_store_adress, L1_pipe_ativo);
endmodule
           
module L1W (
  input clk, rst, L0_pipe_ativo,
  input [31:0] L0_load_aluout,  
  input [4:0] L0_register_store_adress,
  
  output [31:0] L1_load_aluout,  
  output [4:0] L1_register_store_adress,
  output L1_pipe_ativo
);
  always @(posedge clk) 
	begin
		if (!rst) 
		  begin
         	L1_load_aluout           <= 0;
        	L1_register_store_adress <= 0;
            L1_pipe_ativo			 <= 0;
		end
		else 
          begin
			L1_load_aluout           <= L0_load_aluout;
          	L1_register_store_adress <= L0_register_store_adress;
            L1_pipe_ativo			 <= L0_pipe_ativo;
        end
    end
endmodule


module Memory (
  input clk, is_store, 
  input [4:0] memory_addres,
  input [31:0] writedata, 
  
  output [31:0] data_out
);

  integer i;
  reg [31:0] memory [0:31]; // 32 positions de 32 bits cada

  // Inicializa a memória
  initial begin
    for (i = 0; i <= 31; i= i + 1) 
      memory[i] <= i+1;
  end
    
  always @(posedge clk) begin
    if (is_store)
      begin
      	data_out <= memory[memory_addres];
      end
    else begin
      memory[memory_addres] <= writedata;
    end
  end
endmodule


//STORE
//rA endereço de destino
//rB + offset endereço source
//rd1 valor a ser salvo
module S0 (
  input clk, rst, 
  input [31:0]  e_rd1, e_rd2, e_sigext,
  input [2:0] e_select_execute,
  
  output [31:0] S0_store_value, 
  output [4:0] S0_register_store_adress,
  output S0_pipe_ativo
);
  
  //wire [4:0] register_store_adress;
  wire [31:0] store_aluout;
  wire pipe_ativo;
  
  always @*
     begin
       if ( e_select_execute == 3'b011)
          pipe_ativo <= 1;
      else
          pipe_ativo <= 0;
     end
  
  ALU alu (4'b0010, e_rd1, e_sigext, //calculo do endereço com offset
           store_aluout, load_zero);

  S0M s0m (clk, rst, pipe_ativo, store_aluout[4:0], e_rd2,
           S0_store_value, S0_register_store_adress, S0_pipe_ativo);
  
endmodule

module S0M (
  input clk, rst, pipe_ativo,
  input [4:0] store_aluout,
  input [31:0]  e_rd2,

  output [31:0] store_value, 
  output [4:0] register_store_adresss,
  output S0_pipe_ativo
    
);
    always @(posedge clk) 
	begin
		if (!rst) 
		  begin
         	store_value 			 <= 0;
        	register_store_adresss 	 <= 0;
            S0_pipe_ativo 			 <= 0;
		end
		else 
          begin
			store_value 		   <= e_rd2;
          	register_store_adresss <= store_aluout;
            S0_pipe_ativo 		   <= pipe_ativo; 
        end
    end
endmodule


//MULTIPLICACAO
//M0
module M0(
	input clk, rst, 
	input [31:0] regA, regB,
	input [4:0] endRegC,
  	input [2:0] e_select_execute,

	output [31:0] regC_out,
  	output [4:0] endRegC_out,
  	output mul0_pipe_ativo
);
	wire [31:0] regC;
  	wire pipe_ativo;
	
  	always @(*) regC <= regA * regB; //multiplicacao em 1 ciclo, mas deve demorar 4
  
  	always @*
     begin
       if ( e_select_execute == 3'b100)
          pipe_ativo <= 1;
      else
          pipe_ativo <= 0;
     end

  M0M1 m0m1(clk, rst, pipe_ativo, regC, endRegC, //entao temos estagios extras nesse pipeline

			regC_out, endRegC_out, mul0_pipe_ativo);
endmodule
module M0M1(
	input clk, rst, pipe_ativo,
	input [31:0] regC,
	input [4:0] endRegC,
	
	output reg [31:0] regC_out,
  	output reg [4:0] endRegC_out,
  	output reg mul0_pipe_ativo
);
	always @(posedge clk)
	if(!rst)
	begin
		regC_out 		<= 0;
		endRegC_out		<= 0;
      	mul0_pipe_ativo <= 0;
	end
	else
	begin
		regC_out 		<= regC;
		endRegC_out		<= endRegC;
      	mul0_pipe_ativo <= pipe_ativo;
	end
endmodule


//M1
module M1(
	input clk, rst, 
	input [31:0] regC,
	input [4:0] endRegC,
  	input mul0_pipe_ativo,

	output [31:0] regC_out,
  	output [4:0] endRegC_out,
  	output mul1_pipe_ativo,
  
);
 	 M1M2 m1m2(clk, rst, mul0_pipe_ativo, regC, endRegC, 
			regC_out, endRegC_out);
endmodule
module M1M2(
	input clk, rst, mul0_pipe_ativo,
	input [31:0] regC,
	input [4:0] endRegC,
	
	output reg [31:0] regC_out,
  	output reg [4:0] endRegC_out,
  	output reg mul1_pipe_ativo
);
	always @(posedge clk)
	if(!rst)
	begin
		regC_out 		<= 0;
		endRegC_out		<= 0;
      	mul1_pipe_ativo	<= 0;
	end
	else
	begin
		regC_out 		<= regC;
		endRegC_out		<= endRegC;
      	mul1_pipe_ativo	<= mul0_pipe_ativo;
	end
endmodule


//M2
module M2(
	input clk, rst, 
	input [31:0] regC,
	input [4:0] endRegC,
  	input mul1_pipe_ativo,

	output [31:0] regC_out,
  	output [4:0] endRegC_out,
  	output mul2_pipe_ativo
);
  M2M3 m2m3(clk, rst, mul1_pipe_ativo, regC, endRegC, 
			regC_out, endRegC_out, mul2_pipe_ativo);
endmodule
module M2M3(
	input clk, rst, mul1_pipe_ativo,
	input [31:0] regC,
	input [4:0] endRegC,
	
	output reg [31:0] regC_out,
  	output reg [4:0] endRegC_out,
  	output reg mul2_pipe_ativo
);
	always @(posedge clk)
	if(!rst)
	begin
		regC_out 		<= 0;
		endRegC_out		<= 0;
      	mul2_pipe_ativo <= 0;
	end
	else
	begin
		regC_out 	<= regC;
		endRegC_out	<= endRegC;
      	mul2_pipe_ativo <= mul1_pipe_ativo;
	end
endmodule

//M3
module M3(
	input clk, rst, 
	input [31:0] regC,
	input [4:0] endRegC,
  	input mul2_pipe_ativo,

	output [31:0] regC_out,
  	output [4:0] endRegC_out,
  	output mul3_pipe_ativo
);
  M3W m3w(clk, rst, mul2_pipe_ativo, regC, endRegC, 
			regC_out, endRegC_out, mul3_pipe_ativo);
endmodule
module M3W(
	input clk, rst, mul2_pipe_ativo,
	input [31:0] regC,
	input [4:0] endRegC,
	
	output reg [31:0] regC_out,
  	output reg [4:0] endRegC_out,
  	output reg mul3_pipe_ativo
);
	always @(posedge clk)
	if(!rst)
	begin
		regC_out 		<= 0;
		endRegC_out		<= 0;
      	mul3_pipe_ativo	<= 0;
	end
	else
	begin
		regC_out 		<= regC;
		endRegC_out		<= endRegC;
      	mul3_pipe_ativo	<= mul2_pipe_ativo;
	end
endmodule


module WB(

	input clk, rst, 
	input [2:0] fonte, //indica a fonte do dado
	//X0
	input [31:0] x_aluout, 
	input [4:0]  x_register_store_adress,
  	input x_pipe_ativo, x_regwrite,
	//Load
	input [31:0] L0_load_aluout,
  	input [4:0] L0_register_store_adress,
  	input L0_pipe_ativo,
	//Store
	input [31:0] S0_store_value, 
    input [4:0] S0_register_store_adress,
  	input S0_pipe_ativo,
    //Multiplicacao
    input [31:0] M_regC,
	input [4:0] M_endRegC,
  	input M_pipe_ativo,
	
	output reg [31:0] dado_regC,
	output reg [4:0] endRegC,
	output reg permitEscrita, //eh_store
	
);

	always @(posedge clk)
		case(fonte)//cada um dos 4 pipelines
			2'b00:
			begin
				dado_regC 		<= x_aluout;
				endRegC			<= x_register_store_adress;
				permitEscrita	<= x_regwrite;
				//eh_store		<= 0;					
			end
			2'b01:
			begin
				dado_regC 		<= L0_load_aluout;
				endRegC			<= L0_register_store_adress;
				permitEscrita	<= 1;				
				//eh_store		<= 0;					
			end
			2'b10:
			begin
				dado_regC 		<= S0_store_value;
				endRegC			<= S0_register_store_adress;
				permitEscrita	<= 0;	
				//eh_store		<= 1;		
			end
			2'b11:
			begin
				dado_regC 		<= M_regC;
				endRegC			<= endRegC;
				permitEscrita	<= 1;
				//eh_store		<= 0;					
			end
		endcase

endmodule










