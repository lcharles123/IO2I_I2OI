module IDI 
(
	input clk, rst, stall, d_regwrite, d_memtoreg, d_branch, d_memwrite, d_memread, d_regdst, d_alusrc, 
	input [1:0] d_aluop, 
	input [31:0] d_pc, d_sigext, 
	input [5:0] d_opcode, //adicao de opcode
	input [4:0] d_inst1, d_inst2, d_inst3,
	
	output reg e_regwrite, e_memtoreg, e_branch, e_memwrite, e_memread, e_regdst, e_alusrc, 
	output reg [1:0] e_aluop, 
	output reg [31:0] e_pc, e_sigext, 
	output reg [4:0] e_inst1, e_inst2, e_inst3
);

	reg [5:0] 	opcode 	[0:7]; //o IQ tera 8 entradas, 6 bits para o opcode
	reg [31:0] 	imm 	[0:7]; // 32 bits para sigext == imm
	
	reg [1:0] 	v 		[0:7]; //se o dest eh valido(todas exceto branch e store)
	reg [4:0] 	dest  	[0:7]; //endDest do banco de regs
	
	reg [1:0] 	v0 		[0:7]; //se fonte0 eh valido
	reg [1:0]	p0 		[0:7];//se fonte0 esta pendente
	reg [5:0]	fonte0 	[0:7];//end fonte0
		
	reg [1:0]	v1 		[0:7];//se fonte1 eh valido
	reg [1:0]	p1 		[0:7];//se fonte1 esta pendente
	reg [5:0] 	fonte1 	[0:7];//end fonte1
	
	reg [1:2] indice_inst_antiga; //guarda o indice da entrada
	reg [1:2] indice_inst_pronta; //guarda o indice da entrada
	reg [1:2] indice_inst_mais_recente; //guarda o indice da entrada
	reg 	  cheio;
	
/*Op: Opcode
Imm: Imediato
S: bit especulativo
V: Válido (Existe no Src e Dest)
P: Pendente
*/
	





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
			e_sigext   <= 0;
			e_inst1    <= 0;
			e_inst2    <= 0;
			e_inst3    <= 0;
			d_opcode   <= 0;
			cheio	   <= 0;
			indice_inst_antiga   		<= 0; 
			indice_inst_pronta  		<= 0; 
			indice_inst_mais_recente	<= 0;
		end
		else 
		begin
			if(!stall)
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
				e_sigext   <= d_sigext;
				e_inst1    <= d_inst1;
				e_inst2    <= d_inst2;
				e_inst3    <= d_inst3;
			end
				
				
	reg [5:0] 	opcode 	[0:7]; //o IQ tera 8 entradas, 6 bits para o opcode
	reg [31:0] 	imm 	[0:7]; // 32 bits para sigext == imm
	
	reg [1:0] 	v 		[0:7]; //se o dest eh valido(todas exceto branch e store)
	reg [4:0] 	dest  	[0:7]; //endDest do banco de regs
	
	reg [1:0] 	v1 		[0:7]; //se fonte0 eh valido
	reg [1:0]	p1 		[0:7];//se fonte0 esta pendente
	reg [5:0]	fonte1 	[0:7];//end fonte0
		
	reg [1:0]	v2 		[0:7];//se fonte1 eh valido
	reg [1:0]	p2 		[0:7];//se fonte1 esta pendente
	reg [5:0] 	fonte2 	[0:7];//end fonte1
	
	reg [1:0]	v3 		[0:7];//se fonte1 eh valido
	reg [1:0]	p3 		[0:7];//se fonte1 esta pendente
	reg [5:0] 	fonte3 	[0:7];//end fonte1
	
	reg [1:2] indice_inst_antiga; //guarda o indice da entrada
	reg [1:2] indice_inst_pronta; //guarda o indice da entrada
	reg [1:2] indice_inst_mais_recente; //guarda o indice da entrada
		
		if(!cheio)
		begin
			opcode[indice_inst_mais_recente] = d_opcode;

			imm[indice_inst_mais_recente] = d_sigext;
			
			v[indice_inst_mais_recente] = ~(d_branch || d_memwrite); //se branch ou store -> dest nao valido
			dest[indice_inst_mais_recente] = d_inst3; //endDest do banco de regs

			v1[indice_inst_mais_recente]  = 1; //se fonte1 eh valido
			p1[indice_inst_mais_recente]  = stall;//se fonte1 esta pendente
			fonte1[indice_inst_mais_recente] = d_inst1;//end fonte1
		
			v2[indice_inst_mais_recente]  = 1;//fonte1 eh valido em todos (pois nao tem BEQ por exemplo)
			p2[indice_inst_mais_recente]  = stall;//se fonte2 esta pendente
			fonte2[indice_inst_mais_recente] = (opcode == 6'b000001) ? e_inst3 : e_inst2;//apenas opcode define fonte2
			
			indice_inst_mais_recente = indice_inst_mais_recente + 1;
			
			if(indice_inst_mais_recente == 8)	indice_inst_mais_recente = 0;
			
		end
		
		if()
		
			v 		[0:7]; //se o dest eh valido(todas exceto branch e store)
			dest  	[0:7]; //endDest do banco de regs
	
	reg [1:0] 	v0 		[0:7]; //se fonte0 eh valido
	reg [1:0]	p0 		[0:7];//se fonte0 esta pendente
	reg [5:0]	fonte0 	[0:7];//end fonte0
		
	reg [1:0]	v1 		[0:7];//se fonte1 eh valido
	reg [1:0]	p1 		[0:7];//se fonte1 esta pendente
	reg [5:0] 	fonte1 	[0:7];//end fonte1
			
			
			
			
			indice_inst_mais_recente = indice_inst_mais_recente + 1;
			cheio = cheio + 1;
		end
		else
		begin
			
		end
			
			
			
			
			
			
			
			
			
			
		end
endmodule
