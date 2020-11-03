
module Score_Board 
(
	input clk, rst,
	input [4:0] regC, //endRegC
	input [1:0] unidade_fun_in, // indica qual unidade funcional a instrucao usa
	
  output  [0:31] pendente //valor de toda a coluna "Pendente", basta usar os enderecos de regA, regB, regC nessa saida
  
);	
	reg		[0:31]  pendente ; // 32 linhas de 1 bit cada , indexavel
	reg [1:0] unidade_funcional [0:31]; // indica qual pipeline escrevera no registrador
	reg [4:0] dado_disponivel [0:31]; //dado disponivel 32 linhas de 5 bits cada

	
	always @(posedge clk)
	begin
		if(!rst) 
		begin
			case(unidade_fun_in) //preenche a linha com os dados de determinado pipeline
              2'b00 : //ALU
				begin
					pendente[regC] <= 1'b1;
					dado_disponivel[regC] <= 5'b00010;
				end
				2'b01 :	//Load
				begin
					pendente[regC] <= 1'b1;
					dado_disponivel[regC] <= 5'b00100;
				end
				2'b10 :	//Store
				begin
					pendente[regC] <= 1'b1;
					dado_disponivel[regC] <= 5'b00010;
				end
				2'b11 :	//MUL
				begin
					pendente[regC] <= 1'b1;
					dado_disponivel[regC] <= 5'b10000;
				end
			endcase 
		end
	end

endmodule
