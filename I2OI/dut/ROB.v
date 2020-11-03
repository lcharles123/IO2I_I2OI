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

