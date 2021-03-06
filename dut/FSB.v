module test(	input clk, rst, escreverMEM, escreveMEM, //primeiro sinal vem do store, o segundo vem da memoria permitindo a escrita
	input [4:0] enderecoMEM,
	input [32:0] dadoMEM,
	
	output reg [4:0] enderecoMEM_out,
	output reg [32:0] dadoMEM_out,
	output reg cheio, escreverMEM_out);



Finished_Store_Buffer fsb(
	 clk, rst, escreverMEM, escreveMEM, //primeiro sinal vem do store, o segundo vem da memoria permitindo a escrita
	 enderecoMEM,
	 dadoMEM,
	
	enderecoMEM_out,
	dadoMEM_out,
	 cheio, escreverMEM_out
);
endmodule

module Finished_Store_Buffer(
	input clk, rst, escreverMEM, escreveMEM, //primeiro sinal vem do store, o segundo vem da memoria permitindo a escrita nela
	input [4:0] enderecoMEM,
	input [32:0] dadoMEM,
	
	output reg [4:0] enderecoMEM_out,
	output reg [32:0] dadoMEM_out,
	output reg cheio, escreverMEM_out //verificar cheio para stall de stores
);

	assign escreverMEM_out = escreveMEM && cheio;

	always @(posedge clk)
	begin
		if(!rst) 
		begin
			enderecoMEM_out	<= 0;
			dadoMEM_out		<= 0;
			cheio			<= 0;
		end
		else 
		begin
			if(!cheio && escreverMEM) //condicao para passar o dado para memoria
			begin
				enderecoMEM_out	<= enderecoMEM;
				dadoMEM_out		<= dadoMEM;
				cheio = 1 ;
			end
		end
		if(escreveMEM) //indica dado entrou saiu do buffer
			cheio = 0;
	end
endmodule
