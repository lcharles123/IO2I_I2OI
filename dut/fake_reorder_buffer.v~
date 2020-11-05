module circuit(
	input clk, rst,
  	input [31:0] dado,
    input [4:0] endereco,
    input [4:0] contador,

  	output [31:0] dado_escrita,
  output [4:0] endereco_escrita, posicao_atual

);
  Reorder_Buffer rb(clk, rst, dado, endereco, contador,
                 dado_escrita, endereco_escrita, posicao_atual);
endmodule


module Reorder_Buffer 
(
	input clk, rst,
  input [31:0] dado,
  input [4:0] endereco,
  input [4:0] contador,

  output [31:0] dado_escrita,
  output [4:0] endereco_escrita, posicao_atual
);
  reg [36:0] PRF [0:4];
  reg [4:0] posicao_atual; 

  always @(posedge clk) begin
    if (!rst) begin
      PRF[contador] = {endereco, dado};
      if (PRF[posicao_atual] != 0) begin
        dado_escrita = PRF[posicao_atual][31:0];
        endereco_escrita = PRF[posicao_atual][36:32];
        posicao_atual= posicao_atual + 1;
      end else begin
        dado_escrita = 0;
        endereco_escrita = 0;
      end
    end else begin
      PRF[0] = 0;
      PRF[1] = 0;
      PRF[2] = 0;
      PRF[3] = 0;
      PRF[4] = 0;
      posicao_atual = 0;
    end
      

    /*
    if branch_tomado
    posicao_atual += 3
    */
    /*
    if posicao_atual == 32
      posicao_atual = 0
    */

  end
endmodule
