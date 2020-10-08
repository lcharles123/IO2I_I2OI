module geral_tb();

	reg [15:0] ex1_memory [0:3]; //tamanho 
    initial
    begin
        $display("Loading rom.");
		#0 $readmemh("instrucoes.mem", ex1_memory);
    end
    
    initial 
    begin
		$display("memoria 0: %h | ",ex1_memory[0]);		
		$display("memoria 1: %h | ",ex1_memory[1]);		
		$display("memoria 2: %h | ",ex1_memory[2]);
		$display("memoria 3: %h | ",ex1_memory[31]);
    end
    
endmodule



