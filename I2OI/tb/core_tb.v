module core_tb();

	reg clk, rst;
	wire [31:0] saida;
	
	initial
	begin
		#0 clk = 0; rst = 0;
		#2 rst = 1;
	end
	
	always #1 clk = ~clk;
	
	Core core(clk, rst, saida);

	initial 
	begin
        //$dumpfile("dump.vcd"); //gerar arquivo de forma de onda, para ser usado no gtkwave
        //$dumpvars(0,tb);
        
		#1 $monitor("saida: %b", saida);
		#30 $finish;
    end
    

endmodule

