module prf_tb();

	reg clk;
	//wire [31:0] saida;
	
	initial clk = 0;
	
	always #1 clk = ~clk;
	
	;

	initial 
	begin
        //$dumpfile("dump.vcd"); //gerar arquivo de forma de onda, para ser usado no gtkwave
        //$dumpvars(0,tb);
        
		#1 $monitor(": %b", );
		
		
		
		
		
		#30 $finish;
    end
    

endmodule

