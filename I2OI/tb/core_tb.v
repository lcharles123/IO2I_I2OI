module core_tb ();

	reg clk, rst;
	wire [31:0] writedata;
	integer cont;
		
	initial
	begin
		clk <= 0;
		rst <= 0;
		cont = 0;
		#1 rst <= 1;
		
	end
	
	always #1 clk = ~clk; 
	always @(posedge clk) cont = cont + 1; //contador para o $monitor()
	
	pipemips core(clk, rst, 
					writedata);

	initial 
	begin
        //$dumpfile("dump.vcd"); //gerar arquivo de forma de onda, para ser usado no gtkwave
        //$dumpvars(0,design_tb);

		#1 $monitor("clk %d, writedata = %d",cont , writedata);
       
		#80 $finish; //para a simulacao
        
	end

endmodule



