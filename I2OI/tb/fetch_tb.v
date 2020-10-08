module fetch_tb();

	reg zero, rst, clk, branch;
	reg [31:0] branch_res;
	
	wire [31:0] inst;
	
	//wire [31:0] saida;
	
	initial clk = 0;
	
	always #1 clk = ~clk;

	Fetch fetch (zero, rst, clk, branch, branch_res, inst);
  
  
	initial 
	begin
        //$dumpfile("dump.vcd"); //gerar arquivo de forma de onda, para ser usado no gtkwave
        //$dumpvars(0,tb);
        
		#1 $monitor("clk: %b, inst: %b", clk , inst);
		
		
		
		#30 $finish;
    end
    

endmodule

