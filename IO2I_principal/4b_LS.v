module Load_Store 
(
	input [31:0] address, writedata, 
	input memread, memwrite, clk, 
	
	output [31:0] readdata
);

  integer i;
  reg [31:0] memory [0:127]; 
  
  // fill the memory
  initial begin
    for (i = 0; i < 128; i=i+1) 
      memory[i] <= i;
  end

  assign readdata = (memread) ? memory[address[31:2]] : 0;

  always @(posedge clk) begin
    if (memwrite)
      memory[address[31:2]] <= writedata;
	end
endmodule
