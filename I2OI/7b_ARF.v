module ARF 
(
	input clk, regwrite, 
	input [4:0] read_reg1, read_reg2, writereg, 
	input [31:0] writedata, 
	
	output [31:0] read_data1, read_data2
);

	integer i;
	reg [31:0] memory [0:31]; // 32 registers de 32 bits cada

	// fill the memory
	initial 
	begin
		for (i = 0; i <= 31; i=i+1) 
			memory[i] <= i;
	end

	assign read_data1 = (regwrite && read_reg1==writereg) ? writedata : memory[read_reg1];
	assign read_data2 = (regwrite && read_reg2==writereg) ? writedata : memory[read_reg2];

	always @(posedge clk) 
	begin
		if (regwrite)
			memory[writereg] <= writedata;
	end

endmodule
