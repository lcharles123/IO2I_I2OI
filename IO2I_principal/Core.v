//interface com o core

module Core 
(
	input clk, rst, 
	
	output [31:0] writedata
);
  
  wire [31:0] inst, sigext, data1, data2, aluout, readdata;
  wire zero, memread, memwrite, memtoreg, branch, alusrc;
  wire [9:0] funct;
  wire [1:0] aluop;
  
  // FETCH STAGE
  Fetch fetch (zero, rst, clk, branch, sigext, inst);
  
  // DECODE STAGE
  Decode decode (inst, writedata, clk, data1, data2, sigext, alusrc, memread, memwrite, memtoreg, branch, aluop, funct);   
  
  // EXECUTE STAGE
  Execute_ALU execute (data1, data2, sigext, alusrc, aluop, funct, zero, aluout);

  // MEMORY STAGE
  Load_Store memory (aluout, data2, memread, memwrite, clk, readdata);

  // WRITEBACK STAGE
  Writeback writeback (aluout, readdata, memtoreg, writedata);

endmodule
