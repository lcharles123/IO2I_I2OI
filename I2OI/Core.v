module Core (input clk, rst, output [31:0] writedata);
  
  wire [31:0] inst, sigext, data1, data2, aluout, readdata;
  wire zero, memread, memwrite, memtoreg, branch, alusrc;
  wire [9:0] funct;
  wire [1:0] aluop;
  
  // FETCH STAGE
  fetch fetch (zero, rst, clk, branch, sigext, inst);
  
  // DECODE STAGE
  decode decode (inst, writedata, clk, data1, data2, sigext, alusrc, memread, memwrite, memtoreg, branch, aluop, funct);   
  
  // EXECUTE STAGE
  execute execute (data1, data2, sigext, alusrc, aluop, funct, zero, aluout);

  // MEMORY STAGE
  memory memory (aluout, data2, memread, memwrite, clk, readdata);

  // WRITEBACK STAGE
  writeback writeback (aluout, readdata, memtoreg, writedata);

endmodule
