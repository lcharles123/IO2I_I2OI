module writeback (input [31:0] aluout, readdata, input memtoreg, output reg [31:0] write_data);
  always @(memtoreg) begin
    write_data <= (memtoreg) ? readdata : aluout;
  end
endmodule
