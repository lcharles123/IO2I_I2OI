module Controle 
(
	input [6:0] opcode, 
	input [31:0] inst, 

	output reg alusrc, memtoreg, regwrite, memread, memwrite, branch, 
	output reg [1:0] aluop, 
	output reg [31:0] ImmGen
);

  always @(opcode) begin
	alusrc   <= 0;
    memtoreg <= 0;
    regwrite <= 0;
    memread  <= 0;
    memwrite <= 0;
    branch   <= 0;
    aluop    <= 0;
    ImmGen   <= 0; 
    case(opcode) 
      7'b0110011: begin // R type == 51
        regwrite <= 1;
        aluop    <= 2;
			end
		  7'b1100011: begin // beq == 99
        branch   <= 1;
        aluop    <= 1;
        ImmGen   <= {{19{inst[31]}},inst[31],inst[7],inst[30:25],inst[11:8],1'b0};
			end
			7'b0010011: begin // addi == 19
        alusrc   <= 1;
        regwrite <= 1;
        ImmGen   <= {{20{inst[31]}},inst[31:20]};
      end
			7'b0000011: begin // lw == 3
        alusrc   <= 1;
        memtoreg <= 1;
        regwrite <= 1;
        memread  <= 1;
        ImmGen   <= {{20{inst[31]}},inst[31:20]};
      end
			7'b0100011: begin // sw == 35
        alusrc   <= 1;
        memwrite <= 1;
        ImmGen   <= {{20{inst[31]}},inst[31:25],inst[11:7]};
      end
    endcase
  end

endmodule 
