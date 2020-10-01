module Execute_ALU 
(
	input [31:0] in1, in2, ImmGen, 
	input alusrc, input [1:0] aluop, 
	input [9:0] funct, 
	
	output zero, 
	output [31:0] aluout
);

  wire [31:0] alu_B;
  wire [3:0] aluctrl;
  
  assign alu_B = (alusrc) ? ImmGen : in2 ;

  //Unidade Lógico Aritimética
  ALU alu (aluctrl, in1, alu_B, aluout, zero);

  alucontrol alucontrol (aluop, funct, aluctrl);

endmodule

module alucontrol (input [1:0] aluop, input [9:0] funct, output reg [3:0] alucontrol);
  
  wire [7:0] funct7;
  wire [2:0] funct3;

  assign funct3 = funct[2:0];
  assign funct7 = funct[9:3];

  always @(aluop) begin
    case (aluop)
      0: alucontrol <= 4'd2; // ADD to SW and LW
      1: alucontrol <= 4'd6; // SUB to branch
      default: begin
        case (funct3)
          0: alucontrol <= (funct7 == 0) ? /*ADD*/ 4'd2 : /*SUB*/ 4'd6; 
          2: alucontrol <= 4'd7; // SLT
          6: alucontrol <= 4'd1; // OR
          //39: alucontrol <= 4'd12; // NOR
          7: alucontrol <= 4'd0; // AND
          default: alucontrol <= 4'd15; // Nop
        endcase
      end
    endcase
  end
endmodule

module ALU (input [3:0] alucontrol, input [31:0] A, B, output reg [31:0] aluout, output zero);
  
  assign zero = (aluout == 0); // Zero recebe um valor lógico caso aluout seja igual a zero.
  
  always @(alucontrol, A, B) begin
      case (alucontrol)
        0: aluout <= A & B; // AND
        1: aluout <= A | B; // OR
        2: aluout <= A + B; // ADD
        6: aluout <= A - B; // SUB
        //7: aluout <= A < B ? 32'd1:32'd0; //SLT
        //12: aluout <= ~(A | B); // NOR
      default: aluout <= 0; //default 0, Nada acontece;
    endcase
  end
endmodule

