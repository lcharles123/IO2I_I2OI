#compilador
COM=iverilog

#simulador
SIM=vvp

#visualizador de onda
WAV=gtkwave

principal: help

fsb: dut/FSB.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

rob: dut/ROB.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

sb: dut/SB.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp



#wave: run
#	$(WAV) dum.vcd

clean:
	rm -rf *.vvp
	rm -rf *.vcd

help:
	@echo "\tmake [target]\n\n\tlista de targets:\n\tfsb\n\tcore\n\tsb\n\trob \n\tclean\n" 



