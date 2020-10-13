#compilador
COM=iverilog

#simulador
SIM=vvp

#visualizador de onda
WAV=gtkwave

principal: help

core: I2OI.v tb/core_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

io2i: I2OI.v 
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp


#wave: run
#	$(WAV) dum.vcd

clean:
	rm -rf *.vvp
	rm -rf *.vcd

help:
	@echo "\tmake [target]\n\n\tlista de targets:\n\tcore \n\tclean\n" 



