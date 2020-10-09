#compilador
COM=iverilog

#simulador
SIM=vvp

#visualizador de onda
WAV=gtkwave

ajuda: help

core: I2OI.v tb/core_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp




wave: run
	$(WAV) dump.vcd

clean:
	rm -rf *.vvp
	rm -rf *.vcd

help:
	@echo "\tmake target\n\n\tcore  \n\tcontrole \n\tfetch \n\tissue \n\tsb \n\talu \n\tex \n\tls \n\twb \n\tprf \n\trob \n\tfsb \n\tc \n\tarf \n" 



