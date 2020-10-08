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

controle: I2OI.v tb/controle_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

fetch: I2OI.v tb/fetch_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

decode: I2OI.v tb/decode_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

issue: I2OI.v tb/issue_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

sb: I2OI.v tb/sb_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

alu: I2OI.v tb/alu_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

ex: I2OI.v tb/ex_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

ls: I2OI.v tb/_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

wb: I2OI.v tb/wb_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

prf: I2OI.v tb/prf_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

rob: I2OI.v tb/rob_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

fsb: I2OI.v tb/fsb_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

c: I2OI.v tb/c_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp

arf: I2OI.v tb/arf_tb.v
	$(COM) -Wall $^ -o $@.vvp
	$(SIM) $@.vvp







wave: run
	$(WAV) dump.vcd

clean:
	rm -rf *.vvp
	rm -rf *.vcd

help:
	@echo "\tmake target\n\n\tcore  \n\tcontrole \n\tfetch \n\tissue \n\tsb \n\talu \n\tex \n\tls \n\twb \n\tprf \n\trob \n\tfsb \n\tc \n\tarf \n" 



