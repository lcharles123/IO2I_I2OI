#compilador
COM=iverilog

#simulador
SIM=vvp

#visualizador de onda
WAV=gtkwave

all: run

design: *.v
	$(COM) -Wall $^ -o $@.vvp

run: design
	$(SIM) $^.vvp

wave: run
	$(WAV) dump.vcd

clean:
	rm -rf *.vvp
	rm -rf *.vcd

