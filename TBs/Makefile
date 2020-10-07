#compilador
COM=iverilog

#simulador
SIM=vvp

#visualizador de onda
WAV=gtkwave

all: run

compile: ../*.v *.v
	$(COM) -Wall $^ -o $@.vvp

run: compile
	$(SIM) $^.vvp

wave: run
	$(WAV) dump.vcd

clean:
	rm -rf *.vvp
	rm -rf *.vcd

