CC = g++

all: Main.cpp
	g++ Main.cpp -o Sim

clean:
	rm -rf *~ *.o Sim
