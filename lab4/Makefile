all: LAB4

LAB4: lab4.o linuxanalog.o 
	gcc lab4.o linuxanalog.o -lrt -o LAB4 

lab4.o: lab4.c
	gcc -c lab4.c

linuxanalog.o: linuxanalog.c linuxanalog.h
	gcc -c linuxanalog.c linuxanalog.h

clean:
	rm *o LAB4
