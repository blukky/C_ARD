CFLAGS=-c -Wall -O2 -lwiringPy -lm
CC = gcc
all: gyro.a

gyro.a: gyro.o
	ar -rc gyro.a gyro.o ;\
	sudo cp gyro.a /usr/local/lib ;\
	sudo cp gyro.h /usr/local/include

gyro.o: gyro.c
	$(CC) $(CFLAGS) gyro.c

clean:
	rm *.o gyro.a