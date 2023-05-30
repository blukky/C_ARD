CFLAGS=-c -Wall -O2

all: esc.a

esc.a: esc.o
	ar -rc esc.a esc.o ;\
	sudo cp esc.a /usr/local/lib ;\
	sudo cp esc.h /usr/local/include

esc.o: esc.c
	$(CC) $(CFLAGS) esc.c

clean:
	rm *.o esc.a