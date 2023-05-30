CFLAGS=-c -Wall -O2

all: flight_controller.a

flight_controller.a: flight_controller.o
	ar -rc flight_controller.a flight_controller.o ;\
	sudo cp flight_controller.a /usr/local/lib ;\
	sudo cp flight_controller.h /usr/local/include

flight_controller.o: flight_controller.c
	$(CC) $(CFLAGS) flight_controller.c

clean:
	rm *.o flight_controller.a