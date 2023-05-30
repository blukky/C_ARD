CFLAGS=-c -Wall -O2

all: pid_controller.a

gyro.a: pid_controller.o
	ar -rc pid_controller.a pid_controller.o ;\
	sudo cp pid_controller.a /usr/local/lib ;\
	sudo cp pid_controller.h /usr/local/include

pid_controller.o: pid_controller.c
	$(CC) $(CFLAGS) pid_controller.c

clean:
	rm *.o pid_controller.a