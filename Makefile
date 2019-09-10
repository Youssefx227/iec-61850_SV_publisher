
CC=gcc
CFLAGS=-Wall -O -lpthread -lm
LDFLAGS=-static
LIBPATH = -L../libiec61850-1.3.3/build/
LIB = -liec61850
EXEC=sv_publisher
INCLUDE = -I../inc


all: $(EXEC)
$(EXEC): sv_publisher_example.c
				$(CC) $(INCLUDE)  sv_publisher_example.c -o $(EXEC) $(CFLAGS) $(LIBPATH) $(LIB) 

clean:
	rm *.o

