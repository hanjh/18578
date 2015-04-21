OBJS = main.o
CC = g++
DEBUG = -g
CFLAGS = -Wall -c $(DEBUG)
LFLAGS = -Wall $(DEBUG)

demo : $(OBJS); $(CC) $(LFLAGS) $(OBJS) -o demo

main.o : serial_com.h serial_com.cpp optical_flow.h optical_flow.cpp; $(CC) $(CFLAGS) main.cpp

clean:; \rm *.o *~ demo

