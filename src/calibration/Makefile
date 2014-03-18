CC = arm-linux-gnueabi-gcc
CCLAGS = -Wall

SOURCES = $(wildcard *.c)
OBJECTS = $(patsubst %.c, %.o, $(SOURCES))
HEADERS = $(wildcard *.h) ../communication/communication.h
EXECUTABLE =

.PHONY: all
all: $(SOURCES) $(HEADERS) $(OBJECTS) $(EXECUTABLE)

calibration.o: $(HEADERS)
	$(CC) -c calibration.c 


.PHONY: print
print:
        @echo Sources: $(SOURCES)
        @echo Objects: $(OBJECTS)
      	@echo Headers: $(HEADERS)
        @echo Executable: $(EXECUTABLE)

.PHONY: clean
clean:
	-rm -f $(EXECUTABLE) *.o
