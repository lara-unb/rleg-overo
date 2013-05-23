CC = arm-linux-gnueabi-gcc
CCLAGS = -Wall

SOURCES = $(wildcard *.c)
OBJECTS = $(patsubst %.c, %.o, $(SOURCES))
HEADERS = $(wildcard *.h)
EXECUTABLE =

.PHONY: all
all: $(SOURCES) $(HEADERS) $(OBJECTS) $(EXECUTABLE)

#$(EXECUTABLE): $(OBJECTS)
#        $(CC) $(CCFLAGS) $(OBJECTS) -o $@

gdatalogger.o: gdatalogger.c $(HEADERS)
	$(CC) -c gdatalogger.c

gmatlabdatafile.o: gmatlabdatafile.c $(HEADERS)
	$(CC) -c gmatlabdatafile.c

gqueue.o: gqueue.c $(HEADERS)
	$(CC) -c gqueue.c


.PHONY: print
print:
        @echo Sources: $(SOURCES)
        @echo Objects: $(OBJECTS)
      	@echo Headers: $(HEADERS)
        @echo Executable: $(EXECUTABLE)

.PHONY: clean
clean:
	-rm -f $(EXECUTABLE) *.o
