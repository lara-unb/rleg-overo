SUBDIRS = communication gdatalogger
CC = arm-linux-gnueabi-gcc -lpthread -lrt -lm -lcurses -I/usr/arm-linux-gnueabi/include -L/usr/arm-linux-gnueabi/lib
CCFLAGS = -Wall

SOURCES = main2.c datalogger.c ui.c calibration.c
HEADERS = main2.h datalogger.h ui.h #$(wildcard *.h)
CLEANDIRS = $(SUBDIRS:%=clean-%)

#SOURCES = main.c
#OBJECTS = $(wildcard $(SUBDIRS)/*.o) main.o
#HEADERS = $(wildcard *.h)

#@echo Subdirs: $(SUBDIRS)

#EXECUTABLE = main

.PHONY: all
all: subdirs $(SOURCES) $(HEADERS) $(SUBDIRS) newrleg

.PHONY: subdirs $(SUBDIRS)
subdirs: $(SUBDIRS)
$(SUBDIRS):
	@$(MAKE) -C $@
@echo Objects: $(OBJECTS)

OBJECTS = $(wildcard communication/*.o) main2.o datalogger.o ui.o calibration.o
OBJECTS+= $(wildcard gdatalogger/*.o)


newrleg: $(OBJECTS)
	$(CC) $(CCFLAGS) $(OBJECTS) -o $@

main2.o: main2.c main2.h
	$(CC) $(CCFLAGS) -c main2.c

datalogger.o: datalogger.c datalogger.h
	$(CC) $(CCFLAGS) -c datalogger.c

ui.o: ui.c ui.h
	$(CC) $(CCFLAGS) -c ui.c


calibration.o: calibration.c calibration.h
	$(CC) $(CCFLAGS) -c calibration.c




.PHONY: print
print:
        @echo Sources: $(SOURCES)
        @echo Objects: $(OBJECTS)
      	@echo Headers: $(HEADERS)
        @echo Executable: newrleg

        @echo Subdirs: $(SUBDIRS)
        @echo Cleandirs: $(CLEANDIRS)

.PHONY: subdirs $(CLEANDIRS)
clean: $(CLEANDIRS)
$(CLEANDIRS): 
	@$(MAKE) -C $(@:clean-%=%) clean
	-rm -f newrleg *.o



