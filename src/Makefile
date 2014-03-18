# Compiler definitions:
CC = arm-linux-gnueabi-gcc -lpthread -lrt -lm -lcurses -I/usr/arm-linux-gnueabi/include -L/usr/arm-linux-gnueabi/lib
CCFLAGS = -Wall

# Main files definition:
MAIN = main
SOURCES = $(MAIN).c datalogger.c ui.c calibration.c taskScheduler.c
HEADERS = $(MAIN).h datalogger.h ui.h taskScheduler.h #$(wildcard *.h)

# Modules definition:
SUBDIRS = communication gdatalogger
CLEANDIRS = $(SUBDIRS:%=clean-%)

EXECUTABLE = newrleg

.PHONY: all
all: subdirs $(SOURCES) $(HEADERS) $(SUBDIRS) $(EXECUTABLE)

.PHONY: subdirs $(SUBDIRS)
subdirs: $(SUBDIRS)
$(SUBDIRS):
	@$(MAKE) -C $@
@echo Objects: $(OBJECTS)

OBJECTS = $(wildcard communication/*.o) $(MAIN).o ui.o datalogger.o calibration.o taskScheduler.o 
OBJECTS+= $(wildcard gdatalogger/*.o)

#EXECUTABLE = newrleg

datalogger.o: datalogger.c datalogger.h
	$(CC) $(CCFLAGS) -c datalogger.c

ui.o: ui.c ui.h
	$(CC) $(CCFLAGS) -c ui.c

taskScheduler.o: taskScheduler.c taskScheduler.h
	$(CC) $(CCFLAGS) -c taskScheduler.c

# calibration.o: calibration.c calibration.h
# 	$(CC) $(CCFLAGS) -c calibration.c

$(MAIN).o: $(MAIN).c $(HEADERS)
	$(CC) $(CCFLAGS) -c $(MAIN).c

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(CCFLAGS) $(OBJECTS) -o $@


.PHONY: print
print:
        @echo Sources: $(SOURCES)
        @echo Objects: $(OBJECTS)
        @echo Headers: $(HEADERS)
        @echo Executable: $(EXECUTABLE)

        @echo Subdirs: $(SUBDIRS)
        @echo Cleandirs: $(CLEANDIRS)

.PHONY: subdirs $(CLEANDIRS)
clean: $(CLEANDIRS)
$(CLEANDIRS): 
	@$(MAKE) -C $(@:clean-%=%) clean
	-rm -f $(EXECUTABLE) *.o

doc:
	doxygen Doxyfile
	# cd ../doc/; doxygen -w html header.html footer.html custondoxygen.css
