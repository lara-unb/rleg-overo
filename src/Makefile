# Compiler definitions:
CC = arm-linux-gnueabi-gcc -lpthread -lrt -lm -lcurses -I/usr/arm-linux-gnueabi/include -L/usr/arm-linux-gnueabi/lib
CCFLAGS = -Wall

# Main files definition:
MAIN = main2
SOURCES = $(MAIN).c datalogger.c ui.c taskScheduler.c calibration.c
HEADERS = $(MAIN).h datalogger.h ui.h taskScheduler.h #$(wildcard *.h)

# Modules definition:
SUBDIRS = communication gdatalogger
CLEANDIRS = $(SUBDIRS:%=clean-%)

EXECUTABLE = newrleg

##
# Recipes:
##
.PHONY: all
all: subdirs $(SOURCES) $(HEADERS) $(SUBDIRS) $(EXECUTABLE)

.PHONY: subdirs $(SUBDIRS)
subdirs: $(SUBDIRS)
$(SUBDIRS):
	@$(MAKE) -C $@
@echo Objects: $(OBJECTS)

OBJECTS = $(wildcard communication/*.o) $(MAIN).o ui.o datalogger.o taskScheduler.o 
OBJECTS+= $(wildcard gdatalogger/*.o) control/control.o calibration.o

# Datalogger Module
datalogger.o: datalogger.c datalogger.h
	$(CC) $(CCFLAGS) -c datalogger.c

# User Interface Module
ui.o: ui.c ui.h
	$(CC) $(CCFLAGS) -c ui.c

# TaskScheduler Module
taskScheduler.o: taskScheduler.c taskScheduler.h
	$(CC) $(CCFLAGS) -c taskScheduler.c

# Control Module:
control/control: control/control.h subdirs
	$(CC) $(CCFLAGS) -c $@.c -o $@.o


calibration.o: calibration.c calibration.h
	$(CC) $(CCFLAGS) -c calibration.c

##
# Main files:
##
$(MAIN).o: $(MAIN).c $(HEADERS) subdirs
	$(CC) $(CCFLAGS) -c $(MAIN).c

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(CCFLAGS) $(OBJECTS) -o $@

##
# Print all Objects
##
.PHONY: print
print:
        @echo Sources: $(SOURCES)
        @echo Objects: $(OBJECTS)
        @echo Headers: $(HEADERS)
        @echo Executable: $(EXECUTABLE)

        @echo Subdirs: $(SUBDIRS)
        @echo Cleandirs: $(CLEANDIRS)

##
# Clean-up everything
##
.PHONY: subdirs $(CLEANDIRS)
clean: $(CLEANDIRS)
$(CLEANDIRS): 
	@$(MAKE) -C $(@:clean-%=%) clean
	-rm -f $(EXECUTABLE) *.o

##
# Generate Documentation
##
.PHONY: documentation
documentation:
	doxygen Doxyfile
#	cd ../doc/; doxygen -w html header.html footer.html custondoxygen.css
