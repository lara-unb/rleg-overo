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

acc_functions.o: acc_functions.c imu_functions.h imu_regs.h
	$(CC) -c acc_functions.c

mag_functions.o: mag_functions.c imu_functions.h imu_regs.h
	$(CC) -c mag_functions.c

gyr_functions.o: gyr_functions.c imu_functions.h imu_regs.h
	$(CC) -c gyr_functions.c

spi_functions.o: spi_functions.c spi_functions.h
	$(CC) -c spi_functions.c

gpio_functions.o: gpio_functions.c gpio_functions.h
	$(CC) -c gpio_functions.c

encoder_functions.o: encoder_functions.c encoder_functions.h
	$(CC) -c encoder_functions.c

communication.o: communication.c communication.h imu_functions.h imu_regs.h encoder_functions.h
	$(CC) -c communication.c

.PHONY: print
print:
        @echo Sources: $(SOURCES)
        @echo Objects: $(OBJECTS)
        @echo Headers: $(HEADERS)
        @echo Executable: $(EXECUTABLE)

.PHONY: clean
clean:
	-rm -f $(EXECUTABLE) *.o

