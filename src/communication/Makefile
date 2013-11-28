CC = arm-linux-gnueabi-gcc
CCLAGS = -Wall

SOURCES = $(wildcard *.c)
OBJECTS = $(patsubst %.c, %.o, $(SOURCES))
HEADERS = $(wildcard *.h)

.PHONY: all
all: $(SOURCES) $(HEADERS) $(OBJECTS)

acc_functions.o: acc_functions.c imu_functions.h imu_regs.h
	$(CC) $(CCFLAGS) -c acc_functions.c

mag_functions.o: mag_functions.c imu_functions.h imu_regs.h
	$(CC) $(CCFLAGS) -c mag_functions.c

gyr_functions.o: gyr_functions.c imu_functions.h imu_regs.h
	$(CC) $(CCFLAGS) -c gyr_functions.c

spi_functions.o: spi_functions.c spi_functions.h
	$(CC) $(CCFLAGS) -c spi_functions.c

gpio_functions.o: gpio_functions.c gpio_functions.h
	$(CC) $(CCFLAGS) -c gpio_functions.c

encoder_functions.o: encoder_functions.c encoder_functions.h
	$(CC) $(CCFLAGS) -c encoder_functions.c

communication.o: communication.c communication.h imu_functions.h imu_regs.h encoder_functions.h
	$(CC) $(CCFLAGS) -c communication.c

.PHONY: print
print:
        @echo Sources: $(SOURCES)
        @echo Objects: $(OBJECTS)
        @echo Headers: $(HEADERS)

.PHONY: clean
clean:
	-rm -f *.o

