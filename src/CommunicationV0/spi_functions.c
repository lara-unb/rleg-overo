/* Modified from SPI testing utility (using spidev driver)
 * Linux/Documentation/spi/spidev_test.c
*/

#include "spi_functions.h"

int pabort(const char *s)
{
	perror(s);
	return -1;
}

//static uint8_t mode = 0;
static uint8_t bits = 8;
//static uint32_t speed = 46875;
static uint16_t delay;

/*
void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}

void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}
*/
int spi_init(uint8_t mode, uint32_t speed, uint8_t cs/*int argc, char *argv[]*/)
{
	int ret = 0;
	int spi_dev;
	char *device;

	//parse_opts(argc, argv);
	switch(cs){
	  case 0:
	    device = "/dev/spidev1.0";
	    break;
	  case 1:
	    device = "/dev/spidev1.1";
	    break;
	  default:
	    return pabort("invalid chip select value");
	}
	    
	
	spi_dev = open(device, O_RDWR);
	if (spi_dev < 0)
		return pabort("can't open device");
printf("\nspi_dev = %d\n",spi_dev);
	/*
	 * spi mode
	 */
	ret = ioctl(spi_dev, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		return pabort("can't set spi mode");

	ret = ioctl(spi_dev, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		return pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(spi_dev, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		return pabort("can't set bits per word");

	ret = ioctl(spi_dev, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		return pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(spi_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		return pabort("can't set max speed hz");

	ret = ioctl(spi_dev, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		return pabort("can't get max speed hz");

	//printf("spi mode: %d\n", mode);
	//printf("bits per word: %d\n", bits);
	//printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	return spi_dev;
}

void spi_end(int spi_dev)
{
  close(spi_dev);
  return;
}

short int adc_read(int spi_dev, uint8_t ch, uint8_t sgl)
{
	int ret;
	if( ch<0 || ch>7 ) return pabort("Wrong channel value for ADC");
	uint8_t tx[] = {(ch>>2)|(sgl<<1)|0x04, ch<<6, 0x00,};
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = 0,
		.bits_per_word = 0,
	};

	ret = ioctl(spi_dev, SPI_IOC_MESSAGE(1), &tr);
	if (ret == 1) return pabort("can't send spi message");
//printf("\nRx = 0x%X 0x%X 0x%X\n",rx[0],rx[1],rx[2]);
//printf("\nValor retornado pelo conversor AD: %d\n\n",rx[2]+((rx[1]&0x0F)<<8));
	return rx[2]+((rx[1]&0x0F)<<8);
}

uint8_t dac_write(int spi_dev, uint8_t ch, uint8_t _shdn, unsigned short int data)
{
	int ret;
	union txunion{
	unsigned short int data16;
	uint8_t data8[2];
	}txunion;
	if( ch<0 || ch>1 ) return pabort("Wrong channel value for DAC");
	if( data>4095 )	return pabort("Value exceeded for DAC"); 
	txunion.data16 = 0x6000|(ch<<15)|(_shdn<<12)|(data&0x0FFF);
	uint8_t tx[] = {txunion.data8[0], txunion.data8[1]};
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = 0,
		.bits_per_word = 0,
	};

	ret = ioctl(spi_dev, SPI_IOC_MESSAGE(1), &tr);
	if (ret == 1) return pabort("can't send spi message");
//printf("\nRx = 0x%X 0x%X 0x%X\n",rx[0],rx[1],rx[2]);
//printf("\nValor retornado pelo conversor AD: %d\n\n",rx[2]+((rx[1]&0x0F)<<8));
	return 1;
}






