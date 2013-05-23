#include "gpio_functions.h"

int gpio_read(uint8_t gpio)
{
  FILE *fp_gpio=NULL;
  char path[30];
  char fp_value;

  sprintf(path,"/sys/class/gpio/gpio%d/value",gpio);
  if( (fp_gpio=fopen(path,"rb+")) == NULL )
  {
    printf("Open %s failed\n",path);
    return -1;
  }
  rewind(fp_gpio);
  fread(&fp_value, 1, sizeof(char), fp_gpio);
  fclose(fp_gpio);
  return (uint8_t)fp_value-0x30;  
}

int gpio_write(uint8_t gpio, uint8_t value)
{
  FILE *fp_gpio;
  char path[30];
  char fp_read[4];
  char fp_write;

// Verifying if GPIO is set as Output
  sprintf(path,"/sys/class/gpio/gpio%d/direction",gpio);

  if( (fp_gpio=fopen(path,"r")) == NULL )
  {
    printf("Open %s failed\n",path);
    return -1;
  }
  rewind(fp_gpio);
  fread(fp_read, 3, sizeof(char), fp_gpio);
  if( strcmp(fp_read,"out" ) )
  {
    printf("Gpio%d is set as Input. Impossible to write.",gpio);
    return -2;
  }
  fclose(fp_gpio);

// Writing
  sprintf(path,"/sys/class/gpio/gpio%d/value",gpio);
  
  if( (fp_gpio=fopen(path,"w")) == NULL )
  {
    printf("Open %s failed\n",path);
    return -1;
  }
  rewind(fp_gpio);
  fp_write=0x30+value;
  fwrite(&fp_write, sizeof(char), 1, fp_gpio);
  fclose(fp_gpio);
  return 1;
}

int gpio_f_write(uint8_t gpio, uint8_t value)
{
  FILE *fp_gpio;
  char path[30];
  char fp_value;
  
  sprintf(path,"/sys/class/gpio/gpio%d/value",gpio);
  fp_gpio=fopen(path,"w");
  rewind(fp_gpio);
  fp_value=0x30+value;
  fwrite(&fp_value, sizeof(char), 1, fp_gpio);
  fclose(fp_gpio);
  return 1;
}






