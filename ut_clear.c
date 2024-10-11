#include <stdio.h>
#include <stdlib.h>
#include "tmc.h"
#include "ut.h"

int main(int argc, char *argv[])
{
// char *bus_name="any", *device_name="any";
 unsigned char buf[65536*16+8];
 int a,b;
 usbtmc_device_data *current_handle;

 if (argc<1) {
  fprintf(stderr,"Usage: %s <device>\n",argv[0]);
  exit(-1);
 }
 
 usbtmc_init();

 current_handle=usbtmc_open(argv[1],0,0,1000);

 if (!current_handle) return -1;
 usbtmc_clear(current_handle); 
 usbtmc_close(current_handle);
 return 0;
}
