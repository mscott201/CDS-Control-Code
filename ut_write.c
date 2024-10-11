#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "tmc.h"
#include "ut.h"

int main(int argc, char *argv[])
{
// char *bus_name="any", *device_name="any";
 unsigned char buf[65536*16+8];
 int a,b;
 usbtmc_device_data *current_handle;

 if (argc<3) {
  fprintf(stderr,"Usage: %s <device> <command>\n",argv[0]);
  exit(-1);
 }
 
 usbtmc_init();

// current_handle=usbtmc_open(argv[1],0);
  current_handle=usbtmc_open(argv[1],0,0,1000);
// desc_get_eplen(&ep2,&ep4,&ep6,&ep8);

 if (!current_handle) return -1;
// usbtmc_clear(current_handle); 
 a=usbtmc_printf(current_handle,argv[2]);
 usbtmc_close(current_handle);
 return 0;
}
