//NOT RELIABLE WITH E4440A!

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
 
 if (argc<2) {
  fprintf(stderr,"Usage: %s <device> <command>\n",argv[0]);
  exit(-1);
 }
 
 usbtmc_init();
 current_handle=usbtmc_open(argv[1],0,0,1000);
 if (!current_handle) {
  fprintf(stderr,"couldn't open device %s\n",argv[1]);
  return -1;
 } 

// usbtmc_clear(current_handle); 

 a=usbtmc_printf(current_handle,argv[2]);

 usbtmc_readstb(current_handle);
 for(a=0;(a&0x8040)!=0x8040;) {
  usbtmc_readstb(current_handle);
  a=usbtmc_poll_stb(current_handle,1);
  printf("a: %04x\n",a);
  usleep(10000);
 } 
 printf("final status: %04x\n",a);


 usbtmc_close(current_handle);
 return 0;
}
