
#ifdef __cplusplus
extern "C" {
#endif

#include "tmc.h"


struct usbtmc_dev_capabilities {
	unsigned char interface_capabilities;
	unsigned char device_capabilities;
	unsigned char usb488_interface_capabilities;
	unsigned char usb488_device_capabilities;
};

typedef struct  {
        struct usb_device     * dev;
        struct usb_dev_handle * handle;

        unsigned int bulk_in; 
        unsigned int bulk_out;
	unsigned int int_in;

        unsigned int max_size_in; 
        unsigned int max_size_out;
	unsigned int max_size_int;

        unsigned char bTag;
        unsigned char bTag_last_write;     /* needed for abort */
        unsigned char bTag_last_read;      /* needed for abort */

        /* attributes from the USB TMC spec for this device */
        unsigned char TermChar;
        int  TermCharEnabled;
        int auto_abort;

        int zombie; /* fd of disconnected device */

        struct usbtmc_dev_capabilities  capabilities;
//        struct mutex io_mutex;  /* only one i/o function running at a time */

        unsigned char *buffer;
        int bufsize;
        int err;
        
        int max_send;
        int io_timeout;

} usbtmc_device_data;

void usbtmc_init();
usbtmc_device_data *usbtmc_open(char *busdevice,char *iProduct,char *iSerial,int io_timeout);
int usbtmc_clear(usbtmc_device_data *lnk);
void usbtmc_close( usbtmc_device_data *lnk);
int usbtmc_local(usbtmc_device_data *lnk);
int usbtmc_remote(usbtmc_device_data *lnk);
unsigned int usbtmc_flush(usbtmc_device_data *lnk,int timeout);

int usbtmc_readstb(usbtmc_device_data *lnk);
int usbtmc_poll_stb(usbtmc_device_data *lnk,int timeout);
int usbtmc_trigger(usbtmc_device_data *lnk);

int usbtmc_read( usbtmc_device_data *lnk, unsigned char *buf, int len,int *reason);
unsigned int usbtmc_recv_file(usbtmc_device_data *lnk,int fd,unsigned int len,int *reason);
int usbtmc_read_binary(usbtmc_device_data *lnk,char * data,int len);
int usbtmc_recv_binary_file(usbtmc_device_data *lnk,char * name);
int usbtmc_recv_named_file(usbtmc_device_data *lnk,char * name);


int usbtmc_write(usbtmc_device_data *lnk,unsigned char * buf,int len,int end);
int usbtmc_printf(usbtmc_device_data *lnk, char * fmt, ...);
unsigned int usbtmc_send_file(usbtmc_device_data *lnk,int fd,unsigned int len,int end);

#ifdef __cplusplus
};
#endif
