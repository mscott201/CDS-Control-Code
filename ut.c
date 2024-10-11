//usbtmc-wrapper
//TODO:
//timeout useage now reasonably ok? should we allow users to set abort, clear timeouts?
//do we need more than one timeout? not really if we poll for data availability...?
//match abort function to some vxi11 if possible

//clear halt, etc in the open function?

//as far as I understand we are supposed to poll the interrupt endpoint at bIntervall
//this is implemented using usbtmc_poll_stb
//(do we need to use libusb-1 instead?) 

//lock, unlock not supported over usbtmc (makes sense)

//note that the behavior of usbtmc_readstb, trigger, local,remote depends on the capabilities



#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <string.h>
#include <usb.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>
#include "ut.h"


#ifndef DEBUG
 #define DEBUG 0
#endif



/*
 * This structure is the capabilities for the device
 * See section 4.2.1.8 of the USBTMC specification,
 * and section 4.2.2 of the USBTMC usb488 subclass
 * specification for details.
 */




static int usbtmc_ioctl_abort_bulk_in(usbtmc_device_data *lnk);
static int usbtmc_ioctl_abort_bulk_out(usbtmc_device_data *lnk);
static int usbtmc_get_capabilities(usbtmc_device_data *lnk);


/*

 usb_set_configuration(dev->handle,configuration);
 usb_claim_interface(dev->handle,interface);
 usb_release_interface(dev->handle,interface);
 usb_reset(dev->handle);
 usb_resetep(dev->handle,ep);
 usb_clear_halt(dev->handle,ep);
 usb_get_descriptor(dev->handle,type,index,buf,size);
 usb_get_descriptor_by_endpoint(dev->handle,ep,type,index,buf,size);
*/



//with this version null pointers should be sent to disable matching 
struct usb_device *find_device(char *busname, char *devicename, char *iProduct,char *iSerial) {
 struct usb_bus *p;
 struct usb_device *q;
 struct usb_dev_handle * h;

 char buf_iProduct[256],buf_iSerial[256];
 p=usb_busses;
 while(p!=NULL){
  q=p->devices;

  if (busname && (strcmp(p->dirname, busname))){
   p=p->next;
   continue;
  }
  
  while(q!=NULL){
//   printf("device name: '%s'\n",q->filename);	
//   printf("%d %d %d %d\n",q->config->interface->altsetting->bInterfaceClass,q->config->interface->altsetting->bInterfaceSubClass,q->config->interface->altsetting->bInterfaceProtocol,q->config->interface->altsetting->iInterface);

   if (q->config->interface->altsetting->bInterfaceClass!=0xfe) {q=q->next; continue; }	//Application Specific Interface		
   if (q->config->interface->altsetting->bInterfaceSubClass!=3) {q=q->next; continue; }//Test and Measurement
   if (q->config->interface->altsetting->bInterfaceProtocol>1)  {q=q->next; continue; } //TMC without or with USB488   

   if (devicename && (strcmp(q->filename, devicename))) {q=q->next; continue; }

//   usb_get_descriptor(, unsigned char type, unsigned char index, void *buf, int size);

   if (!iProduct && !iSerial) return q;

   h=usb_open(q);
   if (!h)  {q=q->next; continue; }
   usb_get_string_simple(h,q->descriptor.iProduct,buf_iProduct,256);
   buf_iProduct[255]=0;
   usb_get_string_simple(h,q->descriptor.iProduct,buf_iSerial,256);
   buf_iSerial[255]=0;
   usb_close(h);	//easiest just to close now
 
    
   if (iProduct && (strcmp(buf_iProduct,iProduct))) {q=q->next; continue; }

   if (iSerial && (strcmp(buf_iSerial,iSerial))) {q=q->next; continue; }

//   q=q->next;
   return q;
  }
  p=p->next;
 }
 return NULL;
}


#if 1

static inline int usb_endpoint_dir_in(const struct usb_endpoint_descriptor *epd ) {  
        return ((epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN);
}

static inline int usb_endpoint_dir_out( const struct usb_endpoint_descriptor *epd) {
        return ((epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT);
}

static inline int usb_endpoint_xfer_bulk( const struct usb_endpoint_descriptor *epd) {
        return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK);
}


static inline int usb_endpoint_is_bulk_in( const struct usb_endpoint_descriptor *epd) {
        return usb_endpoint_xfer_bulk(epd) && usb_endpoint_dir_in(epd);
}

static inline int usb_endpoint_is_bulk_out( const struct usb_endpoint_descriptor *epd) { 
        return usb_endpoint_xfer_bulk(epd) && usb_endpoint_dir_out(epd);
}

static inline int usb_endpoint_xfer_int( const struct usb_endpoint_descriptor *epd) { 
        return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_INT);
}
                                                        

static inline int usb_endpoint_is_int_in( const struct usb_endpoint_descriptor *epd) {
        return usb_endpoint_xfer_int(epd) && usb_endpoint_dir_in(epd);
}
                                        

#endif

void usbtmc_init() {
  usb_init();
  usb_find_busses();
  usb_find_devices();
}

//"any" matches any device

usbtmc_device_data *usbtmc_open(char *busdevice,char *iProduct,char *iSerial,int io_timeout) {
 struct usb_device *dev; 
 usbtmc_device_data *w;
 struct usb_interface_descriptor *udesc;
 struct usb_endpoint_descriptor *endpoint;
 int n;
 char *bus=0, *device=0;

 if (busdevice) {
  if (busdevice[0]==':') {
   if (busdevice[1]) device=busdevice+1;
  }else {
   bus=strtok(busdevice,":"); 
   device=strtok(0,"");
  }
 }

 dev=find_device(bus,device,iProduct,iSerial);

 if (!dev) {
  fprintf(stderr,"Device %s:%s product: %s serial: %s not found\n",bus,device,iProduct,iSerial);
  return 0;
 }

 
 w=malloc(sizeof(usbtmc_device_data));
 w->handle=usb_open(dev);    
 w->dev=dev;

 /* Find bulk in endpoint */
 
 //assume altsetting1 as usbtmc devices should only have one ..?
 udesc=dev->config->interface->altsetting;
 
 for (n = 0; n < udesc->bNumEndpoints; n++) {
  endpoint = &(udesc->endpoint[n]);
  if (usb_endpoint_is_bulk_in(endpoint)) {
   w->bulk_in = endpoint->bEndpointAddress;
   w->max_size_in = endpoint->wMaxPacketSize;

#if DEBUG
 fprintf(stderr,"Found bulk in endpoint at %u, max packet size: %d\n", w->bulk_in,w->max_size_in);
#endif
   break;
  }
 }

/* Find bulk out endpoint */
 for (n = 0; n < udesc->bNumEndpoints; n++) {
  endpoint = &(udesc->endpoint[n]);
  if (usb_endpoint_is_bulk_out(endpoint)) {
   w->bulk_out = endpoint->bEndpointAddress;
   w->max_size_out = endpoint->wMaxPacketSize;
#if DEBUG   
 fprintf(stderr, "Found Bulk out endpoint at %u, max packet size: %d\n", w->bulk_out,w->max_size_out);
#endif
   break;
  }
 }


/* Find interrupt_in endpoint */
 for (n = 0; n < udesc->bNumEndpoints; n++) {
  endpoint = &(udesc->endpoint[n]);
  if (usb_endpoint_is_int_in(endpoint)) {
   w->int_in = endpoint->bEndpointAddress;
   w->max_size_int = endpoint->wMaxPacketSize;
#if DEBUG   
 fprintf(stderr, "Found Int in endpoint at %u, max packet size: %d\n", w->int_in,w->max_size_int);
#endif
   break;
  }
 }



/*
 FIXME: should we do this?
 usb_clear_halt(w->handle,0);
 usb_clear_halt(w->handle,2);
 usb_clear_halt(w->handle,0x86);

 usb_set_configuration(w->handle, 1);
*/


 if(usb_claim_interface(w->handle, udesc->bInterfaceNumber)<0){
  fprintf(stderr,"Could not claim interface %d: %s\n", udesc->bInterfaceNumber,usb_strerror());
  return 0;
 }
 
 w->buffer=malloc(200*65536+16);	
 w->bufsize=200*65536;	//max size excluding header (12 byte+4 spare)

 usbtmc_get_capabilities(w);
  
 w->zombie = 0;

        /* Initialize USBTMC bTag and other fields */
 w->bTag      = 1;
 w->TermCharEnabled = 0;
 w->TermChar = '\n';
  

 
 w->auto_abort=1;

 if (io_timeout) w->io_timeout=io_timeout; else w->io_timeout=USBTMC_TIMEOUT;

 w->max_send=w->bufsize;	//maximum amount of data to send using a single call to usbtmc_write
 
// w->io_timeout=1000;
 return w;
}

void usbtmc_close( usbtmc_device_data *lnk) {
 struct usb_interface_descriptor *udesc;

 udesc=lnk->dev->config->interface->altsetting;
 usb_release_interface(lnk->handle, udesc->bInterfaceNumber);

 usb_close(lnk->handle);
 free(lnk->buffer);
 free(lnk);
}


//read into lnk->buffer, this function differs from the kernel version in that 
//it expects calls for max lnk->buffsize bytes, and expects the user to repeat as neccecary
//reason&4=end detected   reason&1=completed returning requested amount of data
int usbtmc_read_internal( usbtmc_device_data *lnk, int len, int *reason) {
 unsigned int n_characters;
// unsigned char buffer[USBTMC_SIZE_IOBUFFER];
 unsigned int actual;
 size_t done;
 size_t remaining;
 int retval;
 size_t this_part;

// printf("*\n");

 *reason=0;
 
//mutex_lock(&lnk->io_mutex);
 if (lnk->zombie) {
  retval = -ENODEV;
  goto exit;
 }

 remaining = len;
 done = 0;

//we only request "transfers" of the size of our buffer at a time
//(the HCD automatically combines packages of less than wMaxPacketSize bytes into our reply)

//  if (remaining > USBTMC_SIZE_IOBUFFER - 12 - 3)
//   this_part = USBTMC_SIZE_IOBUFFER - 12 - 3;
//  else
 this_part = remaining;

/* Setup IO buffer for DEV_DEP_MSG_IN message
 * Refer to class specs for details
 */
 lnk->buffer[0] = 2;
 lnk->buffer[1] = lnk->bTag;
 lnk->buffer[2] = ~(lnk->bTag);
 lnk->buffer[3] = 0; /* Reserved */
 lnk->buffer[4] = (this_part) & 255;
 lnk->buffer[5] = ((this_part) >> 8) & 255;
 lnk->buffer[6] = ((this_part) >> 16) & 255;
 lnk->buffer[7] = ((this_part) >> 24) & 255;
 lnk->buffer[8] = lnk->TermCharEnabled * 2;/* Use term character? */
 lnk->buffer[9] = lnk->TermChar;
 lnk->buffer[10] = 0; /* Reserved */
 lnk->buffer[11] = 0; /* Reserved */

/* Send bulk URB */
 retval = usb_bulk_write(lnk->handle,lnk->bulk_out,(char *)lnk->buffer,12,lnk->io_timeout); 

/* Store bTag (in case we need to abort) */
 lnk->bTag_last_write = lnk->bTag;

/* Increment bTag -- and increment again if zero */
 lnk->bTag++;
 if (!lnk->bTag) (lnk->bTag)++;

 if (retval < 0) {
  fprintf(stderr, "usb_bulk_write returned %d\n", retval);
  if (lnk->auto_abort)    usbtmc_ioctl_abort_bulk_out(lnk);

  goto exit;
 }

/* Send bulk URB */
 retval = usb_bulk_read(lnk->handle,lnk->bulk_in,(char *)(lnk->buffer),len,lnk->io_timeout);
 actual=retval;

/* Store bTag (in case we need to abort) */
 lnk->bTag_last_read = lnk->bTag;

 if (retval < 0) {
  fprintf(stderr, "Unable to read data, error %d\n", retval);
  if (lnk->auto_abort) usbtmc_ioctl_abort_bulk_in(lnk);
  
  goto exit;
 }
 
/* How many characters did the instrument send? */
 n_characters = lnk->buffer[4] +
                (lnk->buffer[5] << 8) +
                (lnk->buffer[6] << 16) +
	        (lnk->buffer[7] << 24);

/* Ensure the instrument doesn't lie about it */
 if(n_characters > actual - 12) {
  fprintf(stderr, "Device lies about message size: %u > %d\n", n_characters, actual - 12);
  n_characters = actual - 12;
 }

/* Ensure the instrument doesn't send more back than requested */
 if(n_characters > this_part) {
  fprintf(stderr, "Device returns more than requested: %zu > %zu\n", done + n_characters, done + this_part);
  n_characters = this_part;
 }

/* Bound amount of data received by amount of data requested */
 if (n_characters > this_part) n_characters = this_part;

 done += n_characters;
  
//I don't quite get what the latter part is supposed to do...?
//if we didn't get n_characters  but the device feels done
//something is clearly wrong, but is staying to read more the right thing to do?
//if ((buffer[8] &  0x01) && (actual >= n_characters + 12)) { 
 if (lnk->buffer[8] &  0x01) { 
  *reason=4;
  remaining = 0;
 } else remaining -= n_characters;

 if (done==(unsigned int)len) (*reason)|=1; 
 retval = done;

exit:
//mutex_unlock(&lnk->io_mutex);
 return retval;
}



//copy to user supplied buffer
//reason&4=end detected   reason&1=completed returning requested amount of data
int usbtmc_read( usbtmc_device_data *lnk, unsigned char *buf, int len,int *reason) {
 int a;
 a=usbtmc_read_internal(lnk,len,reason);
 if(a>0) memcpy(buf,lnk->buffer+12,a);
 return a; 
}






int usbtmc_write_internal(usbtmc_device_data *lnk,int len,int end){  
 int retval;
// int actual;
 unsigned long int n_bytes;
 int remaining;
 int done;
 int this_part;
 unsigned char *p;

//	mutex_lock(&lnk->io_mutex);
 if (lnk->zombie) {
  retval = -ENODEV;
  goto exit;
 }

 remaining = len;
 done = 0;

// while (remaining > 0) {
//  if (remaining > USBTMC_SIZE_IOBUFFER - 12) {
//   this_part = USBTMC_SIZE_IOBUFFER - 12;
//   buffer[8] = 0;
//  } else {
   this_part = remaining;
//   buffer[8] = 1;
   lnk->buffer[8]=end;
//  }

/* Setup IO buffer for DEV_DEP_MSG_OUT message */
  lnk->buffer[0] = 1;
  lnk->buffer[1] = lnk->bTag;
  lnk->buffer[2] = ~(lnk->bTag);
  lnk->buffer[3] = 0; /* Reserved */
  lnk->buffer[4] = this_part & 255;
  lnk->buffer[5] = (this_part >> 8) & 255;
  lnk->buffer[6] = (this_part >> 16) & 255;
  lnk->buffer[7] = (this_part >> 24) & 255;
/* buffer[8] is set above... */
  lnk->buffer[9] = 0; /* Reserved */
  lnk->buffer[10] = 0; /* Reserved */
  lnk->buffer[11] = 0; /* Reserved */

//  memcpy(buffer+12,buf+done,this_part);

//according to the specification we must write a multiple of 4 bytes
//  n_bytes = roundup(12 + this_part, 4);
  if (this_part&3)  n_bytes=((12+this_part)&(~3))+4; else
    n_bytes=((12+this_part)&(~3));
  
//  memset(buffer + 12 + this_part, 0, n_bytes - (12 + this_part));

  p=lnk->buffer;
  do {
   retval = usb_bulk_write(lnk->handle,lnk->bulk_out,(char*)p,n_bytes,lnk->io_timeout);
   if (retval < 0) break;  //was !=0, 
	
   n_bytes -= retval;
   p+=retval;
  } while (n_bytes);

  lnk->bTag_last_write = lnk->bTag;
  lnk->bTag++;

  if (!lnk->bTag) lnk->bTag++;	//bTag should never be 0 (according to spec)

  if (retval < 0) {
   fprintf(stderr,"Unable to send data, error %d\n", retval);
   if (lnk->auto_abort) usbtmc_ioctl_abort_bulk_out(lnk);
   goto exit;
  }

  remaining -= this_part;
  done += this_part;
// }

 retval = done;
exit:
// mutex_unlock(&lnk->io_mutex);
 return retval;
}

int usbtmc_write(usbtmc_device_data *lnk,unsigned char * buf,int len,int end){  
 memcpy(lnk->buffer+12,buf,len);
 return usbtmc_write_internal(lnk,len,end);
}



//INITIATE_ABORT_BULK_IN
static int usbtmc_ioctl_abort_bulk_in(usbtmc_device_data *lnk) {
// unsigned char buffer[USBTMC_SIZE_IOBUFFER];
 int rv;
 int n;
 unsigned int actual;

 rv = usb_control_msg(lnk->handle,
                      USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_ENDPOINT,
                      USBTMC_REQUEST_INITIATE_ABORT_BULK_IN,
                      lnk->bTag_last_read, lnk->bulk_in,
                      (char *)lnk->buffer, 2, USBTMC_ABORT_TIMEOUT);

 if (rv < 0) {
  fprintf(stderr,"usb_control_msg(USBTMC_REQUEST_INITIATE_ABORT_BULK_IN) returned %d\n", rv);
  goto exit;
 }

 fprintf(stderr,"INITIATE_ABORT_BULK_IN returned %x\n", lnk->buffer[0]);

 if (lnk->buffer[0] == USBTMC_STATUS_FAILED) {
  rv = 0;
  goto exit;
 }

 if (lnk->buffer[0] != USBTMC_STATUS_SUCCESS) {
  fprintf(stderr, "INITIATE_ABORT_BULK_IN returned %x\n", lnk->buffer[0]);
  rv = -EPERM;
  goto exit;
 }


//Read until empty
 n = 0;
 do {
  fprintf(stderr,"Reading from bulk in EP\n");

  rv = usb_bulk_read(lnk->handle,lnk->bulk_in,(char *)lnk->buffer,lnk->bufsize,USBTMC_ABORT_TIMEOUT);
  actual=rv;

  n++;

  if (rv < 0) {
   fprintf(stderr,"usb_bulk_msg returned %d\n", rv);
   goto exit;
  }
 } while ((actual == lnk->max_size_in) &&  (n < USBTMC_MAX_READS_TO_CLEAR_BULK_IN));

 if (actual == lnk->max_size_in) {
  fprintf(stderr,"Couldn't clear device buffer within %d cycles\n",USBTMC_MAX_READS_TO_CLEAR_BULK_IN);
  rv = -EPERM;
  goto exit;
 }

 n = 0;

usbtmc_abort_bulk_in_status:
 rv = usb_control_msg(lnk->handle,
                       USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_ENDPOINT,
		       USBTMC_REQUEST_CHECK_ABORT_BULK_IN_STATUS,
		       0, lnk->bulk_in, (char *)lnk->buffer, 0x08,
		       USBTMC_ABORT_TIMEOUT);		       

 if (rv < 0) {
  fprintf(stderr,"usb_control_msg(USBTMC_REQUEST_CHECK_ABORT_BULK_IN_STATUS) returned %d\n", rv);
  goto exit;
 }

 fprintf(stderr, "INITIATE_ABORT_BULK_IN returned %x\n", lnk->buffer[0]);

 if (lnk->buffer[0] == USBTMC_STATUS_SUCCESS) {
  rv = 0;
  goto exit;
 }

 if (lnk->buffer[0] != USBTMC_STATUS_PENDING) {
  fprintf(stderr, "INITIATE_ABORT_BULK_IN returned %x\n", lnk->buffer[0]);
  rv = -EPERM;
  goto exit;
 }

 if (lnk->buffer[1] == 1) do {
  fprintf(stderr, "Reading from bulk in EP\n");

  rv = usb_bulk_read(lnk->handle,lnk->bulk_in,(char *)lnk->buffer,lnk->bufsize,USBTMC_ABORT_TIMEOUT);
  actual=rv;

  n++;

  if (rv < 0) {
   fprintf(stderr, "usb_bulk_msg returned %d\n", rv);
   goto exit;
  }
 } while ((actual = lnk->max_size_in) && (n < USBTMC_MAX_READS_TO_CLEAR_BULK_IN));

 if (actual == lnk->max_size_in) {
  fprintf(stderr, "Couldn't clear device buffer within %d cycles\n", USBTMC_MAX_READS_TO_CLEAR_BULK_IN);
  rv = -EPERM;
  goto exit;
 }

 goto usbtmc_abort_bulk_in_status;

exit:
 return rv;
}



static int usbtmc_ioctl_abort_bulk_out(usbtmc_device_data *lnk) {
// unsigned char lnk->buffer[USBTMC_SIZE_IOBUFFER];
 int rv;
 int n; 

 rv = usb_control_msg(lnk->handle,
                      USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_ENDPOINT,
                      USBTMC_REQUEST_INITIATE_ABORT_BULK_OUT,
                      lnk->bTag_last_write, lnk->bulk_out,
                      (char *)lnk->buffer, 2, USBTMC_ABORT_TIMEOUT);

 if (rv < 0) {
  fprintf(stderr, "usb_control_msg(USBTMC_REQUEST_INITIATE_ABORT_BULK_OUT) returned %d\n", rv);
  goto exit;
 }

#if DEBUG
 fprintf(stderr, "INITIATE_ABORT_BULK_OUT returned %x\n", lnk->buffer[0]);
#endif

 if (lnk->buffer[0] != USBTMC_STATUS_SUCCESS) {
  fprintf(stderr, "INITIATE_ABORT_BULK_OUT returned %x\n", lnk->buffer[0]);
  rv = -EPERM;
  goto exit;  
 }

 n = 0;

usbtmc_abort_bulk_out_check_status:

 rv = usb_control_msg(lnk->handle,
                      USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_ENDPOINT,
                      USBTMC_REQUEST_CHECK_ABORT_BULK_OUT_STATUS,
                      0, lnk->bulk_out, (char*)lnk->buffer, 0x08,
                      USBTMC_ABORT_TIMEOUT);
 n++;
 if (rv < 0) {
  fprintf(stderr, "usb_control_msg(USBTMC_REQUEST_CHECK_ABORT_BULK_OUT_STATUS) returned %d\n", rv);
  goto exit;
 }

 fprintf(stderr, "CHECK_ABORT_BULK_OUT returned %x\n", lnk->buffer[0]);

 if (lnk->buffer[0] == USBTMC_STATUS_SUCCESS)
    goto usbtmc_abort_bulk_out_clear_halt;

 if ((lnk->buffer[0] == USBTMC_STATUS_PENDING) && (n < USBTMC_MAX_READS_TO_CLEAR_BULK_IN))
    goto usbtmc_abort_bulk_out_check_status;

 rv = -EPERM;
 goto exit;  

usbtmc_abort_bulk_out_clear_halt:

 rv = usb_clear_halt(lnk->handle, lnk->bulk_out);

 if (rv < 0) {
  fprintf(stderr, "usb_clear_halt returned %d\n", rv);
  goto exit;
 }
 rv = 0;

exit:
 return rv;
}


//INITIATE_CLEAR
int usbtmc_clear(usbtmc_device_data *lnk) {
// unsigned char buffer[USBTMC_SIZE_IOBUFFER];
 int rv;
 int n;
 int actual;

 fprintf(stderr, "Sending INITIATE_CLEAR request\n");

 rv = usb_control_msg(lnk->handle,
		      USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		      USBTMC_REQUEST_INITIATE_CLEAR,
		      0, 0, (char *)lnk->buffer, 1, USBTMC_CLEAR_TIMEOUT);		      
		      
		     
 if (rv < 0) {
  fprintf(stderr,"usb_control_msg(USBTMC_REQUEST_INITIATE_CLEAR) returned %d\n", rv);
  goto exit;
 }

 fprintf(stderr, "INITIATE_CLEAR returned %x\n", lnk->buffer[0]);

 if (lnk->buffer[0] != USBTMC_STATUS_SUCCESS) {
  fprintf(stderr, "INITIATE_CLEAR returned %x\n", lnk->buffer[0]);
  rv = -EPERM;
  goto exit;
 }

 n = 0;

usbtmc_clear_check_status:

 fprintf(stderr, "Sending CHECK_CLEAR_STATUS request\n");

 rv = usb_control_msg(lnk->handle,
		      USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		      USBTMC_REQUEST_CHECK_CLEAR_STATUS,
		      0, 0, (char *)lnk->buffer, 2, USBTMC_CLEAR_TIMEOUT);

		     
 if (rv < 0) {
  fprintf(stderr, "usb_control_msg(USBTMC_REQUEST_CHECK_CLEAR_STATUS) returned %d\n", rv);
  goto exit;
 }

 fprintf(stderr, "CHECK_CLEAR_STATUS returned %x\n", lnk->buffer[0]);

 if (lnk->buffer[0] == USBTMC_STATUS_SUCCESS)
  goto usbtmc_clear_bulk_out_halt;

 if (lnk->buffer[0] != USBTMC_STATUS_PENDING) {
  fprintf(stderr,"CHECK_CLEAR_STATUS returned %x\n", lnk->buffer[0]);
  rv = -EPERM;
  goto exit;
 }

 if (lnk->buffer[1] == 1) do {
  fprintf(stderr,"Reading from bulk in EP\n");

  rv = usb_bulk_read(lnk->handle,lnk->bulk_in,(char *)lnk->buffer,lnk->bufsize,USBTMC_CLEAR_TIMEOUT);
  actual=rv;		    
		    
  n++;

  if (rv < 0) {
   fprintf(stderr, "usb_bulk_read returned %d\n", rv);
   goto exit;
  }
 } while ((actual == (int)lnk->max_size_in) &&  (n < USBTMC_MAX_READS_TO_CLEAR_BULK_IN));

 if (actual == (int)lnk->max_size_in) {
  fprintf(stderr, "Couldn't clear device buffer within %d cycles\n", USBTMC_MAX_READS_TO_CLEAR_BULK_IN);
  rv = -EPERM;
  goto exit;
 }

 goto usbtmc_clear_check_status;

usbtmc_clear_bulk_out_halt:

 //rv = usb_clear_halt(lnk->usb_dev, usb_sndbulkpipe(lnk->usb_dev, lnk->bulk_out));
 rv = usb_clear_halt(lnk->handle, lnk->bulk_out);
 if (rv < 0) {
  fprintf(stderr, "usb_clear_halt(bulk_read) returned %d\n", rv);
  goto exit;
 }
 rv = 0;

exit:
 //kfree(buffer);
 return rv;
}


//from the kernel driver
static int usbtmc_get_capabilities(usbtmc_device_data *lnk) {
// struct device *dev = &lnk->dev->dev;
// char buffer[USBTMC_SIZE_IOBUFFER];
 int rv = 0;

 rv = usb_control_msg(lnk->handle,
                      USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                      USBTMC_REQUEST_GET_CAPABILITIES,
                      0, 0, (char *)lnk->buffer, 0x18, lnk->io_timeout);
                      
                      
 if (rv < 0) {
  fprintf(stderr, "usb_control_msg(USBTMC_REQUEST_GET_CAPABILITIES) returned %d, %s\n", rv,usb_strerror());
  goto err_out;
 }

#if DEBUG
 fprintf(stderr, "GET_CAPABILITIES returned %x\n", lnk->buffer[0]);
#endif
 if (lnk->buffer[0] != USBTMC_STATUS_SUCCESS) {
  fprintf(stderr, "GET_CAPABILITIES returned %x\n", lnk->buffer[0]);
  rv = -EPERM;
  goto err_out;
 }
	
#if DEBUG	
 fprintf(stderr, "Interface capabilities are %x\n", lnk->buffer[4]);
 fprintf(stderr, "Device capabilities are %x\n", lnk->buffer[5]);
 fprintf(stderr, "USB488 interface capabilities are %x\n", lnk->buffer[14]);
 fprintf(stderr, "USB488 device capabilities are %x\n", lnk->buffer[15]);
#endif

 lnk->capabilities.interface_capabilities = lnk->buffer[4];
 lnk->capabilities.device_capabilities = lnk->buffer[5];
 lnk->capabilities.usb488_interface_capabilities = lnk->buffer[14];
 lnk->capabilities.usb488_device_capabilities = lnk->buffer[15];
 rv = 0;

err_out:
 return rv;
}

//returns the status byte IF the device doesn't have interrupt capability 
//if it has, it returns 0 and sends the status on the interrupt EP
int usbtmc_readstb(usbtmc_device_data *lnk) {
// char buffer[USBTMC_SIZE_IOBUFFER];
 int rv = 0;

 rv = usb_control_msg(lnk->handle,
                      USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                      USBTMC_REQUEST_READ_STATUS_BYTE,
                      0, 0, (char *)lnk->buffer, 0x3, lnk->io_timeout);
                      
                      
 if (rv < 0) {
  fprintf(stderr, "usb_control_msg(USBTMC_REQUEST_READ_STATUS_BYTE) returned %d, %s\n", rv,usb_strerror());
  goto err_out;
 }

 /*D*///fprintf(stderr, "USBTMC_REQUEST_READ_STATUS_BYTE returned %x\n", lnk->buffer[0]);
 if (lnk->buffer[0] != USBTMC_STATUS_SUCCESS) {
  fprintf(stderr, "USBTMC_REQUEST_READ_STATUS_BYTE returned %x\n", lnk->buffer[0]);
  rv = -EPERM;
  goto err_out;
 }
 
 /*D*///fprintf(stderr,"status byte: 0x%x\n",lnk->buffer[2]);

 return (unsigned char)lnk->buffer[2];

err_out:
 return rv;
}



int usbtmc_trigger(usbtmc_device_data *lnk){  
// unsigned char buffer[USBTMC_SIZE_IOBUFFER];
 int retval;

//	mutex_lock(&lnk->io_mutex);
 if (lnk->zombie) {
  retval = -ENODEV;
  goto exit;
 }

/* Setup IO buffer for DEV_DEP_MSG_OUT message */
 lnk->buffer[0] = 1;
 lnk->buffer[1] = lnk->bTag;
 lnk->buffer[2] = ~(lnk->bTag);
 lnk->buffer[3] = 0; /* Reserved */
 lnk->buffer[4] = 0;
 lnk->buffer[5] = 0;
 lnk->buffer[6] = 0;
 lnk->buffer[7] = 0;
 lnk->buffer[8] = 0;
 lnk->buffer[9] = 0; /* Reserved */
 lnk->buffer[10] = 0; /* Reserved */
 lnk->buffer[11] = 0; /* Reserved */

 retval = usb_bulk_write(lnk->handle,lnk->bulk_out,(char *)lnk->buffer,12,lnk->io_timeout);
                      
 lnk->bTag_last_write = lnk->bTag;
 lnk->bTag++;

 if (!lnk->bTag) lnk->bTag++;	//why?

 if (retval < 0) {
  fprintf(stderr,"Unable to send data, error %d\n", retval);
  if (lnk->auto_abort) usbtmc_ioctl_abort_bulk_out(lnk);
  goto exit;
 }

 retval = 0;
exit:
// mutex_unlock(&lnk->io_mutex);
 return retval;
}





int usbtmc_local(usbtmc_device_data *lnk) {
// char lnk->buffer[USBTMC_SIZE_IOBUFFER];
 int rv = 0;

 rv = usb_control_msg(lnk->handle,
                      USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                      USBTMC_REQUEST_GO_TO_LOCAL,
                      0, 0, (char *)lnk->buffer, 0x1, lnk->io_timeout);
                      
                      
 if (rv < 0) {
  fprintf(stderr, "usb_control_msg(USBTMC_REQUEST_GO_TO_LOCAL) returned %d, %s\n", rv,usb_strerror());
  goto err_out;
 }

#if DEBUG 
 fprintf(stderr, "USBTMC_REQUEST_GO_TO_LOCAL returned %x\n", lnk->buffer[0]);
#endif
 if (lnk->buffer[0] != USBTMC_STATUS_SUCCESS) {
  fprintf(stderr, "USBTMC_REQUEST_GO_TO_LOCAL returned %x\n", lnk->buffer[0]);
  rv = -EPERM;
  goto err_out;
 }

 return 0; 

err_out:
 return rv;
}

int usbtmc_remote(usbtmc_device_data *lnk) {
// char buffer[USBTMC_SIZE_IOBUFFER];
 int rv = 0;

 rv = usb_control_msg(lnk->handle,
                      USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                      USBTMC_REQUEST_REN_CONTROL,
                      0, 0, (char *)lnk->buffer, 0x1, lnk->io_timeout);
                      
                      
 if (rv < 0) {
  fprintf(stderr, "usb_control_msg(USBTMC_REQUEST_REN_CONTROL) returned %d, %s\n", rv,usb_strerror());
  goto err_out;
 }

#if DEBUG
 fprintf(stderr, "USBTMC_REQUEST_REN_CONTROL returned %x\n", lnk->buffer[0]);
#endif
 if (lnk->buffer[0] != USBTMC_STATUS_SUCCESS) {
  fprintf(stderr, "USBTMC_REQUEST_REN_CONTROL returned %x\n", lnk->buffer[0]);
  rv = -EPERM;
  goto err_out;
 }

 return 0; 

err_out:
 return rv;
}


//if USB488 subclass, 1st byte should be interpreted:
//1xxx xxxx   subclass, xxx=tag from read status byte, 01 if SRQ
//01xx xxxx   vendor specific
//00xx xxxx   future use
//This function returns whatever came through the interrupt endpoint, regardless of class etc 
int usbtmc_poll_stb(usbtmc_device_data *lnk,int timeout) {
// char buffer[USBTMC_SIZE_IOBUFFER];
 int rv = 0;

 rv = usb_interrupt_read(lnk->handle, lnk->int_in, (char *)lnk->buffer, 2, timeout);
 
 if ((rv < 0) && (rv!=-110)) {	//exclude timeouts
  fprintf(stderr, "usb_interrupt_read returned %d, %s\n", rv,usb_strerror());
  goto err_out;
 }

 /*D*///fprintf(stderr, "usb_interrupt_read returned 0x%x 0x%x\n", lnk->buffer[0],lnk->buffer[1]);

 return lnk->buffer[0]*256+lnk->buffer[1]; 

err_out:
 return rv;
}







/*
int usbtmc_abort(usbtmc_device_data *lnk) { 
int usbtmc_enable_srq(usbtmc_device_data *lnk,int enable, unsigned char * handle, int len) {
*/


//Higher level functions
int usbtmc_printf(usbtmc_device_data *lnk, char * fmt, ...) {
 int n;
 va_list ap;
 va_start(ap,fmt);
 n=vsnprintf((char*)(lnk->buffer+12),lnk->bufsize,fmt,ap);
 va_end(ap);
 if (n==(int)lnk->bufsize) {
  lnk->err=129;
  return -1;
 }
 return usbtmc_write_internal(lnk,n,1);
}


//returns the number of bytes _read from the link_, regardless of error
unsigned int usbtmc_recv_file(usbtmc_device_data *lnk,int fd,unsigned int len,int *reason) {
 int i,j,k=0; 
 unsigned int leno=len;
 unsigned char *p;
 do {
  if (len>(unsigned int)lnk->bufsize) {
   if ((i=usbtmc_read_internal(lnk,lnk->bufsize,&j))<0) return leno-len;
   j&=~1;
  } else if ((i=usbtmc_read_internal(lnk,len,&j))<0) return leno-len;
   
  len-=i;

//WARNING: this was a bug, may not have been fixed in all copies of the vxi11 code    
//(is it wrong to suspect that not all gets written?)
  p=lnk->buffer+12;//skip header
  while(i) { k=write(fd,p,i); if (k<0) break; i-=k; p+=k; }
  
  if (k<0) {
   lnk->err=130;
   return leno-len;
  }
 } while (!j);
 if (reason) *reason=j;
 return leno-len; 
}

//returns the number of bytes actually written to the link, regardless of error
unsigned int usbtmc_send_file(usbtmc_device_data *lnk,int fd,unsigned int len,int end) {
 int i,k; 
 unsigned int leno=len;
 while (len) {
  if (len>(unsigned int)lnk->max_send) {
   k=read(fd,lnk->buffer+12,lnk->max_send);
   if (k<0) {
    lnk->err=131;
    return leno-len;
   }
   if (!k) {
    lnk->err=132;
    return leno-len; 
   }
   if ((i=usbtmc_write_internal(lnk,k,0))<0) return leno-len;  
  } else {
   k=read(fd,lnk->buffer+12,len);
   if (k<0) {
    lnk->err=131;
    return leno-len;
   }
   if (!k) {
    lnk->err=132;
    return leno-len; 
   }
   if ((i=usbtmc_write_internal(lnk,k,end))<0) return leno-len;  
  }
  len-=i;
 }
 return leno-len; 
}

int usbtmc_read_binary(usbtmc_device_data *lnk,char * data,int len) {
 int i,j;
 char c;
 unsigned int leno=len;
 unsigned char *p;

 do usbtmc_read(lnk,(unsigned char*)&c,1,0); while (c!='#');
  
 i=usbtmc_read(lnk,(unsigned char *)&c,1,0); 
 usbtmc_read_internal(lnk,c-48,0);
 p=lnk->buffer+12;
 p[c-48]=0;
 i=atoi((char*)p);

 do {
  if (len>(int)lnk->bufsize) {
   if ((i=usbtmc_read(lnk,(unsigned char*)data,lnk->bufsize,&j))<0) return leno-len;
   j&=~1;
  } else if ((i=usbtmc_read(lnk,(unsigned char*)data,len,&j))<0) return leno-len;
  
  data+=i; 
  len-=i;
 } while (!j);

 if (((i=usbtmc_read(lnk,(unsigned char*)&c,1,&j))<1) || (i!=1) || (!(j&4))) return -1;

 return 0;
}

//some instruments (eg. the zvr) doesn't like this one (both reading the header byte by byte, and reading the final byte separately seems unreliable)
int usbtmc_recv_binary_file(usbtmc_device_data *lnk,char * name) {
 int i,fdo,j,k;
 char c;
 unsigned char *p;
 
 fdo=open(name,O_WRONLY|O_TRUNC|O_CREAT,0644);

 do usbtmc_read(lnk,(unsigned char*)&c,1,0); while (c!='#');
  
 i=usbtmc_read(lnk,(unsigned char*)&c,1,0); 
 usbtmc_read_internal(lnk,c-48,0);
 p=lnk->buffer+12;
 p[c-48]=0;
 i=atoi((char*)p);

 k=usbtmc_recv_file(lnk,fdo,i,&j);
 if (lnk->err)  return -1;
 
 if (((i=usbtmc_read(lnk,(unsigned char*)&c,1,&j))<1) || (i!=1) || (!(j&4))) return -1;

 close(fdo);
 return k;
}


//just a wrapper for recv_file
int usbtmc_recv_named_file(usbtmc_device_data *lnk,char * name) {
 int fdo,j,k;
 fdo=open(name,O_WRONLY|O_TRUNC|O_CREAT,0644);
 k=usbtmc_recv_file(lnk,fdo,0x7fffffff,&j);
 if (lnk->err)  return -1;
 return k;
}


//reads until it receives a END, with reduced io_timeout
unsigned int usbtmc_flush(usbtmc_device_data *lnk,int timeout) {
 int i,j;
 i=lnk->io_timeout;
 lnk->io_timeout=timeout;
 do {
   if (usbtmc_read_internal(lnk,lnk->bufsize,&j)<0) {
    lnk->io_timeout=i;
    return -1;
   } 
 } while (!(j&4));
 lnk->io_timeout=i;
 return 0; 
}

 

