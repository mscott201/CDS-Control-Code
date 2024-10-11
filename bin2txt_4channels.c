#include <stdio.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
//#include <unistd.h>
#include <stdlib.h>
#include "TTree.h"
#include "TFile.h"

#define BUFSIZE 2000000+64

int main(int argc, char ** argv) {
 uint8_t buf[BUFSIZE],buf2[32];
 uint8_t buf_chan2[BUFSIZE],buf2_chan2[32];
 uint8_t buf_chan3[BUFSIZE],buf2_chan3[32];
 uint8_t buf_chan4[BUFSIZE],buf2_chan4[32];
 int fd,l,l2,l3,l4, k;
 int fd2,l_chan2,l2_chan2,l3_chan2,m;
 int fd3,l_chan3,l2_chan3,l3_chan3,n;
 int fd4,l_chan4,l2_chan4,l3_chan4,o;
 double d,yref,yinc,yori, pe;
 double d_chan2,y2ref,y2inc,y2ori, pe_chan2;
 double d_chan3,y3ref,y3inc,y3ori, pe_chan3;
 double d_chan4,y4ref,y4inc,y4ori, pe_chan4;

 yinc=atof(argv[5]);
 yori=atof(argv[6]);
 yref=atof(argv[7]);

 y2inc=atof(argv[8]);
 y2ori=atof(argv[9]);
 y2ref=atof(argv[10]);
 
 y3inc=atof(argv[11]);
 y3ori=atof(argv[12]);
 y3ref=atof(argv[13]);

 y4inc=atof(argv[14]);
 y4ori=atof(argv[15]);
 y4ref=atof(argv[16]);
 //yinc = 0.0015625;
 //yori = -0.14575;
 //yref = 128;
 
 fd=open(argv[1],O_RDONLY);
 fd2=open(argv[2],O_RDONLY);
 fd3=open(argv[3],O_RDONLY);
 fd4=open(argv[4],O_RDONLY);
 
 
 l=read(fd,buf,BUFSIZE);
 l_chan2=read(fd2,buf_chan2,BUFSIZE);
 l_chan3=read(fd3,buf_chan3,BUFSIZE);
 l_chan4=read(fd4,buf_chan4,BUFSIZE);
 
 l2=buf[1]-48;
 memcpy(buf2,buf+2,l2);
 buf2[l2]=0;
 l3=atoi(buf2);

 l2_chan2=buf_chan2[1]-48;
 memcpy(buf2_chan2,buf_chan2+2,l2_chan2);
 buf2_chan2[l2_chan2]=0;
 l3_chan2=atoi(buf2_chan2);
 
 l2_chan3=buf_chan3[1]-48;
 memcpy(buf2_chan3,buf_chan3+2,l2_chan3);
 buf2_chan3[l2_chan3]=0;
 l3_chan3=atoi(buf2_chan3);

 l2_chan4=buf_chan4[1]-48;
 memcpy(buf2_chan4,buf_chan4+2,l2_chan4);
 buf2_chan4[l2_chan4]=0;
 l3_chan4=atoi(buf2_chan4);
 //l3 = (int)(sizeof(buf2)/sizeof(uint8_t));
 //l3 = 1000000;
 
 int adcCounts = 0;
 int adcCounts_chan2 = 0;
 int adcCounts_chan3 = 0;
 int adcCounts_chan4 = 0;
 TFile *outFile = new TFile("/home/a/WCTE540/Bsci_Project_T1_2023/out.root","RECREATE");
 TTree *PMT1data  = new TTree("PMT1data","PMT1data");
 PMT1data->Branch("volts"  ,&d        ,"volts/D");
 PMT1data->Branch("pe"     ,&pe       ,"pe/D");
 PMT1data->Branch("adc_cnt",&adcCounts,"adc_cnt/I");
 PMT1data->Branch("ref_cnt",&yref     ,"ref_cnt/D");
 PMT1data->Branch("inc_v"  ,&yinc     ,"inc_v/D");
 PMT1data->Branch("ori_v"  ,&yori     ,"ori_v/D");

 TTree *PMT2data  = new TTree("PMT2data","PMT2data");
 PMT2data->Branch("volts"  ,&d_chan2        ,"volts/D");
 PMT2data->Branch("pe"     ,&pe_chan2       ,"pe/D");
 PMT2data->Branch("adc_cnt",&adcCounts_chan2,"adc_cnt/I");
 PMT2data->Branch("ref_cnt",&y2ref          ,"ref_cnt/D");
 PMT2data->Branch("inc_v"  ,&y2inc          ,"inc_v/D");
 PMT2data->Branch("ori_v"  ,&y2ori          ,"ori_v/D");
 
 TTree *PMT3data  = new TTree("PMT3data","PMT3data");
 PMT3data->Branch("volts"  ,&d_chan3        ,"volts/D");
 PMT3data->Branch("pe"     ,&pe_chan3       ,"pe/D");
 PMT3data->Branch("adc_cnt",&adcCounts_chan3,"adc_cnt/I");
 PMT3data->Branch("ref_cnt",&y3ref          ,"ref_cnt/D");
 PMT3data->Branch("inc_v"  ,&y3inc          ,"inc_v/D");
 PMT3data->Branch("ori_v"  ,&y3ori          ,"ori_v/D");

 TTree *PMT4data  = new TTree("PMT4data","PMT4data");
 PMT4data->Branch("volts"  ,&d_chan4        ,"volts/D");
 PMT4data->Branch("pe"     ,&pe_chan4       ,"pe/D");
 PMT4data->Branch("adc_cnt",&adcCounts_chan4,"adc_cnt/I");
 PMT4data->Branch("ref_cnt",&y4ref          ,"ref_cnt/D");
 PMT4data->Branch("inc_v"  ,&y4inc          ,"inc_v/D");
 PMT4data->Branch("ori_v"  ,&y4ori          ,"ori_v/D");

// fprintf(stderr,"%d %d %d _%s_\n",l,l2,l3,buf2);
 for(k=0;k<l3;k++) {
   //   printf("%d\n",buf[2+l2+k]);
   d=buf[2+l2+k];
   adcCounts = d;
   d=-((d-yref)*yinc+yori);
   pe=d * 0.8e-9/(1.33e6*50*1.6e-19); //divide by the gain to get the number of p.e.
   //printf("%.6f %.6f %.6f %.6f %.6f\n",d,(float)adcCounts,yref,yinc,yori);
   //if(k%2 == 0) 
   PMT1data->Fill();	  
 }

 for(m=0;m<l3_chan2;m++) {
   //   printf("%d\n",buf[2+l2+k]);
   d_chan2=buf_chan2[2+l2_chan2+m];
   adcCounts_chan2 = d_chan2;
   d_chan2=-((d_chan2-y2ref)*y2inc+y2ori);
   pe_chan2=d_chan2 * 0.8e-9/(1.02e6*50*1.6e-19); //divide by the gain to get the number of p.e.
   //if(m%2 == 0) 
   PMT2data->Fill();	  
 }

 for(n=0;n<l3_chan3;n++) {
   //   printf("%d\n",buf[2+l2+k]);
   d_chan3=buf_chan3[2+l2_chan3+n];
   adcCounts_chan3 = d_chan3;
   d_chan3=-((d_chan3-y3ref)*y3inc+y3ori);
   pe_chan3=d_chan3* 0.8e-9/(7.68e5*50*1.6e-19); //divide by the gain etc to get the number of p.e.
   //printf("%.6f %.6f %.6f %.6f %.6f\n",d_chan3,(float)adcCounts_chan3,y3ref,y3inc,y3ori);
   //if(n%2 == 0) 
   PMT3data->Fill();	  
 }
 
 for(o=0;o<l3_chan4;o++) {
   //   printf("%d\n",buf[2+l2+k]);
   d_chan4=buf_chan4[2+l2_chan4+o];
   adcCounts_chan4 = d_chan4;
   d_chan4=-((d_chan4-y4ref)*y4inc+y4ori);
   pe_chan4=d_chan4* 0.8e-9/(7.68e5*50*1.6e-19); //divide by the gain etc to get the number of p.e.
   //printf("%.6f %.6f %.6f %.6f %.6f\n",d_chan3,(float)adcCounts_chan3,y3ref,y3inc,y3ori);
   //if(n%2 == 0) 
   PMT4data->Fill();	  
 }

 outFile->cd();
 PMT1data->Write();
 PMT2data->Write();
 PMT3data->Write();
 PMT4data->Write();
 outFile->Close();
 close(fd);
}
