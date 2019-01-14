#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include "componentserver.h"
#include "xmlio.h"

extern struct {
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;
extern double visionpar[10];
extern double laserpar[10];

void serverconnect(componentservertype *s){
  char buf[256];
  int len;
  s->serv_adr.sin_family = AF_INET;
  s->serv_adr.sin_port= htons(s->port);
  s->serv_adr.sin_addr.s_addr = inet_addr(s->host);
  printf("port %d host %s \n",s->port,s->host);
  if ((s->connected=(connect(s->sockfd, (struct sockaddr *) &s->serv_adr, sizeof(s->serv_adr))) >-1)){
    printf(" connected to %s  \n",s->name);
    len=sprintf(buf,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    send(s->sockfd,buf,len,0);
    len=sprintf(buf,"mrc version=\"1.00\" >\n");
    send(s->sockfd,buf,len,0);
    if (fcntl(s->sockfd,F_SETFL,O_NONBLOCK) == -1) {
          fprintf(stderr,"startserver: Unable to set flag O_NONBLOCK on %s fd \n",s->name);
    }
  }
  else{
      printf("Not connected to %s  %d \n",s->name,s->connected);   
  }
}

void xml_proc(struct xml_in *x)
{
   double a;
   while (1) {
      switch (xml_in_nibble(x)) {
      case XML_IN_NONE:
         return;
      case XML_IN_TAG_START:
#if (0)
         {
            int i;
            printf("start tag: %s, %d attributes\n", x->a, x->n);
            for (i = 0; i < x->n; i++) {
               printf("  %s    %s  \n", x->attr[i].name, x->attr[i].value);
            }
         }
#endif
         if (strcmp("gmk", x->a) == 0) {
            printf("  %s    %s  \n", x->attr[0].name, x->attr[0].value);
            if (getdouble(&a, "id", x)) {
               gmk.id = a;
               printf("id= %f\n", gmk.id);
            }
	    if (getdouble(&a, "crcOK", x)) {
               gmk.crc = a;
               printf("crc= %f\n", gmk.crc);
            }
         }
         if (strcmp("pos3d", x->a) == 0) {
            (getdouble(&gmk.x, "x", x));
            (getdouble(&gmk.y, "y", x));
            (getdouble(&gmk.z, "z", x));
         }
         if (strcmp("rot3d", x->a) == 0) {
            (getdouble(&gmk.omega, "Omega", x));
            (getdouble(&gmk.phi, "Phi", x));
            (getdouble(&gmk.kappa, "Kappa", x));
         }
         if (strcmp("vision", x->a) == 0) {
            int i, ix;
            for (i = 0; i < x->n; i++) {
               ix = atoi(x->attr[i].name + 3);
               if (ix > -1 && ix < 10)
                  visionpar[ix] = atof(x->attr[i].value);
            }

         }

         break;
      case XML_IN_TAG_END:
         //printf("end tag: %s\n", x->a);
         break;
      case XML_IN_TEXT:
         //printf("text: %d bytes\n  \"", x->n);
         //fwrite(x->a, 1, x->n, stdout);
         //printf("\"\n");
         break;
      }
   }
}

void xml_proca(struct xml_in *x){

 while(1){
 switch (xml_in_nibble(x)) {
    case XML_IN_NONE:
      return;
    case XML_IN_TAG_START:
    #if (0)
    {int i;
    double a;
      printf("start tag: %s, %d attributes\n", x->a, x->n);
      for(i=0;i<x->n;i++){
        printf("  %s    %s  \n",x->attr[i].name,x->attr[i].value);
      }
    }
    #endif
       if (strcmp("laser",x->a)==0){
       int i,ix;
       for (i=0;i< x->n;i++){
         ix=atoi(x->attr[i].name+1);
	 if (ix >-1 && ix < 10)
	   laserpar[ix]=atof(x->attr[i].value);
       }
       
     }
   
    break;
    case XML_IN_TAG_END:
      //printf("end tag: %s\n", x->a);
      break;
    case XML_IN_TEXT:
      //printf("text: %d bytes\n  \"", x->n);
      //fwrite(x->a, 1, x->n, stdout);
      //printf("\"\n");
      break;
    }
  } 
}   
