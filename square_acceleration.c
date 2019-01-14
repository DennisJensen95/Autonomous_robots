/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
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

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv,camsrv;

 symTableElement * 
     getinputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('r'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

    symTableElement * 
     getoutputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('w'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }
/*****************************************
* odometry
*/
#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.252	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	24902
#define START_POS_X	-1.9
#define START_POS_Y	0
#define START_ANGLE	0


typedef struct{ //input signals
		int left_enc,right_enc; // encoderticks
		// parameters
		double w;	// wheel separation
		double cr,cl;   // meters per encodertick
	        //output signals
		double right_pos,left_pos;
		// internal variables
		int left_enc_old, right_enc_old;
		// Position x and y
		double x_pos;
		double y_pos;
		double angle_pos;
		} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);




/********************************************
* Motion control
*/

typedef struct{//input
                int cmd;
		int curcmd;
		double speedcmd;
		double dist;
		double angle;
		double left_pos,right_pos;
		// parameters
		double w;
		//output
		double motorspeed_l,motorspeed_r; 
		int finished;
		// internal variables
		double startpos;
	       }motiontype;
	       
enum {mot_stop=1,mot_move,mot_turn};

void update_motcon(motiontype *p, double speed_step);


int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);



typedef struct{
                int state,oldstate;
		int time;
	       }smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {ms_init,ms_fwd,ms_turn,ms_end};

int main()
{
  int running,n=0,arg,time=0;
  double dist=0,angle=0,speed=0.4;

  /* Establish connection to robot sensors and actuators.
   */
     if (rhdConnect('w',"localhost",ROBOTPORT)!='w'){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      } 
      
      printf("connected to robot \n");
      if ((inputtable=getSymbolTable('r'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      if ((outputtable=getSymbolTable('w'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      // connect to robot I/O variables
      lenc=getinputref("encl",inputtable);
      renc=getinputref("encr",inputtable);
      linesensor=getinputref("linesensor",inputtable);
      irsensor=getinputref("irsensor",inputtable);  
      
      speedl=getoutputref("speedl",outputtable);
      speedr=getoutputref("speedr",outputtable);
      resetmotorr=getoutputref("resetmotorr",outputtable);
      resetmotorl=getoutputref("resetmotorl",outputtable);
     // **************************************************
//  Camera server code initialization
//

/* Create endpoint */
   lmssrv.port=24919;
   strcpy(lmssrv.host,"127.0.0.1");
   strcpy(lmssrv.name,"laserserver");
   lmssrv.status=1;
   camsrv.port=24920;
   strcpy(camsrv.host,"127.0.0.1");
   camsrv.config=1;
   strcpy(camsrv.name,"cameraserver");
   camsrv.status=1;

   if (camsrv.config) {
      int errno = 0; 
      camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( camsrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&camsrv);

   xmldata=xml_in_init(4096,32);
   printf(" camera server xml initialized \n");

}   
 
   
   
   
// **************************************************
//  LMS server code initialization
//

/* Create endpoint */
   lmssrv.config=1;
   if (lmssrv.config) {
       char buf[256];
      int errno = 0,len; 
      lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( lmssrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&lmssrv);
   if (lmssrv.connected){
     xmllaser=xml_in_init(4096,32);
     printf(" laserserver xml initialized \n");
     len=sprintf(buf,"push  t=0.2 cmd='mrcobst width=0.4'\n");
     send(lmssrv.sockfd,buf,len,0);
   }

}   
   
 
  /* Read sensors and zero our position.
   */
  rhdSync();
  
  odo.w=0.256;
  odo.cr=DELTA_M;
  odo.cl=odo.cr;
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w=odo.w;
running=1; 
mission.state=ms_init;
mission.oldstate=-1;
//int data_time[1000];
//double data_motorspeedl[1000];
//double data_motorspeedr[1000];

FILE *fp;
fp = fopen("mot_log.dat", "w");

double speed_step = 0;
while (running){ 
   if (lmssrv.config && lmssrv.status && lmssrv.connected){
           while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
             xml_proca(xmllaser);
      }
      
      if (camsrv.config && camsrv.status && camsrv.connected){
          while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
             xml_proc(xmldata);
      }
       

  rhdSync();
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
   
  update_odo(&odo);
  
/****************************************
/ mission statemachine   
*/
   sm_update(&mission);
   switch (mission.state) {
     case ms_init:
       n=0; dist=2;angle=90.0/180*M_PI;;
       mission.state = ms_fwd;
     break;
  
     case ms_fwd:
        if (fwd(dist,speed,mission.time))  mission.state=ms_turn;
     break;
  
     case ms_turn:
        if (n==0)
	    mission.state=ms_end;
       
        if (turn(angle,0.3,mission.time)){
             n=n-1;
	 if (n==0) 
	   mission.state=ms_end;
	 else
	   mission.state=ms_fwd;
	}
     break;    
     
     case ms_end:
        mot.cmd=mot_stop;
        running=0;
        break;
   }  
/*  end of mission  */
 
  mot.left_pos=odo.left_pos;
  mot.right_pos=odo.right_pos;

  update_motcon(&mot, speed_step);

  // Incrementing speed to achieve specific acceleration 3.4
  if (speed_step < speed) {
      speed_step += 0.005;
  } else {
      speed_step = speed;
  }

  speedl->data[0]=100*mot.motorspeed_l;
  speedl->updated=1;
  speedr->data[0]=100*mot.motorspeed_r;
  speedr->updated=1;
  if (time  % 100 ==0)
    //    printf(" laser %f \n",laserpar[3]);
  time++;
   
  fprintf(fp, "%d %f %f %f %f %f\n", mission.time, odo.x_pos, odo.y_pos, odo.angle_pos, mot.motorspeed_r, mot.motorspeed_l);
  
  /*
  data_time[i] = time;
  data_motorspeedl[i] = 100*mot.motorspeed_l;
  data_motorspeedr[i] = 100*mot.motorspeed_l;
  */
  
/* stop if keyboard is activated
*
*/
  ioctl(0, FIONREAD, &arg);
  if (arg!=0)  running=0;
    
}/* end of main control loop */
  
  
  //int j;
  //for (j = 0; j < 1000; j++) {
    
    //printf("%f %f %f\n", data_time[j], data_motorspeedr[j], data_motorspeedl[j]);
  //}
  
  fclose(fp);
  
  speedl->data[0]=0;
  speedl->updated=1;
  speedr->data[0]=0;
  speedr->updated=1;
  rhdSync();
  rhdDisconnect();
  exit(0);
}


/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */


void reset_odo(odotype * p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
  p->x_pos = START_POS_X;
  p->y_pos = START_POS_Y;
  p->angle_pos = START_ANGLE;
}

void update_odo(odotype *p)
{
  int delta;

  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += delta * p->cr;
  double dis_right = delta * p->cr;
  
  
  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += delta * p->cl;
  double dis_left = delta * p->cl;
  
  double dis_center = (dis_right + dis_left)/2;
  double dis_angle = (dis_right - dis_left)/p->w;
   
  
  p->x_pos = p->x_pos + dis_center*cos(p->angle_pos);
  p->y_pos = p->y_pos + dis_center*sin(p->angle_pos);
  p->angle_pos = p->angle_pos + dis_angle;
  
}

void update_motcon(motiontype *p, double speed_step){

if (p->cmd !=0){
     
     p->finished=0;
     switch (p->cmd){
     case mot_stop:
         p->curcmd=mot_stop;
       break;
       case mot_move:
       p->startpos=(p->left_pos+p->right_pos)/2;
       p->curcmd=mot_move;
       break;
       
       case mot_turn:
         if (p->angle > 0) 
	    p->startpos=p->right_pos;
	 else
	    p->startpos=p->left_pos;
         p->curcmd=mot_turn;
       break;
       
       
     }
     
     p->cmd=0;
   }
   
   switch (p->curcmd){
     case mot_stop:
       p->motorspeed_l=0;
       p->motorspeed_r=0;
     break;
     case mot_move:
       
       if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist){
          p->finished=1;
          p->motorspeed_l=0;
          p->motorspeed_r=0;
       }
       else if {
           ((p->right_pos+p->left_pos)/2) 
       }
       else {
           p->motorspeed_l=speed_step;
           p->motorspeed_r=speed_step;
       }
     break;
     
     case mot_turn:
       if (p->angle>0){
          //p->motorspeed_l=0;
	  // Divided the angle with 2 for accounting for both wheels
	  if (p->right_pos-p->startpos < p->angle/2*p->w){
	      p->motorspeed_r=p->speedcmd;
	      // Modified setting speed on left wheel
	      p->motorspeed_l=-p->speedcmd;
	  }
	  else {	     
            p->motorspeed_r=0;
            p->finished=1;
	  }
	}
	else {
          //p->motorspeed_r=0;
	  // Divided the angle with 2 for accounting for both wheels
	  if (p->left_pos-p->startpos < fabs(p->angle)/2*p->w){
	      p->motorspeed_l=p->speedcmd;
	      // Set speed on right wheel for getting both wheels working
	      p->motorspeed_r=-p->speedcmd;
	  }
	  else {	     
            p->motorspeed_l=0;
            p->finished=1;
	  }
	}

     break;
   }   
}   


int fwd(double dist, double speed,int time){
   if (time==0){ 
     mot.cmd=mot_move;
     mot.speedcmd=speed;
     mot.dist=dist;
     return 0;
   }
   else
     return mot.finished;
}

int turn(double angle, double speed,int time){
   if (time==0){ 
     mot.cmd=mot_turn;
     mot.speedcmd=speed;
     mot.angle=angle;
     return 0;
   }
   else
     return mot.finished;
}


void sm_update(smtype *p){
  if (p->state!=p->oldstate){
       p->time=0;
       p->oldstate=p->state;
   }
   else {
     p->time++;
   }
}



