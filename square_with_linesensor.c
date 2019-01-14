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
double norm_linesensor_values[8];

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
    double right_pos_dec, left_pos_dec;
    // internal variables
    int left_enc_old, right_enc_old;
    // Position x and y
    double x_pos;
    double y_pos;
    double angle_pos;
    // Angle resetting for every mission
    double angle_pos_mission;
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
    // Which linesensor is activated
    int lowest_linesensor_index;
    double k_line_sensor[7];
    // Center of mass
    double co_mass;
}motiontype;

enum {mot_stop=1,mot_move,mot_turn,mot_fwl};

void update_motcon(motiontype *p, double speed_step, double vmax, double vmax_angle);


int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);
void norm_linesensor(symTableElement *linesensor, double *norm_values, int black, int white);
int min_max(double *calib, int w_line);
int fwl(double dist, double speed,int time);
double co_mass(double *calib);



typedef struct{
    int state,oldstate;
    int time;
}smtype;

void sm_update(smtype *p, double *speed_step, odotype *c);

// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {ms_init,ms_fwd,ms_turn,ms_end,ms_fwl};

int main()
{
    int running,n=0,arg,time=0,black=85, white=255;
    double dist=0,angle=0,speed=0.3,acceleration=0.5,angular_acceleration=1;
    int w_line = 0;

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
            len=sprintf(buf,"scanpush cmd='zoneobst'\n");
            send(lmssrv.sockfd,buf,len,0);
        }

    }

    /* Read sensors and zero our position.
     */
    rhdSync();

    odo.w = WHEEL_SEPARATION;
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

    FILE *fp, *fp_las, *fp_line;
    fp = fopen("mot_log.dat", "w");
    fp_las = fopen("laser_log.dat", "w");
    fp_line = fopen("linesensor_log.dat", "w");

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

        // Calibrate linesensor values
        norm_linesensor(linesensor, norm_linesensor_values, black, white);

        // Find center of mass
        mot.co_mass = co_mass(norm_linesensor_values);

        // Find the lowest or highest calibration value for the linesensors
        mot.lowest_linesensor_index = min_max(norm_linesensor_values, w_line);
        printf("%d\n", mot.lowest_linesensor_index);

        update_odo(&odo);

        /****************************************
        / mission statemachine
        */
        sm_update(&mission, &speed_step, &odo);
        switch (mission.state) {
            case ms_init:
                n=0; dist=2;angle=90.0/180*M_PI;
                mission.state = ms_fwl;
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

            case ms_fwl:
                if (fwl(dist,speed,mission.time))  mission.state=ms_end;
                break;

            case ms_end:
                mot.cmd=mot_stop;
                running=0;
                break;


        }
        /*  end of mission  */

        mot.left_pos=odo.left_pos;
        mot.right_pos=odo.right_pos;

        double d = mot.dist - fabs((odo.right_pos_dec+odo.left_pos_dec)/2);
        double vmax = sqrt(2*acceleration*d);
        double angle_left = fabs(mot.angle - odo.angle_pos_mission);
        double max_angular_velocity = sqrt(2*angular_acceleration*angle_left);
        double vmax_angel_speed = odo.w/2*max_angular_velocity;

        update_motcon(&mot, speed_step, vmax, vmax_angel_speed);

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
        if (time  % 100 ==0) {
            time++;
        }

        // Making laser data
        fprintf(fp_las, "%f %f %f %f %f %f %f %f %f\n", laserpar[0], laserpar[1], laserpar[2], laserpar[3],
                laserpar[4], laserpar[5], laserpar[6], laserpar[7], laserpar[8]);

        // Making motion data
        fprintf(fp, "%d %f %f %f %f %f %f\n", mission.time, odo.x_pos, odo.y_pos, odo.angle_pos, mot.motorspeed_r, mot.motorspeed_l, speed_step);

//        // Making linesensor data
//        fprintf(fp_line, "%d %d %d %d %d %d %d %d\n", linesensor->data[0], linesensor->data[1], linesensor->data[2],
//                linesensor->data[3], linesensor->data[4], linesensor->data[5], linesensor->data[6], linesensor->data[7]);

        fprintf(fp_line, "%f %f %f %f %f %f %f %f\n", norm_linesensor_values[0], norm_linesensor_values[1],
                norm_linesensor_values[2], norm_linesensor_values[3],
                norm_linesensor_values[4], norm_linesensor_values[5],
                norm_linesensor_values[6], norm_linesensor_values[7]);


/* stop if keyboard is activated
*
*/
        ioctl(0, FIONREAD, &arg);
        if (arg!=0)  running=0;

    }/* end of main control loop */

    fclose(fp);
    fclose(fp_las);
    fclose(fp_line);

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
    p->right_pos_dec += delta * p->cr;
    double dis_right = delta * p->cr;


    delta = p->left_enc - p->left_enc_old;
    if (delta > 0x8000) delta -= 0x10000;
    else if (delta < -0x8000) delta += 0x10000;
    p->left_enc_old = p->left_enc;
    p->left_pos += delta * p->cl;
    p->left_pos_dec += delta * p->cl;
    double dis_left = delta * p->cl;

    double dis_center = (dis_right + dis_left)/2;
    double dis_angle = (dis_right - dis_left)/p->w;

    // Odometry updates
    p->x_pos = p->x_pos + dis_center*cos(p->angle_pos);
    p->y_pos = p->y_pos + dis_center*sin(p->angle_pos);
    p->angle_pos = p->angle_pos + dis_angle;
    p->angle_pos_mission = p->angle_pos_mission + dis_angle;
}

void update_motcon(motiontype *p, double speed_step, double vmax, double vmax_angle){
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

            case mot_fwl:
                p->curcmd=mot_fwl;
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

            if (fabs((p->right_pos+p->left_pos)/2 - p->startpos) > p->dist){
                p->finished=1;
                p->motorspeed_l=0;
                p->motorspeed_r=0;
            }
            else if (speed_step > vmax) {
                p->motorspeed_l=vmax + 0.5 * odo.angle_pos_mission;
                p->motorspeed_r=vmax - 0.5 * odo.angle_pos_mission;
            }
            // Moving backwards with deceleration.
            else if (speed_step < 0) {
                if (fabs(speed_step) > vmax) {
                    p->motorspeed_l=-vmax + 0.5 * odo.angle_pos_mission;
                    p->motorspeed_r=-vmax - 0.5 * odo.angle_pos_mission;
                } else {
                    p->motorspeed_l=speed_step + 0.5 * odo.angle_pos_mission;
                    p->motorspeed_r=speed_step - 0.5 * odo.angle_pos_mission;
                }
            }
            else {
                p->motorspeed_l=speed_step + 0.5 * odo.angle_pos_mission;
                p->motorspeed_r=speed_step - 0.5 * odo.angle_pos_mission;
            }
            break;

        case mot_turn:
            if (p->angle>0){
                // Divided the angle with 2 for accounting for both wheels
                if (p->right_pos-p->startpos <= p->angle/2*p->w){
                    if (speed_step > fabs(vmax_angle)) {
                        if (vmax_angle <= 0.05) {
                            vmax_angle = 0.1;
                            p->motorspeed_r = vmax_angle;
                            p->motorspeed_l = -vmax_angle;
                        } else {
                            p->motorspeed_r = vmax_angle;
                            p->motorspeed_l = -vmax_angle;
                        }

                    } else {
                        p->motorspeed_r=speed_step;
                        p->motorspeed_l=-speed_step;
                    }
                }
                else {
                    p->motorspeed_l=0;
                    p->motorspeed_r=0;
                    p->finished=1;
                }
            }
            else {
                // Divided the angle with 2 for accounting for both wheels
                if (p->left_pos-p->startpos <= fabs(p->angle)/2*p->w){
                    if (speed_step > vmax_angle) {
                        if (vmax_angle <= 0.05) {
                            vmax_angle = 0.1;
                            p->motorspeed_l = vmax_angle;
                            p->motorspeed_r = -vmax_angle;
                        } else {
                            p->motorspeed_l = vmax_angle;
                            p->motorspeed_r = -vmax_angle;
                        }
                    } else {
                        p->motorspeed_l = speed_step;
                        p->motorspeed_r = -speed_step;

                    }
                } else {
                    p->motorspeed_r=0;
                    p->motorspeed_l=0;
                    p->finished=1;
                }
            }
            break;

        case mot_fwl:
            if (fabs((p->right_pos+p->left_pos)/2 - p->startpos) > p->dist){
                p->finished=1;
                p->motorspeed_l=0;
                p->motorspeed_r=0;
            }
            else {
                p->motorspeed_r = speed_step - 0.2 * mot.co_mass;
                p->motorspeed_l = speed_step + 0.2 * mot.co_mass;
            }
            break;

//        case mot_fwl:
//            // Turning right if the line is to the left
//            if (fabs((p->right_pos+p->left_pos)/2 - p->startpos) > p->dist){
//                p->finished=1;
//                p->motorspeed_l=0;
//                p->motorspeed_r=0;
//            } else if (p->lowest_linesensor_index <= 3) {
//                p->motorspeed_r = speed_step;
//                p->motorspeed_l = speed_step + 1 * p->k_line_sensor[p->lowest_linesensor_index];
//            } else {
//                p->motorspeed_r = speed_step + 1 * p->k_line_sensor[p->lowest_linesensor_index];;
//                p->motorspeed_l = speed_step;
//            }
//            break;
    }
}
/*
 * Driving functions
 */

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

int fwl(double dist, double speed,int time){
    if (time==0){
        mot.cmd=mot_fwl;
        mot.speedcmd=speed;
        mot.dist=dist;
        return 0;
    }
    else
        return mot.finished;
}

/*
 * Updating functions
 */

void sm_update(smtype *p, double *speed_step, odotype *c){
    if (p->state!=p->oldstate){
        *speed_step=0;
        c->left_pos_dec=0;
        c->right_pos_dec=0;
        c->angle_pos_mission=0;
        p->time=0;
        p->oldstate=p->state;
    }
    else {
        p->time++;
    }
}

/*
 * Calibration functions
 */

void norm_linesensor(symTableElement *linesensor, double  *norm_values, int black, int white) {
    int i;
    for (i = 0; i < 7; i++) {
        if (linesensor->data[i] < black) {
            norm_values[i] = 0;
        }
        else if (linesensor->data[i] > white) {
            norm_values[i] = 1;
        }
        else {
            norm_values[i] =  (double) (linesensor->data[i] - black)/(white-black);
        }
    }
}

double co_mass(double *calib) {
    double numerator = 0, denominator = 0;
    int i;
    for (i = 0; i < 8; i++) {
        numerator += calib[i] * i;
        denominator += calib[i];
    }
    return (numerator/denominator) - 3.5;
}

int min_max(double *calib_values, int w_line) {
    int i, index = 0;
    if (w_line == 1) {
        for (i = 0; i < 7; i++) {
            if (calib_values[i] > calib_values[index]) {
                index = i;
            }
        }
    } else {
        for (i = 0; i < 7; i++) {
            if (calib_values[i] < calib_values[index]) {
               index = i;
            }
        }
    }
    return index;
}

