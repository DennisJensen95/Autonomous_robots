/*
 * An example SMR program.
 * Modifcation
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

#define Eb 1
#define Ed 1
#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.26*Eb	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	24902
#define START_POS_X	0
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

    // Crossing a line
    int crossing_line;

    // Which linesensor is activated
    double k_line_sensor[7];

    // Center of mass
    double co_mass;

    // Speed step for specific acceleration
    double speed_step;
    double vmax;
    double vmax_angle;

    // Regulators_values
    double reg_move_fwd;
    double reg_fwl_line;
    double inte_fwl_line;
    double inte_move_fwd;
    double last_error_fwl;

    // Last iteration linesensorvalues
    double last_itera_linesensor[7];

}motiontype;

enum {mot_stop=1,mot_move,mot_turn,mot_fwl};

void update_motcon(motiontype *p, odotype *odo);

int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);
int fwl_mid(double dist, double speed,int time);


/********************************************
* Calibrations
*/

void norm_linesensor(symTableElement *linesensor, double *norm_values, int black, int white, char *line_state);
double co_mass(double *calib, char *line_state);
int crossing_line(double *calib, double threshold_crossing_limit);

/********************************************
* Initializations
*/

typedef struct{
    int state, oldstate;
    int time;
}smtype;

void sm_update(smtype *p, motiontype *mot, odotype *c);
void init_last_itera_zero(double *line_sensor_values);

/********************************************
* Regulators
*/

double get_control_fwd(motiontype *mot, odotype *odo, smtype *sm);
double get_control_fwl(motiontype *mot, smtype *sm);

/********************************************
* Motion
*/

typedef struct{
    double dist;
}motion;


// SMR input/output data

symTableElement *inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

motion moving;
odotype odo;
smtype mission;
motiontype mot;

/********************************************
* Missions
*/

enum {ms_init,ms_fwd,ms_turn,ms_end,ms_fwl, ms_turn_180, ms_find_gate};



int main()
{
    // Calibration for robot 6
//    int running, n=0, arg, time=0, black=54, white=75;
    int running, n=0, arg, time=0, black=85, white=255, crossing_lines = 1;
    double angle=0, speed=0.4, acceleration=0.5, angular_acceleration=1, threshold_crossing_limit = 0.2;
    int log_laser = 1, log_motion = 0, log_linesensor = 0, stop_at_line = 0;
    char line_state[2] = "bm";
    moving.dist = 4;

    // Initialize integrator value to zero
    mot.inte_fwl_line = 0;
    mot.last_error_fwl = 0;

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
    odo.cr=DELTA_M*Ed;
    odo.cl=DELTA_M;
    odo.left_enc=lenc->data[0];
    odo.right_enc=renc->data[0];
    reset_odo(&odo);
    printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
    mot.w=odo.w;
    running=1;
    mission.state=ms_init;
    mission.oldstate=-1;

    /************************
    * Making sure data is logged if wanted.
    */

    FILE *fp_mot, *fp_las, *fp_line;
    if (log_motion == 1) {
        fp_mot = fopen("mot_log.dat", "w");
    }

    if (log_laser == 1) {
        fp_las = fopen("laser_log.dat", "w");
    }

    if (log_linesensor == 1) {
        fp_line = fopen("linesensor_log.dat", "w");
    }
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
        norm_linesensor(linesensor, norm_linesensor_values, black, white, line_state);

        // Find center of mass
        mot.co_mass = co_mass(norm_linesensor_values, line_state);

        // Realise if vechicle crosses a line
        mot.crossing_line = crossing_line(norm_linesensor_values, threshold_crossing_limit);

        if (stop_at_line == 1 && mot.crossing_line == 1) {
            crossing_lines += 1;
            mission.state = ms_end;
        } else if (stop_at_line == 0 && mot.crossing_line == 1 && crossing_lines == 0) {
            crossing_lines += 1;
            mot.dist = 0.10;
            mission.state = ms_fwd;
        } else if (stop_at_line == 0 && mot.crossing_line == 1 && crossing_lines == 1) {
            crossing_lines += 1;
            mot.dist = 5;
            mission.state = ms_find_gate;
        } else if (mission.time >= 200) {
            mission.state = ms_end;
        }

        update_odo(&odo);

        /****************************************
        / mission statemachine
        */
        sm_update(&mission, &mot, &odo);
        switch (mission.state) {
            case ms_init:
                n=4; angle=90.0/180*M_PI;
                mission.state = ms_fwl;
                break;

            case ms_fwd:
                if (fwd(moving.dist, mot.speed_step, mission.time))  mission.state=ms_fwl;
                break;

            case ms_turn:
                if (n==0)
                    mission.state=ms_end;

                if (turn(angle, mot.speed_step, mission.time)){
                    n=n-1;
                    if (n==0)
                        mission.state=ms_end;
                    else
                        mission.state=ms_fwd;
                }
                break;

            case ms_fwl:
                if (fwl_mid(moving.dist,speed,mission.time))  mission.state=ms_end;
                break;

            case ms_end:
                mot.cmd=mot_stop;
                running=0;
                break;

            case ms_turn_180:
                angle = 180/180*M_PI;
                if (turn(angle, mot.speed_step, mission.time)) mission.state=ms_fwl;
                break;

            case ms_find_gate:
                mot.cmd = mot_stop;
                break;



        }
        /*  end of mission  */

        mot.left_pos=odo.left_pos;
        mot.right_pos=odo.right_pos;

        // Updating motion values
        double d = mot.dist - fabs((odo.right_pos_dec+odo.left_pos_dec)/2);
        mot.vmax = sqrt(2*acceleration*d);
        double angle_left = fabs(mot.angle - odo.angle_pos_mission);
        mot.vmax_angle = odo.w/2*sqrt(2*angular_acceleration*angle_left);

        mot.reg_move_fwd = get_control_fwd(&mot, &odo, &mission);
        mot.reg_fwl_line = get_control_fwl(&mot, &mission);

        update_motcon(&mot, &odo);

        // Incrementing speed to achieve specific acceleration 3.4
        if (mot.speed_step < speed) {
            mot.speed_step += acceleration/100;
        } else {
            mot.speed_step = speed;
        }

        speedl->data[0]=100*mot.motorspeed_l;
        speedl->updated=1;
        speedr->data[0]=100*mot.motorspeed_r;
        speedr->updated=1;

        if (time  % 100 ==0) {
            time++;
        }
        printf(" laser %f \n",laserpar[3]);

        if (log_motion == 1) {
            // Making motion data
            fprintf(fp_mot, "%d %f %f %f %f %f %f\n", mission.time, odo.x_pos, odo.y_pos, odo.angle_pos,
                    mot.motorspeed_r, mot.motorspeed_l, mot.speed_step);
        }

        if (log_laser == 1) {
            // Making laser data
            fprintf(fp_las, "%f %f %f %f %f %f %f %f %f\n", laserpar[0], laserpar[1], laserpar[2], laserpar[3],
                    laserpar[4], laserpar[5], laserpar[6], laserpar[7], laserpar[8]);
        }

        if (log_linesensor == 1) {
            //Making linesensor data
            fprintf(fp_line, "%d %d %d %d %d %d %d %d\n", linesensor->data[0], linesensor->data[1], linesensor->data[2],
            linesensor->data[3], linesensor->data[4], linesensor->data[5], linesensor->data[6], linesensor->data[7]);
        }





/* stop if keyboard is activated
*
*/
        ioctl(0, FIONREAD, &arg);
        if (arg!=0)  running=0;

    }/* end of main control loop */

    if (log_motion == 1) {
        fclose(fp_mot);
    }

    if (log_laser == 1) {
        fclose(fp_las);
    }

    if (log_linesensor == 1) {
        fclose(fp_line);
    }


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

void update_motcon(motiontype *p, odotype *odo){
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
            else if (p->speed_step > p->vmax) {
                p->motorspeed_l=p->vmax + p->reg_move_fwd;
                p->motorspeed_r=p->vmax - p->reg_move_fwd;
            }
                // Moving backwards with deceleration.
            else if (p->speed_step < 0) {
                if (fabs(mot.speed_step) > p->vmax) {
                    p->motorspeed_l=-p->vmax + p->reg_move_fwd;
                    p->motorspeed_r=-p->vmax - p->reg_move_fwd;
                } else {
                    p->motorspeed_l=p->speed_step + p->reg_move_fwd;
                    p->motorspeed_r=p->speed_step - p->reg_move_fwd;
                }
            }
            else {
                p->motorspeed_l=p->speed_step + p->reg_move_fwd;
                p->motorspeed_r=p->speed_step - p->reg_move_fwd;
            }
            break;

        case mot_turn:
            if (p->angle>0){
                // Divided the angle with 2 for accounting for both wheels
                printf("%f, %f\n", odo->angle_pos_mission, p->angle);
                if (odo->angle_pos_mission <= p->angle) {
                    if (p->speed_step > fabs(p->vmax_angle)) {
                        if (p->vmax_angle <= 0.01) {
                            p->vmax_angle = 0.01;
                            p->motorspeed_r = p->vmax_angle;
                            p->motorspeed_l = -p->vmax_angle;
                        } else {
                            p->motorspeed_r = p->vmax_angle;
                            p->motorspeed_l = -p->vmax_angle;
                        }

                    } else {
                        p->motorspeed_r=p->speed_step;
                        p->motorspeed_l=-p->speed_step;
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
                if (odo->angle_pos_mission >= p->angle) {
                    if (p->speed_step > p->vmax_angle) {
                        if (p->vmax_angle <= 0.05) {
                            p->vmax_angle = 0.05;
                            p->motorspeed_l = p->vmax_angle;
                            p->motorspeed_r = -p->vmax_angle;
                        } else {
                            p->motorspeed_l = p->vmax_angle;
                            p->motorspeed_r = -p->vmax_angle;
                        }
                    } else {
                        p->motorspeed_l = p->speed_step;
                        p->motorspeed_r = -p->speed_step;

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
                p->motorspeed_r = p->speed_step - p->reg_fwl_line;
                p->motorspeed_l = p->speed_step + p->reg_fwl_line;
            }
            break;

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

int fwl_mid(double dist, double speed, int time){
    if (time==0){
        mot.cmd=mot_fwl;
        mot.speedcmd=speed;
        mot.dist=dist;
        return 0;
    }
    else
        return mot.finished;
}

/****************************************
 / Updating functions
*/

void sm_update(smtype *p, motiontype *mot, odotype *c){
    if (p->state!=p->oldstate){
        mot->speed_step=0;
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

/****************************************
 / Calibration functions
*/

void norm_linesensor(symTableElement *linesensor, double  *norm_values, int black, int white, char *line_state) {
    int i;
    for (i = 0; i < 7; i++) {
        if (linesensor->data[i] < black && line_state[0] == 'b') {
            norm_values[i] = 0;
        }
        else if (linesensor->data[i] < black && line_state[0] == 'w') {
            norm_values[i] = 1;
        }
        else if (linesensor->data[i] > white && line_state[0] == 'b') {
            norm_values[i] = 1;
        }
        else if (linesensor->data[i] > white && line_state[0] == 'w') {
            norm_values[i] = 0;
        }
        else {
            if (line_state[0] == 'b') {
                norm_values[i] =  (double) (linesensor->data[i] - black)/(white-black);
            }
            else {
                norm_values[i] = 1 - (double) (linesensor->data[i] - black)/(white-black);
            }
        }
    }
}

double co_mass(double *calib, char *line_state) {
    double numerator = 0, denominator = 0, center = 3.0;
    int i;
    for (i = 0; i < 8; i++) {
        numerator += calib[i] * i;
        denominator += calib[i];
    }
    if (line_state[1] == 'm') {
        center = 3.0;
    } else if (line_state[1] == 'l') {
        center = 3.25;
    } else if (line_state[1] == 'r') {
        center = 2.75;
    }

    return (numerator/denominator) - center;
}

/****************************************
 / Regulator values functions
*/

double get_control_fwd(motiontype *mot, odotype *odo, smtype *sm) {
    double prop = 0.05 * odo->angle_pos_mission;
    return prop + mot->inte_move_fwd;
}

double get_control_fwl(motiontype *mot, smtype *sm) {
    double prop = 0.2 * mot->co_mass;
    mot->inte_fwl_line += 0.03 * mot->co_mass*0.01;
    double derivative = 0.00005 * (mot->co_mass - mot->last_error_fwl)/0.01;
    mot->last_error_fwl = mot->co_mass;

//    printf("prop: %f, integral: %f, derivative: %f, co_mass: %f\n", prop, mot->inte_fwl_line, derivative, mot->co_mass);

    return prop + mot->inte_fwl_line + derivative;
}

/****************************************
 / Detecting functions
*/

int crossing_line(double *calib, double threshold_crossing_limit) {
    unsigned int i = 0, line = 0;
    for (i = 0; i < 8; i++) {
        if (calib[i] >= 0-threshold_crossing_limit && calib[i] <= 0+threshold_crossing_limit) {
            line = 1;
        } else {
            line = 0;
            break;
        }
    }
    return line;
}
