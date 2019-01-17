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

// Normalized linesensor putting them from 0-1
double norm_linesensor_values[8];
// Calibration of infared sensors (Obstacle detection or measurement in meters after calibration)
double ir_calib_sensor_values[5];

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

/********************************************
* Detectors parameters
*/

typedef struct {
    // Counting parameters
    int crossed_lines;
    int mis_state, old_mis_state;

    // Detecting forks
    int negative_fork;

    // Detect obstacles
    int obstacle_det_ir;

    // Crossing a line
    int crossing_line;

    // Line end
    int line_end;

    // Line detect
    int line_detected;
}detectors;

/********************************************
* States
*/

typedef struct {
    char line_state[2];
    double speed;
    double ir_dist_obstacle_det;
}robot_state;

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

    // Which linesensor is activated
    double k_line_sensor[7];

    // Center of mass
    double co_mass;
    double co_mass_prev;

    // Detect obstacle
    int ir_detect;

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
int fwl(double dist, double speed,int time);


/********************************************
* Calibrations
*/

void norm_linesensor(symTableElement *linesensor, double *norm_values, int black, int white, char *line_state);
double co_mass(double *calib, char *line_state, detectors *det);
int crossing_line(double *calib, double threshold_crossing_limit);
int ir_detect(double *ir_calib, double ir_dist);
void calib_ir_sensor(symTableElement *irsensor, double *ir_calib_sensor_values);
int detect_negative_fork(motiontype *mot);

/********************************************
* Initializations
*/

typedef struct{
    int state, oldstate;
    int time;
    int functions, oldfunction;
}smtype;

void sm_update(smtype *p, motiontype *mot, odotype *c, detectors *det);
void init_last_itera_zero(double *line_sensor_values);

/********************************************
* Regulators
*/

double get_control_fwd(motiontype *mot, odotype *odo, smtype *sm);
double get_control_fwl(motiontype *mot, smtype *sm);

/********************************************
* Track function
*/
void competition_track(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);
void make_square(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);

/********************************************
* Subsections of competition track
*/
void first_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);
void second_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);
void third_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);
void fourth_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);
void fifth_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);
void sixth_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);
// SMR input/output data

symTableElement *inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;


odotype odo;
smtype drive_state;
detectors det;
motiontype mot;
robot_state rstate;

/********************************************
* Missions
*/

enum {drive_init,drive_fwd,drive_turn,drive_end,drive_fwl,drive_update, drive_idle};

int main()
{
    // Calibration for robot 6
//    int running, n=0, arg, time=0, black=54, white=75;
    int running, arg, time=0, black=85, white=255;
    double acceleration=0.5, angular_acceleration=1, threshold_crossing_limit = 0.2;
    int log_laser = 1, log_motion = 0, log_linesensor = 0;

    // Motion controls
    mot.angle = -90.0/180*M_PI;

    // Case function
    drive_state.functions = 0;
    drive_state.oldfunction = 0;

    // Distance for obstacle detection
    rstate.ir_dist_obstacle_det = 0.2;

    // Motion speed
    rstate.speed = 0.4;

    // States
    rstate.line_state[0] = 'b';
    rstate.line_state[1] = 'm';

    // Initialize integrator value to zero
    mot.inte_fwl_line = 0;
    mot.last_error_fwl = 0;

    // Detectors states
    det.crossed_lines = 0;
    det.mis_state = 4;
    det.old_mis_state = 0;

    // Initialize center of mass previous value
    mot.co_mass_prev = 0;

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
    drive_state.state=drive_init;
    drive_state.oldstate=-1;

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
        norm_linesensor(linesensor, norm_linesensor_values, black, white, rstate.line_state);

        // Find center of mass
        mot.co_mass = co_mass(norm_linesensor_values, rstate.line_state, &det);
//        printf("%f", mot.co_mass);

        // Realise if vechicle crosses a line
        det.crossing_line = crossing_line(norm_linesensor_values, threshold_crossing_limit);

        // Update obstacle detection
        calib_ir_sensor(irsensor, ir_calib_sensor_values);
        det.obstacle_det_ir = ir_detect(ir_calib_sensor_values, rstate.ir_dist_obstacle_det);

        det.negative_fork = detect_negative_fork(&mot);

        /******************************
        /  Update odemtry parameters
        */
        update_odo(&odo);

        /******************************
        /  THE MISSION BEING RUNNED!
        */
        competition_track(&mot, &odo, &drive_state, &det, &rstate);
//        make_square(&mot, &odo, &drive_state, &det, &rstate);
//        following_white_line(&mot, &odo, &drive_state, &det, &rstate);

        /****************************************
        / Update mission
        */

        sm_update(&drive_state, &mot, &odo, &det);

        switch (drive_state.state) {
            case drive_init:
                drive_state.state = drive_fwd;
                break;

            case drive_fwd:
                if (fwd(mot.dist, mot.speed_step, drive_state.time)) {
                    drive_state.functions += 1;
                }
                break;

            case drive_turn:
                if (turn(mot.angle, mot.speed_step, drive_state.time)){
                    drive_state.functions += 1;
                }
                break;

            case drive_fwl:
                if (fwl(mot.dist,mot.speed_step,drive_state.time)) {
                    drive_state.functions += 1;

                }
                break;

            case drive_end:
                mot.cmd=mot_stop;
                running=0;
                break;

            case drive_idle:
                mot.cmd=mot_stop;
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

        mot.reg_move_fwd = get_control_fwd(&mot, &odo, &drive_state);
        mot.reg_fwl_line = get_control_fwl(&mot, &drive_state);

        // Incrementing speed to achieve specific acceleration 3.4
        if (mot.speed_step < rstate.speed) {
            mot.speed_step += acceleration/100;
        } else {
            mot.speed_step = rstate.speed;
        }

        update_motcon(&mot, &odo);

        speedl->data[0]=100*mot.motorspeed_l;
        speedl->updated=1;
        speedr->data[0]=100*mot.motorspeed_r;
        speedr->updated=1;

        if (time  % 100 ==0) {
            time++;
        }

        if (log_motion == 1) {
            // Making motion data
            fprintf(fp_mot, "%d %f %f %f %f %f %f\n", drive_state.time, odo.x_pos, odo.y_pos, odo.angle_pos,
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

//        printf("%f %f %f %f %f %f %f %f\n", norm_linesensor_values[0], norm_linesensor_values[1], norm_linesensor_values[2],
//                norm_linesensor_values[3], norm_linesensor_values[4], norm_linesensor_values[5], norm_linesensor_values[6]
//                , norm_linesensor_values[7]);




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
                p->startpos = (p->left_pos + p->right_pos)/2;
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
//                printf("%f, %f\n", odo->angle_pos_mission, p->angle);
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
//            printf("Distance gone in following line: %f, The wanted going dist: %f\n",
//                    fabs((p->right_pos+p->left_pos)/2 - p->startpos), p->dist);
            if (fabs((p->right_pos+p->left_pos)/2 - p->startpos) > p->dist) {
//                printf("Finished fwl line\n");
                p->finished=1;
                p->motorspeed_l=0;
                p->motorspeed_r=0;
            }
            else {
//                printf("The constant added to speed depending on co_mass: %f", p->reg_fwl_line);
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

int fwl(double dist, double speed, int time){
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

void sm_update(smtype *p, motiontype *mot, odotype *c, detectors *det){
    if (p->oldfunction!=p->functions || p->oldstate!=p->state || det->old_mis_state!=det->mis_state) {
        mot->inte_fwl_line = 0;
        mot->last_error_fwl = 0;
        printf("Resets mission\n");
        mot->speed_step=0;
        c->left_pos_dec=0;
        c->right_pos_dec=0;
        c->angle_pos_mission=0;
        p->time=0;
        det->old_mis_state = det->mis_state;
        p->oldstate=p->state;
        p->oldfunction=p->functions;
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
    for (i = 0; i < 8; i++) {
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

void calib_ir_sensor(symTableElement *irsensor, double *ir_calib_sensor_values){
    int i;
    double Ka = 15.8463, Kb = 74.6344;
    for (i = 0; i < 5; i++){
        ir_calib_sensor_values[i] = Ka/(irsensor->data[i] - Kb);
    }
}

double co_mass(double *calib, char *line_state, detectors *det) {
    double numerator = 0, denominator = 0, center = 3.0;
    int i;

    if (line_state[1] == 'm') {
        center = 3.5;
    } else if (line_state[1] == 'l') {
        center = 3.8;
    } else if (line_state[1] == 'r') {
        center = 3.2;
    }

    for (i = 0; i < 8; i++) {
        numerator += calib[i] * i;
        denominator += calib[i];
    }

    printf("The denominator for co mass: %f\n",denominator);

    // To avoid nans and detect lines
    if (denominator <= 3) {
        det->crossing_line = 1;
        return 0;
    } else {
        det->crossing_line = 0;
    }

    if (denominator <= 7.5 && denominator >= 2) {
        det->line_detected = 1;
    } else {
        det->line_detected = 0;
    }

    // Detect a line end
    if (denominator >= 7.5) {
        det->line_end = 1;
        return 0;
    } else {
        det->line_end = 0;
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
    mot->inte_fwl_line += 0.04 * mot->co_mass*0.01;
    double derivative = 0.0005 * (mot->co_mass - mot->last_error_fwl)/0.01;
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

int ir_detect(double *ir_calib, double ir_dist) {
    unsigned int i, stop = 0;
    for (i = 1; i < 4; i++) {
        if (ir_calib[i] <= ir_dist) {
            stop = 1;
        } else {
            stop = 0;
            break;
        }
    }
    return stop;
}

//Detect fork with branch <90deg , assume step change in line sensor delta_com=0.6
int detect_negative_fork(motiontype *mot) {
    return (fabs(mot->co_mass) >= fabs(mot->co_mass_prev +0.6));
}

/********************************************
* Track function
*/

void competition_track(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
//    printf("%d", det->mis_state);
    // Mission 1n measure obstacle
    if (det->mis_state == 0) {
        first_mission(mot, odo, drive_state, det, rstate);
    }

    // Mission 2
    else if (det->mis_state == 1) {
        second_mission(mot, odo, drive_state, det, rstate);
    }

    // Mission 3
    else if (det->mis_state == 2) {
        third_mission(mot, odo, drive_state, det, rstate);
    }

    // Mission 4 Wall
    else if (det->mis_state == 3) {
        fourth_mission(mot, odo, drive_state, det, rstate);
    }


    // Mission 5
    else if (det->mis_state == 4) {
        fifth_mission(mot, odo, drive_state, det, rstate);
    }

    // Mission 6
    else if (det->mis_state == 5) {
        sixth_mission(mot, odo, drive_state, det, rstate);
    }

    printf("Time: (%d) || mission func incre: %d, angle: %f, drive_state: %d, angle turn: %f, negative_fork: %d, mission_state: %d, "
           "startpos: %f, distance gone: %f, motiontype dist: %f, crossing_line: %d\n", drive_state->time,
           drive_state->functions, odo->angle_pos_mission, drive_state->state, mot->angle, det->negative_fork, det->mis_state,
           mot->startpos, ((mot->right_pos + mot->left_pos)/2 - mot->startpos), mot->dist, det->crossing_line);
}

void first_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
    if (drive_state->functions == 0) {
        mot->dist = 1.45;
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

        if (det->line_end == 1) {
            drive_state->state = drive_end;
        }
    } else if (drive_state->functions == 1) {
        mot->angle = -180.0/180*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 2) {
        mot->dist = 2;
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

        if (det->negative_fork == 1) {
            drive_state->functions = 3;
        }
    } else if (drive_state->functions == 3) {
        mot->dist = 0.2;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 4) {
        mot->angle = -120.0 / 180 * M_PI;
        drive_state->state = drive_turn;
    } else {
        det->mis_state = 1;
        det->crossed_lines = 0;
        drive_state->functions = 0;
    }
}

void second_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
    if (drive_state->functions == 0) {
        mot->dist = 0.25;
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

    } else if (drive_state->functions == 1) {
        mot->dist = 0.75;
        rstate->line_state[1] = 'l';
        drive_state->state = drive_fwl;
    } else if (drive_state->functions == 2) {
        drive_state->time = 0;
        mot->dist = 2;
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

        if (det->crossing_line == 1) {
            drive_state->functions = 3;
        }
    } else if (drive_state->functions == 3) {
        mot->dist = 0.2;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 4) {
        mot->angle = -90.0/180*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 5) {
        mot->dist = 0.1;
        drive_state->state = drive_fwd;
    } else if ( drive_state->functions == 6) {
        mot->dist = 2;
        drive_state->state = drive_fwl;

        if (det->crossing_line == 1) {
            drive_state->functions = 7;
        }
    } else if (drive_state->functions == 7) {
        mot->dist = 1;
        drive_state->state = drive_fwl;

        if (det->line_end == 1) {
            drive_state->functions = 8;
        }
    } else if (drive_state->functions == 8) {
        mot->speedcmd = -0.4;
        rstate->speed = -0.4;
        mot->dist = 1.25;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 9) {
        mot->speedcmd = 0.4;
        rstate->speed = 0.4;
        mot->angle = 180.0/180.0*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 10) {
        mot->dist = 1;
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

        if (det->crossing_line == 1) {
            drive_state->functions = 11;
        }
    } else if (drive_state->functions == 11) {
        drive_state->state = drive_fwl;

        if (det->crossing_line == 1) {
            drive_state->functions = 12;
        }
    } else if (drive_state->functions == 12) {
        mot->dist = 0.2;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 13) {
        mot->angle = 90.0/180.0*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 14) {
        mot->dist = 1.5;
        rstate->line_state[1] = 'l';
        drive_state->state = drive_fwl;

        if (det->negative_fork == 1) {
            drive_state->functions = 15;
        }
    } else if (drive_state->functions == 15) {
        mot->dist = 0.2;
        drive_state->state = drive_fwd;
    } else if ( drive_state->functions == 16) {
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 17) {
        mot->dist = 1.5;
        drive_state->state = drive_fwl;

        if (det->crossing_line == 1) {
            drive_state->functions = 18;
        }
    } else if (drive_state->functions == 18) {
        mot->dist = 0.2;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 19) {
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 20) {
        rstate->line_state[1] = 'm';
        mot->dist = 1;
        drive_state->state = drive_fwl;
    }  else if (drive_state->functions == 21) {
        mot->dist = 2;
        rstate->line_state[1] = 'r';

        if (det->crossing_line == 1) {
            drive_state->functions = 22;
        }
    } else {
        det->mis_state = 2;
        det->crossed_lines = 0;
        drive_state->functions = 0;
    }
}
void third_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
    if (drive_state->functions == 0) {
        mot->dist = 0.5;
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

    } else if (drive_state->functions == 1){
        mot->dist = 2;
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

        if (det->crossing_line == 1) {
            drive_state->functions = 2;
        }
    } else if (drive_state->functions == 2) {
        mot->dist = 0.2;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 3) {
        mot->angle = 90.0/180*M_PI;
        drive_state->state = drive_turn;
    } else {
        det->mis_state = 3;
        det->crossed_lines = 0;
        drive_state->functions = 0;
    }
}
void fourth_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
    if (drive_state->functions == 0) {
        mot->dist = 2;
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;
    } else {
        det->mis_state = 4;
        det->crossed_lines = 0;
        drive_state->functions = 0;
    }
}
void fifth_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
    if (drive_state->functions == 0) {
        mot->dist = 3;
        rstate->line_state[0] = 'b';
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

        if (det->crossing_line == 1) {
            drive_state->functions = 1;
        }
    } else if (drive_state->functions == 1) {
        mot->dist = 0.4;
        rstate->line_state[0] = 'w';
        rstate->line_state[0] = 'm';
        drive_state->state = drive_fwd;

        if (det->line_detected == 1) {
            drive_state->functions = 2;
        }

    } else if (drive_state->functions == 2) {
        mot->dist = 0.2;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 3) {
        mot->angle = 20.0/180.0*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 4) {
        mot->dist = 1;
        drive_state->state = drive_fwl;

    } else if (drive_state->functions == 5) {
        mot->dist = 2;
        drive_state->state = drive_fwl;

        if (det->line_end == 1) {
            drive_state->functions = 6;
        }
    } else {
        det->mis_state = 5;
        det->crossed_lines = 0;
        drive_state->functions = 0;
    }
}
void sixth_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
    if (drive_state->functions == 0) {
        mot->dist = 0.5;
        rstate->line_state[0] = 'b';
        drive_state->state = drive_fwl;

        if (det->crossing_line == 1) {
            drive_state->functions == 1;
        }
    } else if (drive_state->functions == 1) {
        mot->angle = -90.0/180*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 2) {
        mot->dist = 5;
        drive_state->state = drive_fwl;

        if (det->obstacle_det_ir == 1) {
            drive_state->state = drive_end;
        }
    }
}



void make_square(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
    mot->dist = 1;
    mot->angle = 90.0/180*M_PI;
    if (drive_state->functions == 0) {
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 1) {
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 2) {
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 3) {
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 4) {
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 5) {
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 6) {
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 7) {
        drive_state->state = drive_turn;
    } else {
        drive_state->state = drive_end;
    }

    printf("Time: (%d) || mission func incre: %d, angle: %f, drive_state: %d, angle turn: %f, negative_fork: %d, mission_state: %d, "
           "startpos: %f, distance gone: %f, motiontype dist: %f, crossing_line: %d\n", drive_state->time,
           drive_state->functions, odo->angle_pos_mission, drive_state->state, mot->angle, det->negative_fork, det->mis_state,
           mot->startpos, ((mot->right_pos + mot->left_pos)/2 - mot->startpos), mot->dist, det->crossing_line);
}


