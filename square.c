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

    // Pillar/Gate detection
    int pillar_gate_det;
}detectors;

/********************************************
* States
*/

typedef struct {
    char line_state[2];
    char pillar_det[1];
    double speed;
    double ir_dist_obstacle_det;

    double laser_measure_front;
}robot_state;

/*****************************************
* odometry
*/
//
//robot parameters
//#define Ed 0.988707 /*robot 10 */
//#define WHEEL_SEPARATION 0.267230	/* m robots 10 11 */
//#define WHEEL_SEPARATION 0.257947 // robot 2
//#define DELTA_M 0.000103 // real robot

//simulation parameters
#define Ed 1.0
#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.26
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

    // Edge value
    double co_edge;

    // Detect obstacle
    int ir_detect;

    // Speed step for specific acceleration
    double speed_step;
    double vmax;
    double vmax_angle;

    // Regulators_values
    double reg_edge;
    double reg_move_fwd;
    double reg_fwl_line;
    double reg_hug_left;
    double reg_gate;
    double inte_hug_left;
    double inte_fwl_line;
    double inte_move_fwd;
    double last_error_fwl;

    // Last iteration linesensorvalues
    double last_itera_linesensor[7];

    // Laser distance
    double laser_dist_to_go;

}motiontype;

enum {mot_stop=1,mot_move,mot_turn,mot_fwl, mot_hugleft, mot_fwd_gate};

void update_motcon(motiontype *p, odotype *odo);

int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);
int fwl(double dist, double speed,int time);
int hugleft(double dist, double speed, int time);
int fwd_gate(double dist, double speed, int time);


/********************************************
* Calibrations
*/

void norm_linesensor(symTableElement *linesensor, double *norm_values, double *black, double *white, char *line_state);
void calib_ir_sensor(symTableElement *irsensor, double *ir_calib_sensor_values);
double co_mass(double *calib, char *line_state, detectors *det);
double edge_detection(double *calib, char *line_state);


/********************************************
* Detection functions
*/
int pillar_detect(double *laserpar, char *g, motiontype *mot);
int ir_detect(double *ir_calib, double ir_dist);

int crossing_line(double *calib, double threshold_crossing_limit);
double angle_to_wall(double laser, double ir);

/********************************************
* Measure functions
*/

double get_distance_meas_front(double *laserpar);


/********************************************
* Initializations
*/

typedef struct{
    int state, oldstate;
    int time;
    int functions, oldfunction;
}smtype;

int detect_negative_fork(motiontype *mot,smtype *drive_state);
void sm_update(smtype *p, motiontype *mot, odotype *c, detectors *det);
void init_last_itera_zero(double *line_sensor_values);

/********************************************
* Regulators
*/

double get_control_fwd(motiontype *mot, odotype *odo, smtype *sm);
double get_control_fwl(motiontype *mot, smtype *sm,robot_state *rstate);
double get_control_edge(motiontype *mot, smtype *sm, robot_state *rstate);
double get_control_hugleft(motiontype *mot, odotype *odo, smtype *sm, double *ir_calib_sensor_values, robot_state *rstate, double laser, double ir);
double get_control_gate(double *laserpar);

/********************************************
* Track function
*/
void competition_track(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);
void make_square(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);
void follow_line(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);
void hug_thefkingwall(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);

/********************************************
* Subsections of competition track
*/
void skip_first_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate);
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

enum {drive_init,drive_fwd,drive_turn,drive_end,drive_fwl,drive_update, drive_idle, drive_hugleft, drive_fwd_gate};

int main()
{
//     simulation
//   double white[8] = {255, 255, 255, 255, 255, 255, 255, 255};
//   double black[8] = {85, 85, 85, 85, 85, 85, 85, 85};
//
   //  SMR 11
    double white[8] = {62.96, 59.90, 62.54, 61.56, 71.13, 67.29, 66.42, 60.81};
    double black[8] = {45.37, 45.37, 45.98, 46.02, 46.51, 46.06, 46.43, 46.21};
    // SMR 10
      //  double white[8] = {63.7, 64.43, 64.56, 63.86, 65.55, 66.64, 65.45, 65.82}; //floor competition
//        double white[8] = {58.54, 58.86, 58.74, 58.49, 59.15, 59.82, 59.57, 59.91};//upstairs floor
//        double black[8] = {54.73, 54.95, 54.47, 54.86, 54.77, 55.11, 55.10, 55.15};
//    //SMR 2
//    double white[8] = {57.12, 58.25, 58.42, 57.93, 57.75, 57.78, 58.06, 58.01};
//    double black[8] = {54.01, 54.28, 54.28, 54.04, 54.10, 54.03, 54.32, 54.83};

//// SMR 12
//    double white[8] = {84.20, 98.54, 103.39, 109.6, 114.02, 111.32, 109.04, 93.76};
//    double black[8] = {53.27, 54.59, 56.67, 56.36, 56.39, 55.81, 57.62, 60.16};


    int running, arg, time=0;
    double acceleration=0.5, angular_acceleration=1, threshold_crossing_limit = 0.5;
    int log_laser = 1, log_motion = 0, log_linesensor = 0;

    // Motion controls
    mot.angle = -90.0/180*M_PI;

    // Case function
    drive_state.functions = 0;
    drive_state.oldfunction = 0;

    // Distance for obstacle detection
    rstate.ir_dist_obstacle_det = 0.2;

    // Motion speed
    rstate.speed = 0.2;

    // States
    rstate.line_state[0] = 'b';
    rstate.line_state[1] = 'l';

    // Pillar detection init
    rstate.pillar_det[0] = 'l';

    // Initialize integrator value to zero
    mot.inte_fwl_line = 0;
    mot.last_error_fwl = 0;
    mot.inte_hug_left = 0;

    // Detectors states
    det.crossed_lines = 0;
    det.mis_state = 3;
    det.old_mis_state = 2;

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
//    printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
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

        // Update pillar detection
        if (det.pillar_gate_det == 0 && drive_state.time >= 70) {
//            printf("Update pillar detection\n");
            det.pillar_gate_det = pillar_detect(laserpar, rstate.pillar_det, &mot);
        }

        if (det.mis_state == 0) {
            rstate.laser_measure_front = get_distance_meas_front(laserpar);
        }

        // Edge detection
//        printf("edge detection: %f\n",edge_detection(norm_linesensor_values, rstate.line_state));

        // Find center of mass
        mot.co_mass = co_mass(norm_linesensor_values, rstate.line_state, &det);
        mot.co_edge = edge_detection(norm_linesensor_values, rstate.line_state);

        // Realise if vechicle crosses a line
        det.crossing_line = crossing_line(norm_linesensor_values, threshold_crossing_limit);

        // Update obstacle detection
        calib_ir_sensor(irsensor, ir_calib_sensor_values);
        det.obstacle_det_ir = ir_detect(ir_calib_sensor_values, rstate.ir_dist_obstacle_det);

        det.negative_fork = detect_negative_fork(&mot,&drive_state);

        /******************************
        /  Update odemtry parameters
        */
        update_odo(&odo);

        /******************************
        /  THE MISSION BEING RUNNED!
        */
//       competition_track(&mot, &odo, &drive_state, &det, &rstate);
//        make_square(&mot, &odo, &drive_state, &det, &rstate);
//        following_white_line(&mot, &odo, &drive_state, &det, &rstate);
//
//        follow_line(&mot,&odo,&drive_state,&det,&rstate);
hug_thefkingwall(&mot,&odo,&drive_state,&det,&rstate);

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

            case drive_hugleft:
                if (hugleft(mot.dist, mot.speed_step, drive_state.time)) {
                    drive_state.functions += 1;
                }
                break;

            case drive_fwd_gate:
                if (fwd_gate(mot.dist, mot.speed_step, drive_state.time)) {
                    drive_state.functions += 1;
                }
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
        mot.reg_fwl_line = get_control_fwl(&mot, &drive_state, &rstate);
        mot.reg_edge = get_control_edge(&mot, &drive_state, &rstate);
        mot.reg_hug_left = get_control_hugleft(&mot, &odo, &drive_state, ir_calib_sensor_values, &rstate,laserpar[0], ir_calib_sensor_values[0]);
        printf("hug control : %f \n",mot.reg_hug_left);
        mot.reg_gate = get_control_gate(laserpar);

//        printf("reg gate: %f\n",mot.reg_gate);

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
////
//        printf("%f %f %f %f %f %f %f %f\n", norm_linesensor_values[0], norm_linesensor_values[1], norm_linesensor_values[2],
//                norm_linesensor_values[3], norm_linesensor_values[4], norm_linesensor_values[5], norm_linesensor_values[6]
//                , norm_linesensor_values[7]);
//
        printf("laser : %f\n", laserpar[0]);




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

void update_motcon(motiontype *p, odotype *odo) {
    if (p->cmd != 0) {
        p->finished = 0;
        switch (p->cmd) {
            case mot_stop:
                p->curcmd = mot_stop;
                break;
            case mot_move:
                p->startpos = (p->left_pos + p->right_pos) / 2;
                p->curcmd = mot_move;
                break;

            case mot_turn:
                if (p->angle > 0)
                    p->startpos = p->right_pos;
                else
                    p->startpos = p->left_pos;
                p->curcmd = mot_turn;
                break;

            case mot_fwl:
                p->startpos = (p->left_pos + p->right_pos) / 2;
                p->curcmd = mot_fwl;
                break;

            case mot_hugleft:
                p->startpos = (p->left_pos + p->right_pos) / 2;
                p->curcmd = mot_hugleft;
                break;

            case mot_fwd_gate:
                p->startpos = (p->left_pos + p->right_pos) / 2;
                p->curcmd = mot_fwd_gate;
                break;

        }

        p->cmd = 0;
    }

    switch (p->curcmd) {
        case mot_stop:
            p->motorspeed_l = 0;
            p->motorspeed_r = 0;
            break;

        case mot_move:
            if (fabs((p->right_pos + p->left_pos) / 2 - p->startpos) > p->dist) {
                p->finished = 1;
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
            } else if (p->speed_step > p->vmax) {
                p->motorspeed_l = p->vmax + p->reg_move_fwd;
                p->motorspeed_r = p->vmax - p->reg_move_fwd;
            }
            // Moving backwards with deceleration.
            else if (p->speed_step < 0) {
                if (fabs(mot.speed_step) > p->vmax) {
                    p->motorspeed_l = -p->vmax + p->reg_move_fwd;
                    p->motorspeed_r = -p->vmax - p->reg_move_fwd;
                } else {
                    p->motorspeed_l = p->speed_step + p->reg_move_fwd;
                    p->motorspeed_r = p->speed_step - p->reg_move_fwd;
                }
            } else {
                p->motorspeed_l = p->speed_step + p->reg_move_fwd;
                p->motorspeed_r = p->speed_step - p->reg_move_fwd;
            }
            break;

        case mot_turn:
            if (p->angle > 0) {
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
                        p->motorspeed_r = p->speed_step;
                        p->motorspeed_l = -p->speed_step;
                    }
                } else {
                    p->motorspeed_l = 0;
                    p->motorspeed_r = 0;
                    p->finished = 1;
                }
            } else {
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
                    p->motorspeed_r = 0;
                    p->motorspeed_l = 0;
                    p->finished = 1;
                }
            }
            break;

        case mot_fwl:
//            printf("Distance gone in following line: %f, The wanted going dist: %f\n",
//                    fabs((p->right_pos+p->left_pos)/2 - p->startpos), p->dist);
            if (fabs((p->right_pos + p->left_pos) / 2 - p->startpos) > p->dist) {
//                printf("Finished fwl line\n");
                p->finished = 1;
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
            } else {
//                printf("The constant added to speed depending on co_mass: %f", p->reg_fwl_line);
//                printf("reg_edge: %f ,  reg_fwl_line: %f \n", p->reg_fwl_line , p->reg_edge);
                p->motorspeed_r = p->speed_step - p->reg_fwl_line + p->reg_edge ;
                p->motorspeed_l = p->speed_step + p->reg_fwl_line - p->reg_edge ;

            }
            break;

        case mot_hugleft:
            if (fabs((p->right_pos + p->left_pos) / 2 - p->startpos) > p->dist) {
                p->finished = 1;
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
            } else {
                p->motorspeed_r = p->speed_step + p->reg_hug_left;
                p->motorspeed_l = p->speed_step - p->reg_hug_left;
            }
            break;

        case mot_fwd_gate:
            if (fabs((p->right_pos + p->left_pos) / 2 - p->startpos) > p->dist) {
                p->finished = 1;
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
            } else if (p->speed_step > p->vmax) {
                p->motorspeed_l = p->vmax + p->reg_move_fwd + p->reg_gate;
                p->motorspeed_r = p->vmax - p->reg_move_fwd - p->reg_gate;
            }
                // Moving backwards with deceleration.
            else if (p->speed_step < 0) {
                if (fabs(mot.speed_step) > p->vmax) {
                    p->motorspeed_l = -p->vmax + p->reg_move_fwd;
                    p->motorspeed_r = -p->vmax - p->reg_move_fwd;
                } else {
                    p->motorspeed_l = p->speed_step + p->reg_move_fwd;
                    p->motorspeed_r = p->speed_step - p->reg_move_fwd;
                }
            } else {
                p->motorspeed_l = p->speed_step + p->reg_move_fwd + p->reg_gate;
                p->motorspeed_r = p->speed_step - p->reg_move_fwd - p->reg_gate;
            }
            break;
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

int hugleft(double dist, double speed, int time) {
    if (time == 0) {
    mot.cmd = mot_hugleft;
    mot.speedcmd = speed;
    mot.dist = dist;
    return 0;
    } else return mot.finished;
}

int fwd_gate(double dist, double speed, int time) {
    if (time == 0) {
        mot.cmd = mot_fwd_gate;
        mot.speedcmd = speed;
        mot.dist = dist;
        return 0;
    } else return mot.finished;
}

/****************************************
 / Updating functions
*/

void sm_update(smtype *p, motiontype *mot, odotype *c, detectors *det){
    if (p->oldfunction!=p->functions || p->oldstate!=p->state || det->old_mis_state!=det->mis_state) {

        mot->inte_fwl_line = 0;
        mot->last_error_fwl = 0;
        mot->inte_move_fwd = 0;
        mot->inte_hug_left = 0;
        det->pillar_gate_det = 0;
//        printf("Resets mission\n");
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

void norm_linesensor(symTableElement *linesensor, double  *norm_values, double *black, double *white, char *line_state) {
    int i;
    for (i = 0; i < 8; i++) {
        if (linesensor->data[i] < black[i] && line_state[0] == 'b') {
            norm_values[i] = 0;
        }
        else if (linesensor->data[i] < black[i] && line_state[0] == 'w') {
            norm_values[i] = 1;
        }
        else if (linesensor->data[i] > white[i] && line_state[0] == 'b') {
            norm_values[i] = 1;
        }
        else if (linesensor->data[i] > white[i] && line_state[0] == 'w') {
            norm_values[i] = 0;
        }
        else {
            if (line_state[0] == 'b') {
                norm_values[i] =  (double) (linesensor->data[i] - black[i])/(white[i]-black[i]);
            }
            else {
                norm_values[i] = 1 - (double) (linesensor->data[i] - black[i])/(white[i]-black[i]);
            }
        }
    }
}

void calib_ir_sensor(symTableElement *irsensor, double *ir_calib_sensor_values){
    int i;

    // SMR 11
//    double Ka = 15.8463, Kb = 74.6344;
//    double Ka[5] = {14.07, 13.17, 13.73, 13.44, 14.49};
//    double Kb[5] = {65.36, 46.78, 79.07, 60.97, 80.92};
//    // SMR 10
//        double Ka[5] = {16.92, 15.63, 16.17, 16.86, 17.89};
//        double Kb[5] = {72.27, 39.65, 49.26, 81.81, 56.85};

   //  Simulation
    double Ka[5] = {15.8463, 15.8463, 15.8463, 15.8463, 15.8463};
    double Kb[5] = {74.6344, 74.6344, 74.6344, 74.6344, 74.6344};
    for (i = 0; i < 5; i++){
        ir_calib_sensor_values[i] = Ka[i]/(irsensor->data[i] - Kb[i]);
    }
}

double co_mass(double *calib, char *line_state, detectors *det) {
    double numerator = 0, denominator = 0, center = 3.5;
    int i;

//    printf("l1: %f, l2: %f, l3: %f, l4: %f, l5 %f, l6: %f, l7: %f, l8: %f\n", calib[0], calib[1],
//            calib[2], calib[3], calib[4], calib[5], calib[6], calib[7]);

    for (i = 0; i < 8; i++) {
        numerator += calib[i] * i;
        denominator += calib[i];
    }

    // To avoid nans and detect lines
    if (denominator <= 3) {
        det->crossing_line = 1;
        return 0;
    } else {
        det->crossing_line = 0;
    }

    // Line detection
    if (denominator <= 6.5 && denominator >= 2) {
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
//    printf("\n %f %f \n",numerator,denominator);
    return (numerator/denominator) - center;
}

double get_distance_meas_front(double *laserpar) {
    return laserpar[4];
}

double edge_detection(double *calib, char *line_state) {
    int i;
    double array[8];
    double numerator = 0, denominator = 0, center = 0;

    if (line_state[1] == 'm') {
        return 0.0;
    }

    for (i = 0; i < 7; i++) {
        array[i] = calib[i + 1] - calib[i];
    }

    if (line_state[1] == 'l') {
        center = 4.0;
        for (i = 0; i < 7; i++) {
            if (array[i] < 0) {
                array[i] = 0;
            }
            if ((array[i]>0) && (i<3) ){
                array[i]=0;
            }
            }
        }
     else if (line_state[1] == 'r') {
        center = 2.0;
        for (i = 0; i < 7; i++) {
            if (array[i] > 0) {
                array[i] = 0;
            } else if (array[i] < 0) {
                array[i] = fabs(array[i]);
            }
            if ((array[i] > 0) && (i > 3)) {
                array[i] = 0;
            }
        }
    }
    for (i = 0; i < 7; i++) {
        numerator += array[i] * i;
        denominator += array[i];
    }

    if (denominator <= 0.1) {
        return 0;
    }
//    printf("co_edge = %f \n",numerator/denominator);

    return numerator/denominator - center ;
}

/****************************************
 / Regulator values functions
*/

double get_control_fwd(motiontype *mot, odotype *odo, smtype *sm) {
    double prop = 0.05 * odo->angle_pos_mission;
    return prop + mot->inte_move_fwd;
}

double get_control_fwl(motiontype *mot, smtype *sm,robot_state *rstate) {
    if ( (fabs(mot->co_mass) >0.15)&& (rstate->line_state[1]=='m')) {
        double prop = 0.05 * mot->co_mass;
        mot->inte_fwl_line += 0.03 * mot->co_mass * 0.01;
        double derivative = 0.00 * (mot->co_mass - mot->last_error_fwl) / 0.01;
        mot->last_error_fwl = mot->co_mass;

//    printf("prop: %f, integral: %f, derivative: %f, co_mass: %f\n", prop, mot->inte_fwl_line, derivative, mot->co_mass);

        return (prop + mot->inte_fwl_line + derivative);
    }
    else {
        return 0;
    }
//    return 0;
}

double get_control_edge(motiontype *mot, smtype *sm, robot_state *rstate) {
    double prop = 0 ;
    if ((fabs(mot->co_edge) > 0.03)&& (rstate->line_state[1]!='m')) {
    prop = 0.08 * mot->co_edge;
    return prop;

}
    else return 0;
}

double get_control_hugleft(motiontype *mot, odotype *odo, smtype *sm, double *ir_calib_sensor_values, robot_state *rstate, double laser, double ir) {
    //d : Hug distance
    printf("angle to wall %f \n",angle_to_wall(laser,ir));
    double d = 0.2;
    //
    double d_IRmax = 0.93;
    //omega_turn : Angular velocity depends on driving speed and desired turning radius
    double omega_turn = rstate->speed / (d + WHEEL_SEPARATION / 2.0 );
    // kp is calculated as a proportional gain so
    // that differential speed saturates based on the maximum sensor value.
    double kp = 10*(WHEEL_SEPARATION * omega_turn / 2.0) / (d_IRmax - d);
    double ki = 0.008;
    printf(" ir sensor : %f \n", ir_calib_sensor_values[0]);
    double satlim = (WHEEL_SEPARATION*rstate->speed)/(WHEEL_SEPARATION/2.0 + d + 0.1);
    double e = (ir_calib_sensor_values[0] - d);
    mot->inte_hug_left += e;
    if (fabs(mot->inte_hug_left*ki)>0.01){
        mot->inte_hug_left = 0.009/ki* mot->inte_hug_left/fabs(mot->inte_hug_left);

    }
    double delta_U = e * kp + mot->inte_hug_left * ki;







    if (ir_calib_sensor_values[0]<0){ //on turn


        return satlim/2.0;
    }else if (fabs(delta_U) > (satlim / 2.0)){
        return satlim /2.0* delta_U/fabs(delta_U);

    }else {
        return delta_U;
    }
}

double get_control_gate(double *laserpar) {
    // far away from gate
    double kp = 0.1;

//    printf("l0: %f, l1: %f, l6: %f, l7: %f\n", laserpar[0], laserpar[1], laserpar[6], laserpar[7]);

    if (laserpar[0] <= 0.1 && laserpar[7] <= 0.1) {
        return kp * fabs(laserpar[0] - laserpar[7]);
    } else if (laserpar[1] <= 0.3 && laserpar[6] <= 0.3) {
        return kp * fabs(laserpar[1] - laserpar[6]);
    } else {
        return 0;
    }

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

int pillar_detect(double *laserpar, char *g, motiontype *mot) {
    if (g[0] == 'l' && laserpar[0] < 0.8) {
//        printf("distance left laser: %f\n", laserpar[0]);
        mot->laser_dist_to_go = laserpar[0] * sin(20.0/180.0*M_PI) + 0.45;
        return 1;
    } else if (g[0] == 'r' && laserpar[8] < 0.3) {
//        printf("distance right laser: %f\n", laserpar[8]);
        mot->laser_dist_to_go = laserpar[8] * sin(20.0/180.0*M_PI) + 0.6;
        return 1;
    } else if (g[0] == 'g' && laserpar[0] < 1.1 && laserpar[8] < 1.1) {
//        printf("I went through a gate\n");
        return 1;
    } else {
        return 0;
    }
}

//Detect fork with branch <90deg , assume step change in line sensor delta_com=0.6
int detect_negative_fork(motiontype *mot,smtype *drive_state) {
    if (drive_state->time > 0){

    return (fabs(mot->co_mass) >= fabs(mot->co_mass_prev + 1));
    }
        else{
            return 0;
            }
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

//    printf("Time: (%d) || mission func incre: %d, angle: %f, drive_state: %d, angle turn: %f, negative_fork: %d, mission_state: %d, "
//           "startpos: %f, distance gone: %f, motiontype dist: %f, crossing_line: %d, line detection: %d, line end detection: %d"
//           ", obstacle detection: %d, pillar detection: %d\n", drive_state->time,
//           drive_state->functions, odo->angle_pos_mission, drive_state->state, mot->angle, det->negative_fork, det->mis_state,
//           mot->startpos, ((mot->right_pos + mot->left_pos)/2 - mot->startpos), mot->dist, det->crossing_line,
//           det->line_detected, det->line_end, det->obstacle_det_ir, det->pillar_gate_det);
}

void first_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
//    printf("Position x: %f\n", odo->x_pos);
//    printf("Position y: %f\n", odo->y_pos);
    if (drive_state->functions == 0) {
        mot->dist = 4;
        rstate->line_state[1] = 'r';
        drive_state->state = drive_fwl;

        if (det->crossing_line == 1) {
            drive_state->functions = 1;
        }
    } else if (drive_state->functions == 1) {
        drive_state->state = drive_idle;
//        printf("Distance from start in odemetry x to crossing line: %f\n"
//               "Distance from crossing line to box with laser: %f\n"
//               "Distance from start to box is: %f\n",
//               fabs(odo->y_pos), rstate->laser_measure_front, fabs(odo->y_pos) + rstate->laser_measure_front + 0.23);

        if (drive_state->time == 60) {
            drive_state->functions = 2;
        }
    } else if (drive_state->functions == 2) {
        mot->angle = 180.0/180.0*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 3) {
        mot->dist = 2;
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

        if (det->negative_fork == 1) {
            drive_state->functions = 4;
        }
    } else if (drive_state->functions == 4) {
        mot->dist = 0.2;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 5) {
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
        mot->dist = 0.2;
        drive_state->state = drive_fwl;
    } else if (drive_state->functions == 8) {
        mot->speedcmd = -0.2;
        rstate->speed = -0.2;
        mot->dist = 0.9;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 9) {
        mot->speedcmd = 0.2;
        rstate->speed = 0.2;
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
        drive_state->state = drive_fwd;

        if (det->crossing_line == 1) {
            drive_state->functions = 12;
        }
    } else if (drive_state->functions == 12) {
        mot->dist = 0.42;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 13) {
        mot->angle = 90.0/180.0*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 14) {
        mot->dist = 1.5;
        rstate->line_state[1] = 'm';
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
        rstate->line_state[1] = 'm';
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
        mot->dist = 0.7;
        drive_state->state = drive_fwl;

    }  else if (drive_state->functions == 21) {
        mot->dist = 2;
        rstate->line_state[1] = 'r';
        drive_state->state = drive_fwl;
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
        rstate->pillar_det[0] = 'l';
        rstate->speed = 0.2;
        mot->speedcmd = 0.2;

        mot->dist = 3;
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

        if (det->pillar_gate_det == 1) {
            drive_state->functions = 1;
        }

    } else if (drive_state->functions == 1){
        mot->dist = mot->laser_dist_to_go;
        drive_state->state = drive_fwl;
//        printf("dist: %f\n", mot->dist);
    } else if (drive_state->functions == 2) {
        mot->angle = 90.0/180.0*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 3) {
        mot->dist = 3.0;
        drive_state->state = drive_fwd_gate;

        if (det->obstacle_det_ir == 1) {
            drive_state->functions = 4;
        }
    } else if (drive_state->functions == 4) {
        mot->angle = -90.0/180.0*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 5) {
        mot->dist = 2;
        drive_state->state = drive_fwd;

        if (det->crossing_line == 1) {
            drive_state->functions = 6;
        }
    } else if (drive_state->functions == 6) {
        mot->dist = 0.2;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 7) {
        mot->angle = 90.0/180.0*M_PI;
        drive_state->state = drive_turn;
    } else {
        det->mis_state = 3;
        det->crossed_lines = 0;
        drive_state->functions = 0;
        det->pillar_gate_det = 0;
    }
}
void fourth_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
//    printf(" function state %d /n",drive_state->functions);
    if (drive_state->functions == 0) {
        rstate->pillar_det[0] = 'l';
        mot->dist = 2;
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

        if (det->pillar_gate_det == 1) {
            drive_state->functions = 1;
        }
    } else if (drive_state->functions == 1) {
        mot->dist = 0.9;
        drive_state->state = drive_fwl;
    } else if (drive_state->functions == 2) {
        mot->angle = 90.0/180.0*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 3) {
        mot->dist = 0.3;
        mot->speedcmd = 0.2;
        rstate->speed = 0.2;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 4) {
//        printf("hugging and line_detected= %d",det->line_detected);
        mot->dist = 10;
        rstate->speed = 0.2;
        drive_state->state = drive_hugleft;

        if (det->line_detected == 1) {
            drive_state->functions = 5;
        }
    } else if (drive_state->functions == 5) {
        mot->dist = 0.2;
        mot->speedcmd = 0.2;
        rstate->speed = 0.2;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 6) {
        mot->angle = 50.0/180.0*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 7) {
        mot->dist = 0.2;
        drive_state->state = drive_fwl;
    } else {
        det->mis_state = 4;
        det->crossed_lines = 0;
        drive_state->functions = 0;
    }
}
void fifth_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
    if (drive_state->functions == 0) {
        rstate->speed=0.2;
        mot->dist = 3;
        rstate->line_state[0] = 'b';
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

        if (det->crossing_line == 1) {
            drive_state->functions = 1;
        }
    } else if (drive_state->functions == 1) {
        mot->dist = 0.1;
        rstate->line_state[0] = 'w';
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwd;

    } else if (drive_state->functions == 2) {
        mot->dist = 0.5;
        drive_state->state = drive_fwd;

        if (det->line_detected == 1) {
            drive_state->functions = 3;
        }
    } else if (drive_state->functions == 3) {
        mot->dist = 0.2;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 4) {
        mot->angle = 90.0/180.0*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 5) {
        mot->dist = 1;
        drive_state->state = drive_fwl;

    } else if (drive_state->functions == 6) {
        mot->dist = 2;
        drive_state->state = drive_fwl;

        if (det->line_end == 1) {
            drive_state->functions = 7;
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

        if (det->line_detected == 1 && drive_state->time >= 100) {
            drive_state->functions = 1;
        }
    } else if (drive_state->functions == 1) {
        mot->dist = 0.2;
        drive_state->state = drive_fwd;
    } else if (drive_state->functions == 2) {
        mot->angle = -90.0/180*M_PI;
        drive_state->state = drive_turn;
    } else if (drive_state->functions == 3) {
        mot->dist = 5;
        drive_state->state = drive_fwl;

        if (det->obstacle_det_ir == 1) {
            drive_state->state = drive_end;
        }
    }
}

void skip_first_mission(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
    if (drive_state->functions == 0) {
        mot->dist = 1;
        rstate->line_state[1] = 'm';
        drive_state->state = drive_fwl;

    } else {
        det->mis_state = 1;
        det->crossed_lines = 0;
        drive_state->functions = 0;
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
           "startpos: %f, distance gone: %f, motiontype dist: %f, crossing_line: %d, line detection: %d, line end detection: %d"
           ", obstacle detection: %d\n", drive_state->time,
           drive_state->functions, odo->angle_pos_mission, drive_state->state, mot->angle, det->negative_fork, det->mis_state,
           mot->startpos, ((mot->right_pos + mot->left_pos)/2 - mot->startpos), mot->dist, det->crossing_line,
           det->line_detected, det->line_end, det->obstacle_det_ir);
}

void follow_line(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
    mot->dist = 1;
    rstate->line_state[0] = 'b';
    rstate->line_state[1] = 'r';
    drive_state->state = drive_fwl;
}

void hug_thefkingwall(motiontype *mot, odotype *odo, smtype *drive_state, detectors *det, robot_state *rstate) {
    mot->dist = 10;
    rstate->speed = 0.2;
    drive_state->state = drive_hugleft;
}

double angle_to_wall(double laser, double ir){
    return (laser-ir-0.1);
}



