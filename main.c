/*****************************************************

PA10 Control Program ( 1-Axis Only )

******************************************************/


#include "include.h"
#include "main_cfg.h"
#include "cfg_arm.h"
#include "params_arm.h"
#include "./Arcnet/arc_pci.h"
#include "timer.h"
#include "reverse.h"

#define LEN 10000

FILE *fp;

struct params motorp;
int brakeoff_joint;
int ctrlEndFlag = 0;
int ctrltrig = OFF;

/* Joint constants */
// TODO: adjust these to account for all 7 joints
const double inertia[7] = { INERTIA1, INERTIA2, INERTIA3, INERTIA4, INERTIA5, INERTIA6, INERTIA7 };
const double Kd[7] = { KD1, KD2, KD3, KD4, KD5, KD6, KD7 };
const double Kp[7] = { KP1, KP2, KP3, KP4, KP5, KP6, KP7 };
const double joint_limit[2] = {175.*DEG2RAD, 85.*DEG2RAD};
const double max_torque[7]= { MAX_TORQUE1, MAX_TORQUE2, MAX_TORQUE3, MAX_TORQUE4, MAX_TORQUE5, MAX_TORQUE6, MAX_TORQUE7};
struct path    path_j;
struct status  cur_j, des_j;    
double torque[7];
double des_pos_log[7][LEN], des_vel_log[7][LEN], des_acc_log[7][LEN], cur_pos_log[7][LEN];

/* Initialization functions */
void init(void);
int initializeAll(void);
void initializeData(void);
void start(void);
int ctrlTask(struct params *motor);
void fin(void);

/* Control functions */
void jointCtrl(struct params *param, int trig);
void lineCtrl(struct params *param, int trig);
void circleCtrl(struct params *param, int trig);
void getCurrentPosition(struct status *Cur_j);
void pathInit_j(double *Start, double *Destination, double Time, int joint_num);
void pathInit_l(double Time);
void pathInit_c(double Time);
void pathGenerate_j(struct status *Des_j, unsigned long Time, int joint_num);
void pathGenerate_l(struct status *Des_j, struct params *param, unsigned long Time);
void pathGenerate_c(struct status *Des_j, struct params *param, unsigned long Time);
void pdCtrl(struct status *Des_j, struct status *Cur_j);

/* Basic brake control functions */
void allzero(void);
void allboff(void);	void allbon(void);
void boff1(void);	void boff2(void);
void boff3(void);	void boff4(void);
void boff5(void);	void boff6(void);
void boff7(void);

void control(struct params *motor);
void allbrakeoff(void);
void brakeoff(int joint);
void Nop(void);
int endTask(void);
void joint_moveto(double Angle[NUM_JOINTS], double Time);
void line_drawing(double orientation[3], double start[3], double destination[3], double Time);
void circle_drawing(double orientation[3], double direction[3], double origin[3], double r, double Time);
void All_OFFBrake(void);
void OFFBrake(void);
void ONBrake(void);

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
extern thread_pool_t * cp (int argc, char **argv);

/*--------------------------------------------------------------------*/

/*
 * init: initialize timer channel, setup timer, and initialize all parameters
 * @params: void
 * @return: void
 */
 void init(void) {
 	ThreadCtl(_NTO_TCTL_IO,0);
	
	// exit upon failure to create timer channel
 	if ((chid = ChannelCreate (0)) == -1) {
 		fprintf (stderr,"timer.c: couldn't create channel!\n");
 		perror (NULL);
 		exit (EXIT_FAILURE);
 	}
 	setupTimer();
 	initializeAll();
 }

/*
 * initializeAll: prompt for control mode, set CtrlMode, and initialize data
 * @params: void
 * @return: error code
 */
 int initializeAll(void) {
 	char key[16];

	// prompt for control mode and get input
 	printf("\n\t***** Choose the Control Mode *****\n\n");
 	printf("\tOperation or Simulation ? [o/s] \n\n");
 	fgets(key, sizeof(key), stdin);

 	if (*key == 'o') {
 		CtrlMode = operation;
 		arcInit();
 		printf("\n\tControl Mode -> Operation \n\n");
 	}
 	else if (*key == 's') {
 		CtrlMode = simulation;
 		printf("\n\tControl Mode -> Simulation \n\n");
 	}

	// default control mode is simulation
 	else {
 		CtrlMode = simulation;
 		printf("\n\tControl Mode -> Simulation \n\n");
 	}

	// initialize data
 	initializeData(); 
 	return OK;
 }

/*
 * initalizeData: initialize motor parameters
 * @params: void
 * @return: void
 */
 void initializeData(void) {
 	int i;

 	for (i = 0; i < NUM_JOINTS; i++) { motorp.desPos[i] = 0.; }
	for (i = 0; i < 9; i++)          { motorp.line[i] = 0.; }
	for (i = 0; i < 10; i++)         { motorp.circle[i] = 0.; }
 	motorp.mode = nop;
 	motorp.desTime = 0.;
 	memset(&path_j, 0, sizeof(path_j));
 }

/*
 * start: start communication channel with robot (if necessary), and start
 *        timer. Also begins control task for motor parameters
 * @params: void
 * @return: void
 */
 void start(void) {
 	arcnet_start();

 	timer_settime(timerid, 0, &timer, NULL);
 	ctrltrig = 1;
 	ctrlEndFlag = 1;

 	ctrlTask(&motorp);
 }

/*
 * ctrlTask: given motor parameters, initializes threads to control motors
 * @params: motor - params struct containing control parameters for the motors
 * @return: error code
 */
 int ctrlTask(struct params *motor) {
	int i;
	int rcvid = 1;
 	MessageT msg;
 	static unsigned long ticks = 0;
 	pthread_attr_t attr;

 	pthread_attr_init(&attr);
 	pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED );

	// continually control motors while ctrltrig is ON
 	while (ctrltrig) {
		 
		//record the log data in the arrays
		 for(i=0; i < 7; i++){
		 	if((ticks%10) == 0 && (ticks/10) < LEN){ //make 1 record every 10 TICKS
		 		des_pos_log[i][ticks / 10] = des_j.pos[i]; 
				des_vel_log[i][ticks / 10] = des_j.vel[i];
				des_acc_log[i][ticks / 10] = des_j.acc[i];
				cur_pos_log[i][ticks / 10] = cur_j.pos[i];
		 	}
		 }
		// QNX command: wait for message from channel and check success code
 		rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
 		if (rcvid == 0) {
 			if (!ctrlEndFlag) {
 				printf("ERROR : Control Task is out of time.\n        fin\n");
 				fin();
 				return (ERROR);
 			}
 			ctrlEndFlag = 0;

			// spawn thread to control motor and increment ticks
 			pthread_create(NULL, &attr, (void *)control, motor);
 			ticks++;
 		}
 	}
 	return (EXIT_SUCCESS);
 }

/*
 * control: given motor parameters, sends appropriate commands to control them
 * @params: motor - params struct containing control parameters for the motors
 * @return: void
 */
 void control(struct params *motor) {
 	int stat1;
 	static struct params param;
 	static int mode = nop;
 	static int joint;
 	static int trig;

	// make sure to lock the mutex for motor control
 	if ( pthread_mutex_lock ( &mutex ) != EOK ) {
 		printf(" ERROR : mutex cannnot lock.\n        fin\n");
 		fin();
 		return;
 	}

	// different setup procedures for different control modes
	// probably for debug only, stat1 is never used
 	switch(CtrlMode) {
 		case operation:		stat1 = iSend_C(); 		break;
 		case simulation:	break;
 		default:	   		break;
 	}

	// obtain a copy of the parameters, and get current position of arm
 	memcpy(&param, motor, sizeof(struct params));  
 	getCurrentPosition(&cur_j);

 	trig = param.trig;
	if (trig) { motor->trig = FALSE; } // switch off trig in motor param after copying if it was on before
	mode = param.mode;
	joint = param.joint;

	// different control methods for different modes
	switch(mode) {
		case joint_mode: 		jointCtrl(&param, trig); 	break;
		case line_mode:         lineCtrl(&param, trig);     break;
		case circle_mode:       circleCtrl(&param, trig);   break;
		case allbrakeoff_mode: 	allbrakeoff(); 				break;
		case brakeoff_mode:		brakeoff(joint);    		break;
		default:	    	    Nop();			    	    break;
	}

	// probably more debug stuff, probably don't need to change anything here
	switch(CtrlMode) {
		case operation:   		stat1 = RecData();    		break;
		case simulation:   		break;
		default:		   		break;
	}

	endTask();
	return;
}

/*
 * jointCtrl: joint control mode control instructions
 * @params: param - struct params containing control arguments
 *          trig - boolean indicating control designation
 * @return: void
 */
 void jointCtrl(struct params *param, int trig) {
 	static unsigned long tick;
 	static double desPos[7], desTime;
 	int i; 

	// only initalize path if trig is ON
 	if (trig) { 
 		desTime = param->desTime;
		// initialize path and reset ticks
 		for (i = 0; i < NUM_JOINTS; i++) { 
 			desPos[i] = param->desPos[i];
			pathInit_j(&(cur_j.pos[i]), &(desPos[i]), desTime, i);
 		}
		//turn the tick to 0 after initalize path
 		tick = 0;
 		return;
 	}

	// if trig is OFF, just generate the path and increment ticks
 	for (i = 0; i < NUM_JOINTS; i++) {
 		pathGenerate_j(&des_j, tick, i);
 	}	
 	pdCtrl(&des_j, &cur_j);
	
 	tick++;
 }

 /*
 * lineCtrl: line control mode control instructions
 * @params: param - struct params containing control arguments
 *          trig - boolean indicating control designation
 * @return: void
 */
 void lineCtrl(struct params *param, int trig) {
	 static unsigned long tick;
	 static double desTime;
	 
	 // only initalize path if trig is ON
	 if (trig) {  
	 desTime = param->desTime;
	 pathInit_l(desTime);
	 
	 tick = 0;
	 return;
	 }
	 
	 // if trig is OFF, just generate the path and increment ticks
	 pathGenerate_l(&des_j, param, tick);
	 pdCtrl(&des_j, &cur_j);
	 
	 tick++;
 }

/*
 * circleCtrl: circle control mode control instructions
 * @params: param - struct params containing control arguments
 *          trig - boolean indicating control designation
 * @return: void
 */
  void circleCtrl(struct params *param, int trig) {
	  static unsigned long tick;
	  static desTime;
	  int i;

	  // only initalize path if trig is ON
	  if (trig) {
	 	 desTime = param->desTime;
		 pathInit_c(desTime);
	 	 
	 	 tick = 0;
	 	 return;
	  }

	  // if trig is OFF, just generate the path and increment ticks
	
	  pathGenerate_c(&des_j, param, tick);
	  pdCtrl(&des_j, &cur_j);
	  
	  tick++;
  }


 static void print_path(struct path *p, int joint_num) {
 	int i;
 	printf("TIME:  %3.2f\n", p->time);
 	printf("\nPOS:\n");
 	for (i = 0; i < 6; i++) { printf("%d: %4.2f  |  ", i, p->pos[joint_num][i]); }
 	printf("\nVEL:\n");
 	for (i = 0; i < 5; i++) { printf("%d: %4.2f  |  ", i, p->vel[joint_num][i]); }
 	printf("\nACC:\n");
 	for (i = 0; i < 4; i++) { printf("%d: %4.2f  |  ", i, p->acc[joint_num][i]); }
 	printf("\n");
 }



 /*
 * getCurrentPosition: gets the current position of all joints into Cur_j
 * @params: Cur_j - pointer into status structure to which position should be copied
 * @return: void
 */
 void getCurrentPosition(struct status *Cur_j) {

	 double angle[NUM_JOINTS];
	 int i;
	 static double pre_j_pos[NUM_JOINTS] = { 0, 0, 0, 0, 0, 0, 0 };
	 static double pre_j_vel[NUM_JOINTS] = { 0, 0, 0, 0, 0, 0, 0 };

	 
	 GetPosition(angle);



	 /*--- Selection of control mode ---*/

	 // the commands here should theoretically be very similar once we get to them
	 // TODO: update pre-delta values once we have them so that deltas can be checked for reverse kinematics
	 switch (CtrlMode) {
	 case operation:

		 // return angle of all joints
		 for (i = 0; i < NUM_JOINTS; i++) {
			 Cur_j->pos[i] = angle[i];

			 // calculate velocity and acceleration of joint
			 Cur_j->vel[i] = (Cur_j->pos[i] - pre_j_pos[i]) / TICKS;
			 Cur_j->acc[i] = (Cur_j->vel[i] - pre_j_vel[i]) / TICKS;

			 pre_j_pos[i] = Cur_j->pos[i];
			 pre_j_vel[i] = Cur_j->vel[i];
		 }
		 break;

	 case simulation:	

		 // return angle of all joints
		 for (i = 0; i < NUM_JOINTS; i++) {
			 // in simulation mode, the cur_j and des_j are the same
			 Cur_j->pos[i] = des_j.pos[i];

			 // calculate velocity and acceleration of joint
			 Cur_j->vel[i] = (Cur_j->pos[i] - pre_j_pos[i]) / TICKS;
			 Cur_j->acc[i] = (Cur_j->vel[i] - pre_j_vel[i]) / TICKS;

			 pre_j_pos[i] = Cur_j->pos[i];
			 pre_j_vel[i] = Cur_j->vel[i];
		 }
		 break;

	 default:			break;
	 }
 }

 /*
 * pathInit_j: initialize global path_j values given start position, end position
 *             and time, using polynomial interpolation
 * @params: Start - start value in one dimension
 *          Destination - end value in one dimension
 *          Time - time required for total path traversal
 * @return: void
 */

 void pathInit_j(double *Start, double *Destination, double Time, int joint_num) {
 	int i = joint_num;
 	double t3, t4, t5;
 	double diff;

	// define vars for t^3, t^4, and t^5 for convenience
 	t3 = Time * Time * Time;
 	t4 = t3 * Time;
 	t5 = t4 * Time;

 	diff = *Destination - *Start;

	// set initial values for polynomial interpolation in path_j
 	path_j.pos[i][0] = *Start;
 	path_j.pos[i][3] =  10. * diff / t3;
 	path_j.pos[i][4] = -15. * diff / t4;
 	path_j.pos[i][5] =   6. * diff / t5;

 	path_j.vel[i][2] = 3. * path_j.pos[i][3];
 	path_j.vel[i][3] = 4. * path_j.pos[i][4];
 	path_j.vel[i][4] = 5. * path_j.pos[i][5];

 	path_j.acc[i][1] =  6. * path_j.pos[i][3];
 	path_j.acc[i][2] = 12. * path_j.pos[i][4];
 	path_j.acc[i][3] = 20. * path_j.pos[i][5];

 	// only increment time when adjusting first joint
 	if (joint_num == 0) { path_j.time = Time; }

 	
 }

 /*
 * pathInit_l: initialize global path_j values given time, using polynomial interpolation
 * @params: diff - the value of the needed for parametric equation,here is 1 
 *          Time - time required for total path traversal
 * @return: void
 */
 void pathInit_l(double Time) {
	 double t3, t4, t5;
	 double diff;

	 // define vars for t^3, t^4, and t^5 for convenience
	 t3 = Time * Time * Time;
	 t4 = t3 * Time;
	 t5 = t4 * Time;

	 diff = 1;

	 // set initial values for polynomial interpolation in path_j
	 // in line mode, the vel and acc of the polynomial are meaningless, only calculate the polynomial for pose
	 path_j.pos[1][0] = 0;
	 path_j.pos[1][3] = 10. * diff / t3;
	 path_j.pos[1][4] = -15. * diff / t4;
	 path_j.pos[1][5] = 6. * diff / t5;

	 path_j.time = Time;

 }

 /*
 * pathInit_c: initialize global path_j values given time, using polynomial interpolation
 * @params: diff - the value of the needed for parametric equation, here is 2PI
 *          Time - time required for total path traversal
 * @return: void
 */
 void pathInit_c(double Time) {
	 double t3, t4, t5;
	 double diff;

	 // define vars for t^3, t^4, and t^5 for convenience
	 t3 = Time * Time * Time;
	 t4 = t3 * Time;
	 t5 = t4 * Time;

	 diff = 2*PI;

	 // set initial values for polynomial interpolation in path_j
	 path_j.pos[1][0] = 0;
	 path_j.pos[1][3] = 10. * diff / t3;
	 path_j.pos[1][4] = -15. * diff / t4;
	 path_j.pos[1][5] = 6. * diff / t5;

	 path_j.time = Time;

 }

/*
 * pathGenerate_j: increment Des_j along path given the elapsed time within path
 * @params: Des_j - status of destination joint
 *          Time - elapsed time since path start
 *          joint_num - number of the joint to generate path for
 * @return: void
 */

 void pathGenerate_j(struct status *Des_j, unsigned long Time, int joint_num) {
 	double t;
 	int i = joint_num;

	// get current elapsed time in ticks
	// if the ticks is out of the range of path time, stop adding it
 	t = (Time * TICKS);
 	t = (t > path_j.time) ? path_j.time : t;

	// calculate new joint status along path
 	Des_j->pos[i] = path_j.pos[i][0] + t * t * t * (path_j.pos[i][3]	+ t * (path_j.pos[i][4] + t * path_j.pos[i][5]));
 	Des_j->vel[i] = t * t * (path_j.vel[i][2]	+ t * (path_j.vel[i][3] + t * path_j.vel[i][4]));
 	Des_j->acc[i] = t * (path_j.acc[i][1] + t * (path_j.acc[i][2] + t * path_j.acc[i][3]));
 }
 
 
 /*
 * pathGenerate_l: increment Des_j along path given the elapsed time within a line path
 * @params: Des_j - status of destination joint
 *          Time - elapsed time since path start
 *          param - information of the line
 * @return: void
 */
 void pathGenerate_l(struct status *Des_j, struct params *param, unsigned long Time){
	 double t0, t1, t2, alpha0, alpha1, alpha2;
	 int i;
	 double T1[M_SIZE][M_SIZE], T2[M_SIZE][M_SIZE], T3[M_SIZE][M_SIZE];
	 double solutions1[NUM_JOINTS], solutions2[NUM_JOINTS], solutions3[NUM_JOINTS];
	 double c1, c2, c3, s1, s2, s3;
	 double orientation[3], destination[3], start[3];

	 //get information of the line from pointer param
	 orientation[0] = param->line[0];  orientation[1] = param->line[1];  orientation[2] = param->line[2];
	 start[0]       = param->line[3];        start[1] = param->line[4];        start[2] = param->line[5]; 
	 destination[0] = param->line[6];  destination[1] = param->line[7];  destination[2] = param->line[8];

	 // get current elapsed time in ticks
	 // t0 t1 t2 means the time of t0, t0+TICK and t0 + 2TICK
	 t0 = (Time * TICKS);
	 t0 = (t0> path_j.time) ? path_j.time : t0;
	 t1 = t0 + TICKS;
	 t1 = (t1> path_j.time) ? path_j.time : t1;
	 t2 = t1 + TICKS;
	 t2 = (t2> path_j.time) ? path_j.time : t2;
	 
	 // calculate the value needed for the Parametric equation of the line through fifth order polynomials
	 alpha0 = path_j.pos[1][0] + t0 * t0 * t0 * (path_j.pos[1][3] + t0 * (path_j.pos[1][4] + t0 * path_j.pos[1][5]));
	 alpha1 = path_j.pos[1][0] + t1 * t1 * t1 * (path_j.pos[1][3] + t1 * (path_j.pos[1][4] + t1 * path_j.pos[1][5]));
	 alpha2 = path_j.pos[1][0] + t2 * t2 * t2 * (path_j.pos[1][3] + t2 * (path_j.pos[1][4] + t2 * path_j.pos[1][5]));
	
	 // calculate new joint status along path
	

	 //transfer input into 4*4 D-H matrix T;
	 //the euler angles order is X Y Z;
	 c1 = cos(orientation[0] * DEG_TO_RAD);
	 c2 = cos(orientation[1] * DEG_TO_RAD);
	 c3 = cos(orientation[2] * DEG_TO_RAD);
	 s1 = sin(orientation[0] * DEG_TO_RAD);
	 s2 = sin(orientation[1] * DEG_TO_RAD);
	 s3 = sin(orientation[2] * DEG_TO_RAD);
	 

	 //generate the d-h matrix in t0, t0 + TICKS and t0 + 2TICKS
	 T1[0][0] = c2*c3;             T1[0][1] = -c2*s3;            T1[0][2] = s2;     T1[0][3] = start[0] + (destination[0] - start[0])*alpha0;
	 T1[1][0] = s1*s2*c3 + c1*s3;  T1[1][1] = -s1*s2*s3 + c1*c3; T1[1][2] = -s1*c2; T1[1][3] = start[1] + (destination[1] - start[1])*alpha0;
	 T1[2][0] = -c1*s2*c3 + s1*s3; T1[2][1] = c1*s2*s3 + s1*c3;  T1[2][2] = c1*c2;  T1[2][3] = start[2] + (destination[2] - start[2])*alpha0;
	 T1[3][0] = 0;                 T1[3][1] = 0;                 T1[3][2] = 0;      T1[3][3] = 1;
	 
	 T2[0][0] = c2*c3;             T2[0][1] = -c2*s3;            T2[0][2] = s2;     T2[0][3] = start[0] + (destination[0] - start[0])*alpha1;
	 T2[1][0] = s1*s2*c3 + c1*s3;  T2[1][1] = -s1*s2*s3 + c1*c3; T2[1][2] = -s1*c2; T2[1][3] = start[1] + (destination[1] - start[1])*alpha1;
	 T2[2][0] = -c1*s2*c3 + s1*s3; T2[2][1] = c1*s2*s3 + s1*c3;  T2[2][2] = c1*c2;  T2[2][3] = start[2] + (destination[2] - start[2])*alpha1;
	 T2[3][0] = 0;                 T2[3][1] = 0;                 T2[3][2] = 0;      T2[3][3] = 1;

	 T3[0][0] = c2*c3;             T3[0][1] = -c2*s3;            T3[0][2] = s2;     T3[0][3] = start[0] + (destination[0] - start[0])*alpha2;
	 T3[1][0] = s1*s2*c3 + c1*s3;  T3[1][1] = -s1*s2*s3 + c1*c3; T3[1][2] = -s1*c2; T3[1][3] = start[1] + (destination[1] - start[1])*alpha2;
	 T3[2][0] = -c1*s2*c3 + s1*s3; T3[2][1] = c1*s2*s3 + s1*c3;  T3[2][2] = c1*c2;  T3[2][3] = start[2] + (destination[2] - start[2])*alpha2;
	 T3[3][0] = 0;                 T3[3][1] = 0;                 T3[3][2] = 0;      T3[3][3] = 1;

	 //generate destination angles[7] in  t0, t0 + TICKS and t0 + 2TICKS
	 for (i = 0; i < NUM_JOINTS; i++) { solutions1[i] = 0; }
	 for (i = 0; i < NUM_JOINTS; i++) { solutions2[i] = 0; }
	 for (i = 0; i < NUM_JOINTS; i++) { solutions3[i] = 0; }
	 inverse(T1, solutions1);
	 inverse(T2, solutions2);
	 inverse(T3, solutions3);

	 // calculate new joint status along path
	 for (i = 0; i < NUM_JOINTS; i++) {
		 
		 Des_j->pos[i] = solutions1[i];
		 Des_j->vel[i] = (solutions2[i] - solutions1[i]) / (TICKS);
		 Des_j->acc[i] = (solutions3[i] - 2 * solutions2[i] + solutions1[i]) / (TICKS*TICKS);
	 }
 }
 
/*
 * pathGenerate_c: increment Des_j along path given the elapsed time within a line path
 * @params: Des_j - status of destination joint
 *          Time - elapsed time since path start
 *          param - information of the circle
 * @return: void
 */
 void pathGenerate_c(struct status *Des_j, struct params *param, unsigned long Time){
	 
	 double t0, t1, t2, alpha0, alpha1, alpha2;
     double x0, y0, z0, x1, y1, z1, x2, y2, z2;
     double r;
 	 int i;
 	 double T1[M_SIZE][M_SIZE], T2[M_SIZE][M_SIZE], T3[M_SIZE][M_SIZE];
 	 double solutions1[NUM_JOINTS], solutions2[NUM_JOINTS], solutions3[NUM_JOINTS];
 	 double c1, c2, c3, s1, s2, s3;
	 double orientation[3], direction[3], origin[3];
	 double a[3], b[3], a_temp[3];
	 double vector_i[3], vector_j[3];
	

	 orientation[0] = param->circle[0];  orientation[1] = param->circle[1];  orientation[2] = param->circle[2];
	 direction[0] = param->circle[3];    direction[1] = param->circle[4];    direction[2] = param->circle[5];
	 origin[0] = param->circle[6];       origin[1] = param->circle[7];       origin[2] = param->circle[8];
	 r = param->circle[9];

      //calculate the vectors a b which are needed in circle Parametric equation generating
      vector_i[0] = 1; vector_i[1] = 0; vector_i[2] = 0; 
      vector_j[0] = 0; vector_j[1] = 1; vector_j[2] = 0;
      cross_unit(direction, vector_i, a_temp);
	  if (a_temp[0]*a_temp[0] + a_temp[1]*a_temp[1] + a_temp[2]*a_temp[2] == 0 ){ cross_unit(direction, vector_j, a); }
      else {for(i = 0; i < 3; i++ ) {a[i] = a_temp[i];}}
      cross_unit(direction, a, b);

     
	  t0 = (Time * TICKS);
	  t0 = (t0> path_j.time) ? path_j.time : t0;
	  t1 = t0 + TICKS;
	  t1 = (t1> path_j.time) ? path_j.time : t1;
	  t2 = t1 + TICKS;
	  t2 = (t2> path_j.time) ? path_j.time : t2;
	 
	  alpha0 = path_j.pos[1][0] + t0 * t0 * t0 * (path_j.pos[1][3] + t0 * (path_j.pos[1][4] + t0 * path_j.pos[1][5]));
	  alpha1 = path_j.pos[1][0] + t1 * t1 * t1 * (path_j.pos[1][3] + t1 * (path_j.pos[1][4] + t1 * path_j.pos[1][5]));
	  alpha2 = path_j.pos[1][0] + t2 * t2 * t2 * (path_j.pos[1][3] + t2 * (path_j.pos[1][4] + t2 * path_j.pos[1][5]));
      
	  
	  //calculate the desired position of the end point along the circle
      //the equation is a little complex so calculate it separately
	  x0 = origin[0] + r*cos(alpha0)*a[0] + r*sin(alpha0)*b[0];
	  y0 = origin[1] + r*cos(alpha0)*a[1] + r*sin(alpha0)*b[1];
	  z0 = origin[2] + r*cos(alpha0)*a[2] + r*sin(alpha0)*b[2];
     
	  x1 = origin[0] + r*cos(alpha1)*a[0] + r*sin(alpha1)*b[0];
	  y1 = origin[1] + r*cos(alpha1)*a[1] + r*sin(alpha1)*b[1];
	  z1 = origin[2] + r*cos(alpha1)*a[2] + r*sin(alpha1)*b[2];
     
	  x2 = origin[0] + r*cos(alpha2)*a[0] + r*sin(alpha2)*b[0];
	  y2 = origin[1] + r*cos(alpha2)*a[1] + r*sin(alpha2)*b[1];
	  z2 = origin[2] + r*cos(alpha2)*a[2] + r*sin(alpha2)*b[2];

 	 // calculate new joint status along path
 	 
 	 //transfer input into 4*4 D-H matrix T;
 	 //the euler angles order is X Y Z;
 	 c1 = cos(orientation[0] * DEG_TO_RAD);
 	 c2 = cos(orientation[1] * DEG_TO_RAD);
 	 c3 = cos(orientation[2] * DEG_TO_RAD);
 	 s1 = sin(orientation[0] * DEG_TO_RAD);
 	 s2 = sin(orientation[1] * DEG_TO_RAD);
 	 s3 = sin(orientation[2] * DEG_TO_RAD);
	 

 	 //generate the d-h matrix in time, time+TICKS and time+TICKS+TICKS
 	 T1[0][0] = c2*c3;             T1[0][1] = -c2*s3;            T1[0][2] = s2;     T1[0][3] = x0;
	 T1[1][0] = s1*s2*c3 + c1*s3;  T1[1][1] = -s1*s2*s3 + c1*c3; T1[1][2] = -s1*c2; T1[1][3] = y0;
 	 T1[2][0] = -c1*s2*c3 + s1*s3; T1[2][1] = c1*s2*s3 + s1*c3;  T1[2][2] = c1*c2;  T1[2][3] = z0;
 	 T1[3][0] = 0;                 T1[3][1] = 0;                 T1[3][2] = 0;      T1[3][3] = 1;
	 
 	 T2[0][0] = c2*c3;             T2[0][1] = -c2*s3;            T2[0][2] = s2;     T2[0][3] = x1;
	 T2[1][0] = s1*s2*c3 + c1*s3;  T2[1][1] = -s1*s2*s3 + c1*c3; T2[1][2] = -s1*c2; T2[1][3] = y1;
	 T2[2][0] = -c1*s2*c3 + s1*s3; T2[2][1] = c1*s2*s3 + s1*c3;  T2[2][2] = c1*c2;  T2[2][3] = z1;
 	 T2[3][0] = 0;                 T2[3][1] = 0;                 T2[3][2] = 0;      T2[3][3] = 1;

 	 T3[0][0] = c2*c3;             T3[0][1] = -c2*s3;            T3[0][2] = s2;     T3[0][3] = x2;
	 T3[1][0] = s1*s2*c3 + c1*s3;  T3[1][1] = -s1*s2*s3 + c1*c3; T3[1][2] = -s1*c2; T3[1][3] = y2;
 	 T3[2][0] = -c1*s2*c3 + s1*s3; T3[2][1] = c1*s2*s3 + s1*c3;  T3[2][2] = c1*c2;  T3[2][3] = z2;
 	 T3[3][0] = 0;                 T3[3][1] = 0;                 T3[3][2] = 0;      T3[3][3] = 1;

 	 //generate destination angles[7] in time, time+1 and time+2
 	 for (i = 0; i < NUM_JOINTS; i++) { solutions1[i] = 0; }
 	 for (i = 0; i < NUM_JOINTS; i++) { solutions2[i] = 0; }
 	 for (i = 0; i < NUM_JOINTS; i++) { solutions3[i] = 0; }
 	 inverse(T1, solutions1);
 	 inverse(T2, solutions2);
 	 inverse(T3, solutions3);

 	 // calculate new joint status along path
 	 for (i = 0; i < NUM_JOINTS; i++) {
 	 Des_j->pos[i] = solutions1[i];
 	 Des_j->vel[i] = (solutions2[i] - solutions1[i]) / (TICKS);
 	 Des_j->acc[i] = (solutions3[i] - 2 * solutions2[i] + solutions1[i]) / (TICKS*TICKS);
 	 }
 
 }


 /*
 * pdCtrl: calculate torque necessary to match argument statuses, and send control to robot arm
 *         when in operation mode.
 * @params: Des_j - destination status of joint
 *          Cur_j - current status of joint
 * @return: void
 */

 void pdCtrl(struct status *Des_j, struct status *Cur_j) 
 {
 	double accel[NUM_JOINTS];	
	double g_compensation[NUM_JOINTS];
	double MGL1 = 10.15 * 9.8 * 0.3147;
	double MGL2 = 4.65 * 9.8 * 0.2951;
	double MGL3 = 3.12 * 9.8 * -0.0205;
	int i;
	
	//calculate the gravity compensation
	g_compensation[0] = 0;
	g_compensation[1] = -MGL1*sin(Cur_j->pos[1]) + MGL2*sin(Cur_j->pos[1])*cos(Cur_j->pos[3]);
	g_compensation[2] = 0;
	g_compensation[3] = -MGL2*sin(Cur_j->pos[3]) - MGL3*(sin(Cur_j->pos[3])*cos(Cur_j->pos[5]) + cos(Cur_j->pos[3])*cos(Cur_j->pos[4])*sin(Cur_j->pos[6]));
	g_compensation[4] = MGL3*sin(Cur_j->pos[4])*sin(Cur_j->pos[5])*sin(Cur_j->pos[1] + Cur_j->pos[3]);
	g_compensation[5] = 0;
	g_compensation[6] = 0;

	// calculate necessary acceleration and torque given inertial and limiting constraints
 	for (i = 0; i < NUM_JOINTS; i++) {
 		accel[i] = Des_j->acc[i] + Kd[i] * (Des_j->vel[i] - Cur_j->vel[i]) + Kp[i] * (Des_j->pos[i] - Cur_j->pos[i]);
		torque[i] = inertia[i] * accel[i] + g_compensation[i];

		// adjust torque to be within torque constraints
 		if (torque[i] > max_torque[i]) { torque[i] = max_torque[i]; }
 		else if (torque[i] < - max_torque[i]) { torque[i] = -max_torque[i]; }

 	}
 // set torque of actual robot arm in operation mode
 switch (CtrlMode) {
 case operation:	SetTorq(torque); 	break;
 case simulation: 	NoTorq();   		break;
 default:			NoTorq();			break;
 }
 }

 /*
 * endTask: end control task and release mutex
 * @params: void
 * @return: error code
 */
 int endTask(void) {
 	ctrlEndFlag = 1;
 	pthread_mutex_unlock ( &mutex );
 	return (EXIT_SUCCESS);
 }

 void allbrakeoff(void) {
 	AllBrakeOFF(); 
 }

 void brakeoff(int joint) {
 	BrakeOFF(joint);
 }

 void Nop(void) {
 	NoTorq();
 }

/*
 * joint_moveto: helper function to set status parameters
 *               should be adjusted to handle 7 joints later
 * @params: Angle - target angle to move joint to
 *          Time - target time at which angle should be reached
 * @return: void
 */
 void joint_moveto(double Angle[NUM_JOINTS], double Time) {
 	int i;

 	for (i = 0; i < NUM_JOINTS; i++) { motorp.desPos[i] = Angle[i]; }

 	motorp.desTime = Time;
 	motorp.trig    = TRUE;
 	motorp.mode    = joint_mode;
 }


/*
 * line_drawing: helper function to set status parameters              
 * @params: param.line - information of the line
 *          Time - target time at which angle should be reached
 * @return: void
 */
 void line_drawing(double orientation[3], double start[3], double destination[3], double Time){
 	int i;

 	motorp.desTime = Time;
 	motorp.trig    = TRUE;
 	motorp.mode    = line_mode;

 	motorp.line[0] = orientation[0];  motorp.line[1] = orientation[1];  motorp.line[2] = orientation[2]; 
	motorp.line[3] = start[0];        motorp.line[4] = start[1];        motorp.line[5] = start[2]; 
	motorp.line[6] = destination[0];  motorp.line[7] = destination[1];  motorp.line[8] = destination[2];

	printf("\n Line drawing from %.2lf %.2lf %.2lf to %.2lf %.2lf %.2lf \n",start[0],start[1],start[2],destination[0],destination[1],destination[2]);
	printf("\n The orientation of the endpoint in Euler angles X Y Z is %.2lf %.2lf %.2lf \n", orientation[0],orientation[1],orientation[2]);
 }


/*
 * circle_drawing: helper function to set status parameters              
 * @params: param.circle - information of the circle
 *          Time - target time at which angle should be reached
 * @return: void
 */
 void circle_drawing(double orientation[3], double direction[3], double origin[3], double r, double Time){
    
     motorp.desTime = Time;
  	 motorp.trig    = TRUE;
  	 motorp.mode    = circle_mode;

	 motorp.circle[0] = orientation[0]; motorp.circle[1] = orientation[1]; motorp.circle[2] = orientation[2];
	 motorp.circle[3] = direction[0];   motorp.circle[4] = direction[1];   motorp.circle[5] = direction[2];
	 motorp.circle[6] = origin[0];      motorp.circle[7] = origin[1];      motorp.circle[8] = origin[2];
	 motorp.circle[9] = r;

	 printf("\n Circle drawing at the origin %.2lf %.2lf %.2lf and radius %.2lf \n", origin[0], origin[1], origin[2], r);
	 printf("\n The orientation of the endpoint in Euler angles X Y Z is %.2lf %.2lf %.2lf \n", orientation[0], orientation[1], orientation[2]);
	 printf("\n The direction of the circle is %.2lf %.2lf %.2lf \n", direction[0], direction[1], direction[2]);
 }

 void All_OFFBrake(void) {
 	motorp.mode = allbrakeoff_mode;
 }

 void OFFBrake(void) {
 	motorp.mode = brakeoff_mode;
 	motorp.joint = brakeoff_joint;
 }

 void ONBrake(void) {  
 	motorp.mode = nop;
 }

 void fin(void) {
	int i, j;
	
	ctrltrig = 0;
 	ONBrake();          
 	arcFin(); 
	
	//write the log file in csv
	fp = fopen("log_feng.csv", "w");
	fprintf(fp, "time/sec,des_pos_1,cur_pos_1,des_vel_1,des_acc_1,time/sec,des_pos_2,cur_pos_2,des_vel_2,des_acc_2,");
	fprintf(fp, "time/sec,des_pos_3,cur_pos_3,des_vel_3,des_acc_3,time/sec,des_pos_4,cur_pos_4,des_vel_4,des_acc_4,");
	fprintf(fp, "time/sec,des_pos_5,cur_pos_5,des_vel_5,des_acc_5,time/sec,des_pos_6,cur_pos_6,des_vel_6,des_acc_6,time / sec,des_pos_7,cur_pos_7,des_vel_7,des_acc_7\n");
	for (i = 0; i<LEN; i++){
		for (j = 0; j<7; j++){
			fprintf(fp, "%.2f,%.5f,%.5f,%.5f,%.5f,", i*0.01, des_pos_log[j][i], cur_pos_log[j][i], des_vel_log[j][i], des_acc_log[j][i]);
		}
		fprintf(fp, "\n");
	}
	fclose(fp);


 	timer_delete(timerid);
	

 }

/*
 * main: initalize CPC pipeline listener through cp.c and let CP take care
 *       of the rest
 * @params: void
 * @return: error code
 */
 int main(void) {
 	cp(NULL, NULL);      
 	return(OK);
 }
