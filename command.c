/******************************************************
	 Command Functions for PA10
*******************************************************/
#include "main_cfg.h"
#include "include.h"
#include "params_arm.h"
#include "reverse.h"

#define DEG_TO_RAD (3.14159265 / 180.0)
#define RAD_TO_DEG (180.0 / 3.14159265)

void allzero(void);
void sample(void);
void set_angles(double angles[8]);
void endpoint_moveto(double input[8]);
void lineinit(void);
void line(void);
void circleinit(void);
void circle(void);

void allboff(void);
void allbon(void);
void boff1(void);
void boff2(void);
void boff3(void);
void boff4(void);
void boff5(void);
void boff6(void);
void boff7(void);


extern void inverse(double[M_SIZE][M_SIZE], double[NUM_JOINTS]);

extern void All_OFFBrake(void);
extern void joint_moveto(double Angle[NUM_JOINTS],double Time);
extern void line_drawing(double orientation[3], double start[3], double destination[3], double Time);
extern void circle_drawing(double orientation[3], double direction[3], double origin[3], double r, double Time);
extern void OFFBrake(void);
extern void ONBrake(void);



void allzero(void) {
	double angle[NUM_JOINTS];
	double desTime;
	int i;

	// move all angles to 0 rad in 10 seconds
	desTime = 10.;
	for (i = 0; i < NUM_JOINTS; i++) { angle[i] = 0.; }

	joint_moveto(angle, desTime);

	printf("\n\t Joint Control --- allzero!!\n\n");
}

void sample(void) {
	double angle[NUM_JOINTS] = { 10.0, 20.0, 0.0, 30.0, 40.0, 50.0, 60.0 };
	double desTime;
	int i;

	// convert angles to radians, and set movement time to 10 seconds
	desTime = 10.0;
	for (i = 0; i < NUM_JOINTS; i++) { angle[i] = angle[i]*DEG_TO_RAD; }

	joint_moveto(angle, desTime);
	printf("\n\t Joint Control --- sample angles!!\n\n");
}

void set_angles(double angles[8]) {
	double desTime = 10;
	double angle[NUM_JOINTS];
	int i;
	
	// shift angles for joints > 3 to account for fixed third joint
	for (i = NUM_JOINTS - 1; i > 1; i--) { angles[i] = angles[i-1]; }
	angles[2] = 0;

	// convert angles to radians
	for (i = 0; i < NUM_JOINTS; i++) { angle[i] = angles[i]*DEG_TO_RAD; }
	
	joint_moveto(angle, desTime);
	printf("\n\t Joint Control --- setting to { ");
	for (i = 0; i < NUM_JOINTS; i++) { printf("%.2f ", angles[i]); }
	printf("}\n");
}

void endpoint_moveto(double input[8]){
	double desTime = 10;
	double angle[NUM_JOINTS];
	double T[M_SIZE][M_SIZE];
	double solutions[NUM_JOINTS];
	double c1, c2, c3, s1, s2, s3;
	int i;
	
	//transfer input into 4*4 D-H matrix T;
	//the euler angles order is X Y Z;
	c1 = cos(input[0] * DEG_TO_RAD);
	c2 = cos(input[1] * DEG_TO_RAD);
	c3 = cos(input[2] * DEG_TO_RAD);
	s1 = sin(input[0] * DEG_TO_RAD); 
	s2 = sin(input[1] * DEG_TO_RAD); 
	s3 = sin(input[2] * DEG_TO_RAD);

	T[0][0] = c2*c3;             T[0][1] = -c2*s3;            T[0][2] = s2;     T[0][3] = input[3];
	T[1][0] = s1*s2*c3 + c1*s3;  T[1][1] = -s1*s2*s3 + c1*c3; T[1][2] = -s1*c2; T[1][3] = input[4];
	T[2][0] = -c1*s2*c3 + s1*s3; T[2][1] = c1*s2*s3 + s1*c3;  T[2][2] = c1*c2;  T[2][3] = input[5];
	T[3][0] = 0;                 T[3][1] = 0;                 T[3][2] = 0;      T[3][3] = 1;
	

	//generate destination angles[7]
	for (i = 0; i < NUM_JOINTS; i++) { solutions[i] = 0;}
	inverse(T, solutions);
	for (i = 0; i < 7; i++) { printf("The angle of joint%d is %lf \n",i+1, solutions[i]); }
	//move to the desired angles
	for (i = 0; i<NUM_JOINTS; i++) { 
		angle[i] = solutions[i];
	}
	joint_moveto(angle, desTime);
	printf("\n\t Endpoint Control is Done!!!\n\n");
}

//move the end-effect to the start point of the line
void lineinit(void){
	
	double input[8];
	input[0] = 0.0;   input[1] = 80.0;  input[2] = 0.0;
	input[3] = 100.0; input[4] = 100.0; input[5] = 1250.0;
	input[6] = 0.0;   input[7] = 0.0;

	endpoint_moveto(input);
	printf("\n The endpoint has reached the start, line init is done!\n");
}

// drawing the line
void line(void){
	double desTime = 20;
	double orientation[3], start[3], destination[3];

	orientation[0] = 0.0;    orientation[1] = 80.0;    orientation[2] = 0.0;
	start[0]       = 100.0;        start[1] = 100.0;         start[2] = 1250.0; 
	destination[0] = 500.0;  destination[1] = 100.0;   destination[2] = 1100.0;

	line_drawing(orientation, start, destination, desTime);
	printf("\n\t Linedrawing Control is Done!!!\n\n");
}

//move the end-effect to the start point of the circle
void circleinit(void){
    double input[8];
	double a[3], vector_i[3], vector_j[3], a_temp[3];
	double origin[3], direction[3];
	double r;
	int i;

	direction[0] = 1;    direction[1] = 1;    direction[2] = 1;
	origin[0] = 350;     origin[1] = 250;       origin[2] = 1100;
	r = 100;

	vector_i[0] = 1; vector_i[1] = 0; vector_i[2] = 0;
	vector_j[0] = 0; vector_j[1] = 1; vector_j[2] = 0;
	
	cross_unit(direction, vector_i, a_temp);
	if (a_temp[0]*a_temp[0] + a_temp[1]*a_temp[1] + a_temp[2]*a_temp[2] == 0){ cross_unit(direction, vector_j, a); }
	else { for (i = 0; i < 3; i++){ a[i] = a_temp[i]; } }
	
	input[0] = -50.0; input[1] = 1.0; input[2] = 1.0;
	input[3] = origin[0] + r*a[0]; 
	input[4] = origin[1] + r*a[1];
	input[5] = origin[2] + r*a[2];
	input[6] = 0.0;  input[7] = 0.0;

	endpoint_moveto(input);
 	printf("\n The endpoint has reached the start, circle init is done!\n");
 }


//drawing the circle
 void circle(void){
	 
	 double desTime = 20;
	 double orientation[3], direction[3], origin[3];
	 double r;

	 orientation[0] = -50.0;  orientation[1] = 1.0;     orientation[2] = 1.0;
	 direction[0] = 1;        direction[1] = 1;       direction[2] = 1;
	 origin[0] = 350;         origin[1] = 250;          origin[2] = 1100;
	 r = 100;

	 circle_drawing(orientation, direction, origin, r, desTime);
	 printf("\n\t Circledrawing Control is Done!!!\n\n");
 }

/*============== Brake off ===============*/

void allboff(void) {
	All_OFFBrake();
	printf("\n\tAll axes are brake off!!\n");
}

void allbon(void) {
	ONBrake();
	printf("\n\tAll axes are brake on!!\n");
}

void boff1(void) {
	brakeoff_joint = 0;
	OFFBrake();
	printf("\n\t 1 axis is brake off!!\n");
}

void boff2(void) {
	brakeoff_joint = 1;
	OFFBrake();
	printf("\n\t 2 axis is brake off!!\n");
}

void boff3(void) {
	brakeoff_joint = 2;
	OFFBrake();
	printf("\n\t 3 axis is brake off!!\n");
}

void boff4(void) {
	brakeoff_joint = 3;
	OFFBrake();
	printf("\n\t 4 axis is brake off!!\n");
}

void boff5(void) {
	brakeoff_joint = 4;
	OFFBrake();
	printf("\n\t 5 axis is brake off!!\n");
}

void boff6(void) {
	brakeoff_joint = 5;
	OFFBrake();
	printf("\n\t 6 axis is brake off!!\n");
}

void boff7(void) {
	brakeoff_joint = 6;
	OFFBrake();
	printf("\n\t 7 axis is brake off!!\n");
}
