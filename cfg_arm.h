#ifndef _CFG_ARM_H
#define _CFG_ARM_H

#define NUM_JOINTS 7

/*--- イナーシャ ---*/

#define INERTIA1   (0.75)
#define INERTIA2   (0.75)
#define INERTIA3   (0.2125)
#define INERTIA4   (0.2125)
#define INERTIA5   (0.0575)
#define INERTIA6   (0.0575)
#define INERTIA7   (0.0575)

/*--- 許容最大トルク指令値 ---*/

#define MAX_TORQUE1   (200.0)
#define MAX_TORQUE2   (200.0)
#define MAX_TORQUE3   (80.0)
#define MAX_TORQUE4   (80.0)
#define MAX_TORQUE5   (15.0)
#define MAX_TORQUE6   (15.0)
#define MAX_TORQUE7   (10.0)

/*--- Length of arm_link ---*/

#define LB  (0.0)
#define LS  (0.450)
#define LE  (0.480)
#define LW  (0.070)
#define HZ  (0.0)



/*--- 速度ゲイン ---*/

#define KD1   (123.)
#define KD2   (123.)
#define KD3   (104.)
#define KD4   (104.)
#define KD5   (70.)
#define KD6   (91.)
#define KD7   (91.)


/*--- 位置ゲイン ---*/

#define KP1   (10500.)
#define KP2   (9000.)
#define KP3   (7000.)
#define KP4   (7000.)
#define KP5   (5000.)
#define KP6   (5000.)
#define KP7   (4900.)

#define SQ2   (1./sqrt(2.))

/*--- parameter for trajectory ---*/

struct status {
  double pos[NUM_JOINTS];
  double vel[NUM_JOINTS];
  double acc[NUM_JOINTS];
};


struct path{
  double pos[NUM_JOINTS][6];
  double vel[NUM_JOINTS][5];
  double acc[NUM_JOINTS][4];
  double time;
};

#endif /* _CFG_ARM_H */



