#ifndef _PARAMS_H
#define _PARAMS_H

#include "./cfg_arm.h"

#ifndef NUM_JOINTS
#define NUM_JOINTS 7
#endif /* NUM_JOINTS */


/*---------- �����������ѥѥ�᡼�� --------------------------------*/

extern const double inertia[NUM_JOINTS];

extern const double Kd[NUM_JOINTS];
extern const double Kp[NUM_JOINTS];

extern const double joint_limit[2];
extern const double max_torque[NUM_JOINTS];




/*---------- ��ƻ�����֡������ѥѥ�᡼�� --------------------------*/

extern struct path    path_j;
extern struct status  cur_j, des_j, com_j;   /* ���祤��Ⱥ�ɸ�� */
extern struct status  cur_o, des_o, com_o;   /* ����ɸ�� */


/*---------- �ƴ���Υȥ륯(�ǡ�����) ------------------------------*/

extern double torque[NUM_JOINTS];

extern int brakeoff_joint;

#endif /* _PARAMS_H */
