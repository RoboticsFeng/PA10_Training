#ifndef REVERSE_H
#define REVERSE_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef PI
#define PI (3.141592653589793238462643383279502)
#endif

#ifndef M_SIZE
#define M_SIZE (4)
#define N_SOL (2)
#endif

#ifndef NUM_JOINTS
#define NUM_JOINTS (7)
#endif

static double det2(double[2][2]);
static double det3(double[3][3]);
static double det4(double[4][4]);

static void mult(double[M_SIZE][M_SIZE], double[M_SIZE][M_SIZE], double[M_SIZE][M_SIZE]);
static void inv(double[M_SIZE][M_SIZE], double[M_SIZE][M_SIZE]);
static void transp(double[M_SIZE][M_SIZE], double[M_SIZE][M_SIZE]);
static void cofactor(double[M_SIZE][M_SIZE], double[M_SIZE][M_SIZE]);
static void print(double[M_SIZE][M_SIZE]);

void inverse(double[M_SIZE][M_SIZE], double[NUM_JOINTS]);



#endif