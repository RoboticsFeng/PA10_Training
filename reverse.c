#include "reverse.h"

static double atan2b(double y, double x) {
	if (x == 0) {
		if (y == 0) { return 0; } else { return PI/2; }
	} else { return atan(y/x); }
}

static double det2(double i[2][2]) {
	return i[0][0]*i[1][1] - i[0][1]*i[1][0];
}

static double det3(double i[3][3]) {
	double a[2][2], b[2][2], c[2][2];
	a[0][0] = i[1][1]; a[0][1] = i[1][2]; a[1][0] = i[2][1]; a[1][1] = i[2][2];
	b[0][0] = i[1][0]; b[0][1] = i[1][2]; b[1][0] = i[2][0]; b[1][1] = i[2][2];
	c[0][0] = i[1][0]; c[0][1] = i[1][1]; c[1][0] = i[2][0]; c[1][1] = i[2][1]; 
	return i[0][0]*det2(a) - i[0][1]*det2(b) + i[0][2]*det2(c);
}

/* 
 * det4: returns the determinant of a 4x4 matrix
 * @params: in - input matrix
 * @return: determinant of in (|in|)
 */
static double det4(double in[M_SIZE][M_SIZE]) {
	double a[3][3], b[3][3], c[3][3], d[3][3];
	int i, j;
	for (i = 0; i < 3; i++) { for (j = 0; j < 3; j++) { 
		a[i][j] = in[i+1][j+1]; 
		b[i][j] = in[i+1][j < 1 ? j : j+1];
		c[i][j] = in[i+1][j < 2 ? j : j+1];
		d[i][j] = in[i+1][j < 3 ? j : j+1]; 
	}}
	return in[0][0]*det3(a) - in[0][1]*det3(b) + in[0][2]*det3(c) - in[0][3]*det3(d);
}

/*
 * mult: finds the product of two matrices
 * @params: a - first input matrix
 *          b - second input matrix
 *          out - output matrix (a*b)
 * @return: void
 */
static void mult(double a[M_SIZE][M_SIZE], double b[M_SIZE][M_SIZE], double out[M_SIZE][M_SIZE]) {
	int i, j, ra, ca, rb, cb;

	for (i = 0; i < M_SIZE; i++) { for (j = 0; j < M_SIZE; j++) {
		out[i][j] = 0;
	}}
	for (ra = 0; ra < M_SIZE; ra++) { for (ca = 0; ca < M_SIZE; ca++) {
		for (rb = 0; rb < M_SIZE; rb++) { for (cb = 0; cb < M_SIZE; cb++) {
			if (ca == rb) { out[ra][cb] += a[ra][ca] * b[rb][cb]; }
		}}
	}}
	return;
}

/*
 * inv: calculates the inverse of an M_SIZE x M_SIZE matrix
 * @params: a - input matrix
 *          out - output matrix (a^-1)
 * @return: void
 */
static void inv(double a[M_SIZE][M_SIZE], double out[M_SIZE][M_SIZE]) {
	double det_m = det4(a);
	double cof[M_SIZE][M_SIZE];
	int i, j;

	cofactor(a, cof);
	transp(cof, out);

	// divide each element of adjoint matrix by the determinant
	for (i = 0; i < M_SIZE; i++) { for (j = 0; j < M_SIZE; j++) {
		out[i][j] = out[i][j] / det_m;
	}}

	return;
}

/*
 * cofactor: finds the cofactor of a M_SIZE x M_SIZE matrix
 * @params: a - input matrix
 *          out - output matrix (cof(a))
 * @return: void
 */
static void cofactor(double a[M_SIZE][M_SIZE], double out[M_SIZE][M_SIZE]) {
	int i, j, ii, jj, i1, j1;
	double det, c[3][3];

	for (i = 0; i < M_SIZE; i++) { for (j = 0; j < M_SIZE; j++) {
		i1 = 0;
		// form the matrix of minors
		for (ii = 0; ii < M_SIZE; ii++) {
			if (ii == i) { continue; }
			j1 = 0;
			for (jj = 0; jj < M_SIZE; jj++) {
				if (jj == j) { continue; }
				c[i1][j1] = a[ii][jj];
				j1++;
			}
			i1++;
		}
		det = det3(c);

		// multiply by correct sign to form the cofactor matrix
		out[i][j] = pow(-1.0, i + j + 2.0) * det;
	}}
	return;
}

/*
 * transp: transposes an input matrix
 * @params: a - input matrix (M_SIZExM_SIZE)
 *          out - output matrix (a')
 * @return: void
 */
static void transp(double a[M_SIZE][M_SIZE], double out[M_SIZE][M_SIZE]) {
	int i, j;
	for (i = 0; i < M_SIZE; i++) { for (j = 0; j < M_SIZE; j++) {	
		out[j][i] = a[i][j];
	}}
	return;
}


// cross_unit: cross two vector and unit the product
//@params: a b - input vector
//         out - output unit vector
// @return: void
//this function is needed in generating the parametric equation  of the circle
void cross_unit(double a[3], double b[3], double out[3]){
	double product[3];
	double abbs;
	int i;

	product[0] = a[1]*b[2] - a[2]*b[1];
	product[1] = a[2]*b[0] - a[0]*b[2];
	product[2] = a[0]*b[1] - a[1]*b[0];

    abbs = sqrt(product[0]*product[0] + product[1]*product[1] + product[2]*product[2] );

	if (abbs > 0){for (i = 0; i < 3; i++){ out[i] = product[i] / abbs; }}
	else { for (i = 0; i < 3; i++){ out[i] = product[i]; } }
    return;
}

/*
 * inverse: given an end-effector location/rotation matrix, determines the solutions
 *          for each of the seven angles, assuming a3 is fixed
 * @params: T - end-effector location/rotation matrix
 *          solutions - output set of solutions for each joint angle
 * @return: void
 */
void inverse(double T[M_SIZE][M_SIZE], double solutions[NUM_JOINTS]) {
	double bx = T[0][2], by = T[1][2], bz = T[2][2];
	double px = T[0][3], py = T[1][3], pz = T[2][3];
	double px1, py1, pz1, m, n, k, n_tmp;
	double a1, a2, a2_1, a2_2, a4, a5, a6, a7;
	double c1, c2, c24, s1, s2, s24, b;
	double a04_inv[M_SIZE][M_SIZE], T1[M_SIZE][M_SIZE];
	int i;
	
	px1 = px - 70*bx; py1 = py - 70*by; pz1 = pz - 70*bz;

	a1 = atan2b(py1, px1);

	m = cos(a1)*px1 + sin(a1)*py1;
	n = pz1 - 317;
	k = ((pow(m, 2) + pow(n, 2))/900) - 31; 

	n_tmp = sqrt(pow(m, 2) + pow(n, 2) - pow(k, 2));
	a2_1 = 2 * atan2b(m + n_tmp, n + k);
	a2_2 = 2 * atan2b(m - n_tmp, n + k);
    
	if ((a2_1 * a2_1) > (a2_2 * a2_2)){ a2 = a2_2; }
	else { a2 = a2_1; }
	
	b = atan2b(m - 450*sin(a2), n - 450*cos(a2)) - a2;
	if ((pz1 - 317 - 450*cos(a2)) / (480*cos(b + a2)) > 0) { a4 = b; } 
	else if (b < 0.2*PI) { a4 = b + PI; }
	else { a4 = b - PI; }

	c1 = cos(a1); c2 = cos(a2); c24 = cos(a2 + a4);
	s1 = sin(a1); s2 = sin(a2); s24 = sin(a2 + a4);

	// symbolic inv(a04) simplified in MATLAB
	a04_inv[0][0] = c24*c1; a04_inv[0][1] = c24*s1; a04_inv[0][2] = -s24; a04_inv[0][3] = +450 * sin(a4) + 317 * s24;
	a04_inv[1][0] = -s1;    a04_inv[1][1] = c1;     a04_inv[1][2] = 0;    a04_inv[1][3] = 0;
	a04_inv[2][0] = s24*c1; a04_inv[2][1] = s24*s1; a04_inv[2][2] = c24;  a04_inv[2][3] = -450 * cos(a4) - 317 * c24;
	a04_inv[3][0] = 0;      a04_inv[3][1] = 0;      a04_inv[3][2] = 0;    a04_inv[3][3] = 1;

	
	// find T1, and calculate the final angles
	mult(a04_inv, T, T1);

	a5 = atan2b(T1[1][2], T1[0][2]);
	a6 = atan2b(T1[0][3]*cos(a5) + T1[1][3]*sin(a5), T1[2][3] - 480);
	a7 = atan2b(T1[1][0]*cos(a5) - T1[0][0]*sin(a5), T1[1][1]*cos(a5) - T1[0][1]*sin(a5));
	solutions[0] = a1; solutions[1] = a2; solutions[2] = 0; solutions[3] = a4; solutions[4] = a5; solutions[5] = a6; solutions[6] = a7;
	
	return;
}

/*
 * print: prints a full M_SIZExM_SIZE matrix
 * @params: a - M_SIZExM_SIZE matrix
 * @return: void
 */
static void print(double a[M_SIZE][M_SIZE]) {
	int i, j;
	printf("Size:\t%d\n", M_SIZE);
	for (i = 0; i < M_SIZE; i++) {
		printf("[  ");
		for (j = 0; j < M_SIZE; j++) {
			printf("%8.4f	", a[i][j]);
		}
		printf("]\n");
	}
}

/* 
 * print_sols: prints a set of solutions in either radians or degrees
 * @params: a - a set of solutions for NUM_JOINTS
 *          degrees - 0 for radians, anything else for degrees
 * @return: void
 */
//static void print_sols(double a[N_SOL][NUM_JOINTS], int degrees) {
//	int i, j;
//	for (i = 0; i < N_SOL; i++) {
//		printf("[  ");
//		for (j = 0; j < NUM_JOINTS; j++) {
//			printf("%8.4f	", a[i][j] * (degrees ? 180/PI : 1));
//		}
//		printf("]\n");
//	}
//}

//void main() {
//	int i, j;
//	double T[M_SIZE][M_SIZE] = 
//	{{ -0.215533103772415, +0.607451653675777, +0.764557368432738, -15.9669897866072 },
//	 { -0.921427386892164, +0.132700274281278, -0.365187907645846, -37.8154111000753 },
//	 { -0.323290970896663, -0.783194181319190, +0.531121287922501, +1249.74789095409 },
//	 {                  0,                  0,                  0,                 1 }};
//
//	double solutions[NUM_JOINTS];
//	inverse(T, solutions);
//	print_sols(solutions, 1);
//}