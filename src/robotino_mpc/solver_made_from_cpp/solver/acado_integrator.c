/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


real_t rk_dim5_swap;

/** Column vector of size: 5 */
real_t rk_dim5_bPerm[ 5 ];

/** Column vector of size: 8 */
real_t auxVar[ 8 ];

real_t rk_ttt;

/** Row vector of size: 29 */
real_t rk_xxx[ 29 ];

/** Column vector of size: 5 */
real_t rk_kkk[ 5 ];

/** Matrix of size: 5 x 5 (row major format) */
real_t rk_A[ 25 ];

/** Column vector of size: 5 */
real_t rk_b[ 5 ];

/** Row vector of size: 5 */
int rk_dim5_perm[ 5 ];

/** Column vector of size: 5 */
real_t rk_rhsTemp[ 5 ];

/** Row vector of size: 45 */
real_t rk_diffsTemp2[ 45 ];

/** Column vector of size: 5 */
real_t rk_diffK[ 5 ];

/** Matrix of size: 5 x 9 (row major format) */
real_t rk_diffsPrev2[ 45 ];

/** Matrix of size: 5 x 9 (row major format) */
real_t rk_diffsNew2[ 45 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim5_perm, rk_A, rk_b, rk_diffsPrev2, rk_diffsNew2, rk_diffsTemp2, rk_dim5_swap, rk_dim5_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 5;
const real_t* od = in + 9;
/* Vector of auxiliary variables; number of elements: 4. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[2]));
a[1] = (sin(xd[2]));
a[2] = (sin(xd[2]));
a[3] = (cos(xd[2]));

/* Compute outputs: */
out[0] = ((u[0]*a[0])-(u[1]*a[1]));
out[1] = ((u[0]*a[2])+(u[1]*a[3]));
out[2] = u[2];
out[3] = (u[3]+u[2]);
out[4] = (((((((((((((((((((od[0]+od[1])+od[2])+od[3])+od[4])+od[5])+od[6])+od[7])+od[8])+od[9])+od[10])+od[11])+od[12])+od[13])+od[14])+od[15])+od[16])+od[17])+od[18])+od[19]);
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 5;
/* Vector of auxiliary variables; number of elements: 8. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[1] = (cos(xd[2]));
a[2] = (cos(xd[2]));
a[3] = (sin(xd[2]));
a[4] = (cos(xd[2]));
a[5] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[6] = (sin(xd[2]));
a[7] = (cos(xd[2]));

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = ((u[0]*a[0])-(u[1]*a[1]));
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = a[2];
out[6] = ((real_t)(0.0000000000000000e+00)-a[3]);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = ((u[0]*a[4])+(u[1]*a[5]));
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = a[6];
out[15] = a[7];
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(1.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(1.0000000000000000e+00);
out[35] = (real_t)(1.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
}



void acado_solve_dim5_triangular( real_t* const A, real_t* const b )
{

b[4] = b[4]/A[24];
b[3] -= + A[19]*b[4];
b[3] = b[3]/A[18];
b[2] -= + A[14]*b[4];
b[2] -= + A[13]*b[3];
b[2] = b[2]/A[12];
b[1] -= + A[9]*b[4];
b[1] -= + A[8]*b[3];
b[1] -= + A[7]*b[2];
b[1] = b[1]/A[6];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim5_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 5; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
if(fabs(A[5]) > fabs(A[0]) && fabs(A[5]) > fabs(A[10]) && fabs(A[5]) > fabs(A[15]) && fabs(A[5]) > fabs(A[20])) {
rk_dim5_swap = A[0];
A[0] = A[5];
A[5] = rk_dim5_swap;
rk_dim5_swap = A[1];
A[1] = A[6];
A[6] = rk_dim5_swap;
rk_dim5_swap = A[2];
A[2] = A[7];
A[7] = rk_dim5_swap;
rk_dim5_swap = A[3];
A[3] = A[8];
A[8] = rk_dim5_swap;
rk_dim5_swap = A[4];
A[4] = A[9];
A[9] = rk_dim5_swap;
rk_dim5_swap = b[0];
b[0] = b[1];
b[1] = rk_dim5_swap;
intSwap = rk_perm[0];
rk_perm[0] = rk_perm[1];
rk_perm[1] = intSwap;
}
else if(fabs(A[10]) > fabs(A[0]) && fabs(A[10]) > fabs(A[5]) && fabs(A[10]) > fabs(A[15]) && fabs(A[10]) > fabs(A[20])) {
rk_dim5_swap = A[0];
A[0] = A[10];
A[10] = rk_dim5_swap;
rk_dim5_swap = A[1];
A[1] = A[11];
A[11] = rk_dim5_swap;
rk_dim5_swap = A[2];
A[2] = A[12];
A[12] = rk_dim5_swap;
rk_dim5_swap = A[3];
A[3] = A[13];
A[13] = rk_dim5_swap;
rk_dim5_swap = A[4];
A[4] = A[14];
A[14] = rk_dim5_swap;
rk_dim5_swap = b[0];
b[0] = b[2];
b[2] = rk_dim5_swap;
intSwap = rk_perm[0];
rk_perm[0] = rk_perm[2];
rk_perm[2] = intSwap;
}
else if(fabs(A[15]) > fabs(A[0]) && fabs(A[15]) > fabs(A[5]) && fabs(A[15]) > fabs(A[10]) && fabs(A[15]) > fabs(A[20])) {
rk_dim5_swap = A[0];
A[0] = A[15];
A[15] = rk_dim5_swap;
rk_dim5_swap = A[1];
A[1] = A[16];
A[16] = rk_dim5_swap;
rk_dim5_swap = A[2];
A[2] = A[17];
A[17] = rk_dim5_swap;
rk_dim5_swap = A[3];
A[3] = A[18];
A[18] = rk_dim5_swap;
rk_dim5_swap = A[4];
A[4] = A[19];
A[19] = rk_dim5_swap;
rk_dim5_swap = b[0];
b[0] = b[3];
b[3] = rk_dim5_swap;
intSwap = rk_perm[0];
rk_perm[0] = rk_perm[3];
rk_perm[3] = intSwap;
}
else if(fabs(A[20]) > fabs(A[0]) && fabs(A[20]) > fabs(A[5]) && fabs(A[20]) > fabs(A[10]) && fabs(A[20]) > fabs(A[15])) {
rk_dim5_swap = A[0];
A[0] = A[20];
A[20] = rk_dim5_swap;
rk_dim5_swap = A[1];
A[1] = A[21];
A[21] = rk_dim5_swap;
rk_dim5_swap = A[2];
A[2] = A[22];
A[22] = rk_dim5_swap;
rk_dim5_swap = A[3];
A[3] = A[23];
A[23] = rk_dim5_swap;
rk_dim5_swap = A[4];
A[4] = A[24];
A[24] = rk_dim5_swap;
rk_dim5_swap = b[0];
b[0] = b[4];
b[4] = rk_dim5_swap;
intSwap = rk_perm[0];
rk_perm[0] = rk_perm[4];
rk_perm[4] = intSwap;
}

A[5] = -A[5]/A[0];
A[6] += + A[5]*A[1];
A[7] += + A[5]*A[2];
A[8] += + A[5]*A[3];
A[9] += + A[5]*A[4];
b[1] += + A[5]*b[0];

A[10] = -A[10]/A[0];
A[11] += + A[10]*A[1];
A[12] += + A[10]*A[2];
A[13] += + A[10]*A[3];
A[14] += + A[10]*A[4];
b[2] += + A[10]*b[0];

A[15] = -A[15]/A[0];
A[16] += + A[15]*A[1];
A[17] += + A[15]*A[2];
A[18] += + A[15]*A[3];
A[19] += + A[15]*A[4];
b[3] += + A[15]*b[0];

A[20] = -A[20]/A[0];
A[21] += + A[20]*A[1];
A[22] += + A[20]*A[2];
A[23] += + A[20]*A[3];
A[24] += + A[20]*A[4];
b[4] += + A[20]*b[0];

det = + det*A[0];

if(fabs(A[11]) > fabs(A[6]) && fabs(A[11]) > fabs(A[16]) && fabs(A[11]) > fabs(A[21])) {
rk_dim5_swap = A[5];
A[5] = A[10];
A[10] = rk_dim5_swap;
rk_dim5_swap = A[6];
A[6] = A[11];
A[11] = rk_dim5_swap;
rk_dim5_swap = A[7];
A[7] = A[12];
A[12] = rk_dim5_swap;
rk_dim5_swap = A[8];
A[8] = A[13];
A[13] = rk_dim5_swap;
rk_dim5_swap = A[9];
A[9] = A[14];
A[14] = rk_dim5_swap;
rk_dim5_swap = b[1];
b[1] = b[2];
b[2] = rk_dim5_swap;
intSwap = rk_perm[1];
rk_perm[1] = rk_perm[2];
rk_perm[2] = intSwap;
}
else if(fabs(A[16]) > fabs(A[6]) && fabs(A[16]) > fabs(A[11]) && fabs(A[16]) > fabs(A[21])) {
rk_dim5_swap = A[5];
A[5] = A[15];
A[15] = rk_dim5_swap;
rk_dim5_swap = A[6];
A[6] = A[16];
A[16] = rk_dim5_swap;
rk_dim5_swap = A[7];
A[7] = A[17];
A[17] = rk_dim5_swap;
rk_dim5_swap = A[8];
A[8] = A[18];
A[18] = rk_dim5_swap;
rk_dim5_swap = A[9];
A[9] = A[19];
A[19] = rk_dim5_swap;
rk_dim5_swap = b[1];
b[1] = b[3];
b[3] = rk_dim5_swap;
intSwap = rk_perm[1];
rk_perm[1] = rk_perm[3];
rk_perm[3] = intSwap;
}
else if(fabs(A[21]) > fabs(A[6]) && fabs(A[21]) > fabs(A[11]) && fabs(A[21]) > fabs(A[16])) {
rk_dim5_swap = A[5];
A[5] = A[20];
A[20] = rk_dim5_swap;
rk_dim5_swap = A[6];
A[6] = A[21];
A[21] = rk_dim5_swap;
rk_dim5_swap = A[7];
A[7] = A[22];
A[22] = rk_dim5_swap;
rk_dim5_swap = A[8];
A[8] = A[23];
A[23] = rk_dim5_swap;
rk_dim5_swap = A[9];
A[9] = A[24];
A[24] = rk_dim5_swap;
rk_dim5_swap = b[1];
b[1] = b[4];
b[4] = rk_dim5_swap;
intSwap = rk_perm[1];
rk_perm[1] = rk_perm[4];
rk_perm[4] = intSwap;
}

A[11] = -A[11]/A[6];
A[12] += + A[11]*A[7];
A[13] += + A[11]*A[8];
A[14] += + A[11]*A[9];
b[2] += + A[11]*b[1];

A[16] = -A[16]/A[6];
A[17] += + A[16]*A[7];
A[18] += + A[16]*A[8];
A[19] += + A[16]*A[9];
b[3] += + A[16]*b[1];

A[21] = -A[21]/A[6];
A[22] += + A[21]*A[7];
A[23] += + A[21]*A[8];
A[24] += + A[21]*A[9];
b[4] += + A[21]*b[1];

det = + det*A[6];

if(fabs(A[17]) > fabs(A[12]) && fabs(A[17]) > fabs(A[22])) {
rk_dim5_swap = A[10];
A[10] = A[15];
A[15] = rk_dim5_swap;
rk_dim5_swap = A[11];
A[11] = A[16];
A[16] = rk_dim5_swap;
rk_dim5_swap = A[12];
A[12] = A[17];
A[17] = rk_dim5_swap;
rk_dim5_swap = A[13];
A[13] = A[18];
A[18] = rk_dim5_swap;
rk_dim5_swap = A[14];
A[14] = A[19];
A[19] = rk_dim5_swap;
rk_dim5_swap = b[2];
b[2] = b[3];
b[3] = rk_dim5_swap;
intSwap = rk_perm[2];
rk_perm[2] = rk_perm[3];
rk_perm[3] = intSwap;
}
else if(fabs(A[22]) > fabs(A[12]) && fabs(A[22]) > fabs(A[17])) {
rk_dim5_swap = A[10];
A[10] = A[20];
A[20] = rk_dim5_swap;
rk_dim5_swap = A[11];
A[11] = A[21];
A[21] = rk_dim5_swap;
rk_dim5_swap = A[12];
A[12] = A[22];
A[22] = rk_dim5_swap;
rk_dim5_swap = A[13];
A[13] = A[23];
A[23] = rk_dim5_swap;
rk_dim5_swap = A[14];
A[14] = A[24];
A[24] = rk_dim5_swap;
rk_dim5_swap = b[2];
b[2] = b[4];
b[4] = rk_dim5_swap;
intSwap = rk_perm[2];
rk_perm[2] = rk_perm[4];
rk_perm[4] = intSwap;
}

A[17] = -A[17]/A[12];
A[18] += + A[17]*A[13];
A[19] += + A[17]*A[14];
b[3] += + A[17]*b[2];

A[22] = -A[22]/A[12];
A[23] += + A[22]*A[13];
A[24] += + A[22]*A[14];
b[4] += + A[22]*b[2];

det = + det*A[12];

if(fabs(A[23]) > fabs(A[18])) {
rk_dim5_swap = A[15];
A[15] = A[20];
A[20] = rk_dim5_swap;
rk_dim5_swap = A[16];
A[16] = A[21];
A[21] = rk_dim5_swap;
rk_dim5_swap = A[17];
A[17] = A[22];
A[22] = rk_dim5_swap;
rk_dim5_swap = A[18];
A[18] = A[23];
A[23] = rk_dim5_swap;
rk_dim5_swap = A[19];
A[19] = A[24];
A[24] = rk_dim5_swap;
rk_dim5_swap = b[3];
b[3] = b[4];
b[4] = rk_dim5_swap;
intSwap = rk_perm[3];
rk_perm[3] = rk_perm[4];
rk_perm[4] = intSwap;
}

A[23] = -A[23]/A[18];
A[24] += + A[23]*A[19];
b[4] += + A[23]*b[3];

det = + det*A[18];

det = + det*A[24];

det = fabs(det);
acado_solve_dim5_triangular( A, b );
return det;
}

void acado_solve_dim5_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim5_bPerm[0] = b[rk_perm[0]];
rk_dim5_bPerm[1] = b[rk_perm[1]];
rk_dim5_bPerm[2] = b[rk_perm[2]];
rk_dim5_bPerm[3] = b[rk_perm[3]];
rk_dim5_bPerm[4] = b[rk_perm[4]];
rk_dim5_bPerm[1] += A[5]*rk_dim5_bPerm[0];

rk_dim5_bPerm[2] += A[10]*rk_dim5_bPerm[0];
rk_dim5_bPerm[2] += A[11]*rk_dim5_bPerm[1];

rk_dim5_bPerm[3] += A[15]*rk_dim5_bPerm[0];
rk_dim5_bPerm[3] += A[16]*rk_dim5_bPerm[1];
rk_dim5_bPerm[3] += A[17]*rk_dim5_bPerm[2];

rk_dim5_bPerm[4] += A[20]*rk_dim5_bPerm[0];
rk_dim5_bPerm[4] += A[21]*rk_dim5_bPerm[1];
rk_dim5_bPerm[4] += A[22]*rk_dim5_bPerm[2];
rk_dim5_bPerm[4] += A[23]*rk_dim5_bPerm[3];


acado_solve_dim5_triangular( A, rk_dim5_bPerm );
b[0] = rk_dim5_bPerm[0];
b[1] = rk_dim5_bPerm[1];
b[2] = rk_dim5_bPerm[2];
b[3] = rk_dim5_bPerm[3];
b[4] = rk_dim5_bPerm[4];
}



/** Column vector of size: 1 */
static const real_t acado_Ah_mat[ 1 ] = 
{ 2.5000000000000001e-02 };


/* Fixed step size:0.05 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

rk_ttt = 0.0000000000000000e+00;
rk_xxx[5] = rk_eta[50];
rk_xxx[6] = rk_eta[51];
rk_xxx[7] = rk_eta[52];
rk_xxx[8] = rk_eta[53];
rk_xxx[9] = rk_eta[54];
rk_xxx[10] = rk_eta[55];
rk_xxx[11] = rk_eta[56];
rk_xxx[12] = rk_eta[57];
rk_xxx[13] = rk_eta[58];
rk_xxx[14] = rk_eta[59];
rk_xxx[15] = rk_eta[60];
rk_xxx[16] = rk_eta[61];
rk_xxx[17] = rk_eta[62];
rk_xxx[18] = rk_eta[63];
rk_xxx[19] = rk_eta[64];
rk_xxx[20] = rk_eta[65];
rk_xxx[21] = rk_eta[66];
rk_xxx[22] = rk_eta[67];
rk_xxx[23] = rk_eta[68];
rk_xxx[24] = rk_eta[69];
rk_xxx[25] = rk_eta[70];
rk_xxx[26] = rk_eta[71];
rk_xxx[27] = rk_eta[72];
rk_xxx[28] = rk_eta[73];

for (run = 0; run < 2; ++run)
{
if( run > 0 ) {
for (i = 0; i < 5; ++i)
{
rk_diffsPrev2[i * 9] = rk_eta[i * 5 + 5];
rk_diffsPrev2[i * 9 + 1] = rk_eta[i * 5 + 6];
rk_diffsPrev2[i * 9 + 2] = rk_eta[i * 5 + 7];
rk_diffsPrev2[i * 9 + 3] = rk_eta[i * 5 + 8];
rk_diffsPrev2[i * 9 + 4] = rk_eta[i * 5 + 9];
rk_diffsPrev2[i * 9 + 5] = rk_eta[i * 4 + 30];
rk_diffsPrev2[i * 9 + 6] = rk_eta[i * 4 + 31];
rk_diffsPrev2[i * 9 + 7] = rk_eta[i * 4 + 32];
rk_diffsPrev2[i * 9 + 8] = rk_eta[i * 4 + 33];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 5; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 45 ]) );
for (j = 0; j < 5; ++j)
{
tmp_index1 = (run1 * 5) + (j);
rk_A[tmp_index1 * 5] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 45) + (j * 9)];
rk_A[tmp_index1 * 5 + 1] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 45) + (j * 9 + 1)];
rk_A[tmp_index1 * 5 + 2] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 45) + (j * 9 + 2)];
rk_A[tmp_index1 * 5 + 3] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 45) + (j * 9 + 3)];
rk_A[tmp_index1 * 5 + 4] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 45) + (j * 9 + 4)];
if( 0 == run1 ) rk_A[(tmp_index1 * 5) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 5] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 5 + 1] = rk_kkk[run1 + 1] - rk_rhsTemp[1];
rk_b[run1 * 5 + 2] = rk_kkk[run1 + 2] - rk_rhsTemp[2];
rk_b[run1 * 5 + 3] = rk_kkk[run1 + 3] - rk_rhsTemp[3];
rk_b[run1 * 5 + 4] = rk_kkk[run1 + 4] - rk_rhsTemp[4];
}
det = acado_solve_dim5_system( rk_A, rk_b, rk_dim5_perm );
for (j = 0; j < 1; ++j)
{
rk_kkk[j] += rk_b[j * 5];
rk_kkk[j + 1] += rk_b[j * 5 + 1];
rk_kkk[j + 2] += rk_b[j * 5 + 2];
rk_kkk[j + 3] += rk_b[j * 5 + 3];
rk_kkk[j + 4] += rk_b[j * 5 + 4];
}
}
}
for (i = 0; i < 2; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 5; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 5] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 5 + 1] = rk_kkk[run1 + 1] - rk_rhsTemp[1];
rk_b[run1 * 5 + 2] = rk_kkk[run1 + 2] - rk_rhsTemp[2];
rk_b[run1 * 5 + 3] = rk_kkk[run1 + 3] - rk_rhsTemp[3];
rk_b[run1 * 5 + 4] = rk_kkk[run1 + 4] - rk_rhsTemp[4];
}
acado_solve_dim5_system_reuse( rk_A, rk_b, rk_dim5_perm );
for (j = 0; j < 1; ++j)
{
rk_kkk[j] += rk_b[j * 5];
rk_kkk[j + 1] += rk_b[j * 5 + 1];
rk_kkk[j + 2] += rk_b[j * 5 + 2];
rk_kkk[j + 3] += rk_b[j * 5 + 3];
rk_kkk[j + 4] += rk_b[j * 5 + 4];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 5; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 45 ]) );
for (j = 0; j < 5; ++j)
{
tmp_index1 = (run1 * 5) + (j);
rk_A[tmp_index1 * 5] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 45) + (j * 9)];
rk_A[tmp_index1 * 5 + 1] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 45) + (j * 9 + 1)];
rk_A[tmp_index1 * 5 + 2] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 45) + (j * 9 + 2)];
rk_A[tmp_index1 * 5 + 3] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 45) + (j * 9 + 3)];
rk_A[tmp_index1 * 5 + 4] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 45) + (j * 9 + 4)];
if( 0 == run1 ) rk_A[(tmp_index1 * 5) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 5; ++run1)
{
for (i = 0; i < 1; ++i)
{
rk_b[i * 5] = - rk_diffsTemp2[(i * 45) + (run1)];
rk_b[i * 5 + 1] = - rk_diffsTemp2[(i * 45) + (run1 + 9)];
rk_b[i * 5 + 2] = - rk_diffsTemp2[(i * 45) + (run1 + 18)];
rk_b[i * 5 + 3] = - rk_diffsTemp2[(i * 45) + (run1 + 27)];
rk_b[i * 5 + 4] = - rk_diffsTemp2[(i * 45) + (run1 + 36)];
}
if( 0 == run1 ) {
det = acado_solve_dim5_system( rk_A, rk_b, rk_dim5_perm );
}
 else {
acado_solve_dim5_system_reuse( rk_A, rk_b, rk_dim5_perm );
}
for (i = 0; i < 1; ++i)
{
rk_diffK[i] = rk_b[i * 5];
rk_diffK[i + 1] = rk_b[i * 5 + 1];
rk_diffK[i + 2] = rk_b[i * 5 + 2];
rk_diffK[i + 3] = rk_b[i * 5 + 3];
rk_diffK[i + 4] = rk_b[i * 5 + 4];
}
for (i = 0; i < 5; ++i)
{
rk_diffsNew2[(i * 9) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 9) + (run1)] += + rk_diffK[i]*(real_t)5.0000000000000003e-02;
}
}
for (run1 = 0; run1 < 4; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 5; ++j)
{
tmp_index1 = (i * 5) + (j);
tmp_index2 = (run1) + (j * 9);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 45) + (tmp_index2 + 5)];
}
}
acado_solve_dim5_system_reuse( rk_A, rk_b, rk_dim5_perm );
for (i = 0; i < 1; ++i)
{
rk_diffK[i] = rk_b[i * 5];
rk_diffK[i + 1] = rk_b[i * 5 + 1];
rk_diffK[i + 2] = rk_b[i * 5 + 2];
rk_diffK[i + 3] = rk_b[i * 5 + 3];
rk_diffK[i + 4] = rk_b[i * 5 + 4];
}
for (i = 0; i < 5; ++i)
{
rk_diffsNew2[(i * 9) + (run1 + 5)] = + rk_diffK[i]*(real_t)5.0000000000000003e-02;
}
}
rk_eta[0] += + rk_kkk[0]*(real_t)5.0000000000000003e-02;
rk_eta[1] += + rk_kkk[1]*(real_t)5.0000000000000003e-02;
rk_eta[2] += + rk_kkk[2]*(real_t)5.0000000000000003e-02;
rk_eta[3] += + rk_kkk[3]*(real_t)5.0000000000000003e-02;
rk_eta[4] += + rk_kkk[4]*(real_t)5.0000000000000003e-02;
if( run == 0 ) {
for (i = 0; i < 5; ++i)
{
for (j = 0; j < 5; ++j)
{
tmp_index2 = (j) + (i * 5);
rk_eta[tmp_index2 + 5] = rk_diffsNew2[(i * 9) + (j)];
}
for (j = 0; j < 4; ++j)
{
tmp_index2 = (j) + (i * 4);
rk_eta[tmp_index2 + 30] = rk_diffsNew2[(i * 9) + (j + 5)];
}
}
}
else {
for (i = 0; i < 5; ++i)
{
for (j = 0; j < 5; ++j)
{
tmp_index2 = (j) + (i * 5);
rk_eta[tmp_index2 + 5] = + rk_diffsNew2[i * 9]*rk_diffsPrev2[j];
rk_eta[tmp_index2 + 5] += + rk_diffsNew2[i * 9 + 1]*rk_diffsPrev2[j + 9];
rk_eta[tmp_index2 + 5] += + rk_diffsNew2[i * 9 + 2]*rk_diffsPrev2[j + 18];
rk_eta[tmp_index2 + 5] += + rk_diffsNew2[i * 9 + 3]*rk_diffsPrev2[j + 27];
rk_eta[tmp_index2 + 5] += + rk_diffsNew2[i * 9 + 4]*rk_diffsPrev2[j + 36];
}
for (j = 0; j < 4; ++j)
{
tmp_index2 = (j) + (i * 4);
rk_eta[tmp_index2 + 30] = rk_diffsNew2[(i * 9) + (j + 5)];
rk_eta[tmp_index2 + 30] += + rk_diffsNew2[i * 9]*rk_diffsPrev2[j + 5];
rk_eta[tmp_index2 + 30] += + rk_diffsNew2[i * 9 + 1]*rk_diffsPrev2[j + 14];
rk_eta[tmp_index2 + 30] += + rk_diffsNew2[i * 9 + 2]*rk_diffsPrev2[j + 23];
rk_eta[tmp_index2 + 30] += + rk_diffsNew2[i * 9 + 3]*rk_diffsPrev2[j + 32];
rk_eta[tmp_index2 + 30] += + rk_diffsNew2[i * 9 + 4]*rk_diffsPrev2[j + 41];
}
}
}
resetIntegrator = 0;
rk_ttt += 5.0000000000000000e-01;
}
for (i = 0; i < 5; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



