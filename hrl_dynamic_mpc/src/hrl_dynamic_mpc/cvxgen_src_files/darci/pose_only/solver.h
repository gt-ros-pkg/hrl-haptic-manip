/* Produced by CVXGEN, 2014-01-15 15:37:12 -0500.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double kappa[1];
  double mass[49];
  double tau_max_delta_t[7];
  double alpha[1];
  double delta_x_d[3];
  double J[21];
  double q_0[7];
  double mu[1];
  double beta[1];
  double n_K_J_all[70];
  double delta_f_max[10];
  double zeta[1];
  double delta_rate_f_max[10];
  double A_tl[49];
  double qd_0[7];
  double A_tr[49];
  double B_t1[49];
  double q_des_cur_0[7];
  double B_t2[49];
  double tau_cont_sum_0[7];
  double B_t3[49];
  double A_bl[49];
  double A_br[49];
  double B_b1[49];
  double B_b2[49];
  double B_b3[49];
  double q_min[7];
  double q_max[7];
  double torque_min[7];
  double Kp[49];
  double Kd[49];
  double torque_max[7];
  double *q[1];
  double *qd[1];
  double *q_des_cur[1];
  double *tau_cont_sum[1];
} Params;
typedef struct Vars_t {
  double *t_01; /* 7 rows. */
  double *t_02; /* 7 rows. */
  double *t_03; /* 7 rows. */
  double *t_04; /* 3 rows. */
  double *u_0; /* 7 rows. */
  double *u_1; /* 7 rows. */
  double *u_2; /* 7 rows. */
  double *t_05; /* 10 rows. */
  double *t_06; /* 10 rows. */
  double *t_07; /* 10 rows. */
  double *t_08; /* 10 rows. */
  double *t_09; /* 10 rows. */
  double *t_10; /* 10 rows. */
  double *t_11; /* 10 rows. */
  double *t_12; /* 10 rows. */
  double *t_13; /* 10 rows. */
  double *t_14; /* 7 rows. */
  double *qd_1; /* 7 rows. */
  double *t_15; /* 7 rows. */
  double *qd_2; /* 7 rows. */
  double *t_16; /* 7 rows. */
  double *qd_3; /* 7 rows. */
  double *q_1; /* 7 rows. */
  double *q_2; /* 7 rows. */
  double *q_3; /* 7 rows. */
  double *q_4; /* 7 rows. */
  double *q_5; /* 7 rows. */
  double *q_6; /* 7 rows. */
  double *q_des_cur_1; /* 7 rows. */
  double *q_des_cur_2; /* 7 rows. */
  double *qd_4; /* 7 rows. */
  double *q_des_cur_3; /* 7 rows. */
  double *qd_5; /* 7 rows. */
  double *qd_6; /* 7 rows. */
  double *qd[7];
  double *q[7];
  double *u[3];
  double *q_des_cur[4];
} Vars;
typedef struct Workspace_t {
  double h[390];
  double s_inv[390];
  double s_inv_z[390];
  double b[108];
  double q[261];
  double rhs[1149];
  double x[1149];
  double *s;
  double *z;
  double *y;
  double lhs_aff[1149];
  double lhs_cc[1149];
  double buffer[1149];
  double buffer2[1149];
  double KKT[5577];
  double L[7722];
  double d[1149];
  double v[1149];
  double d_inv[1149];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
