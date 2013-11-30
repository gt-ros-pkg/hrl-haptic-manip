/* Produced by CVXGEN, 2012-03-08 11:12:44 -0800.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
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
  double delta_x_d[3];
  double J[21];
  double x_0[7];
  double I[9];
  double KP_t_KP[49];
  double B[49];
  double q_min[1];
  double q_max[1];
  double f_min[100];
  double n_K_ci_J_ci[700];
  double f_max[100];
  double u_min[7];
  double u_max[7];

  double *x[1];
} Params;

typedef struct Vars_t {
  double *t_01; /* 3 rows. */
  double *x_1; /* 7 rows. */
  double *u_0; /* 7 rows. */

  double *x[2];
  double *u[1];
} Vars;

typedef struct Workspace_t {
  double h[228];
  double s_inv[228];
  double s_inv_z[228];
  double b[10];
  double q[17];
  double rhs[483];
  double x[483];
  double *s;
  double *z;
  double *y;
  double lhs_aff[483];
  double lhs_cc[483];
  double buffer[483];
  double buffer2[483];

  double KKT[2226];
  double L[1808];
  double d[483];
  double v[483];
  double d_inv[483];

  double gap;
  double optval;

  double ineq_resid_squared;
  double eq_resid_squared;

  double block_33[1];

  /* Pre-op symbols. */
  double quad_642343022592[1];

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

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexing(void);
void set_start(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

#endif
