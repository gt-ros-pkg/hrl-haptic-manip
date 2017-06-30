/* Produced by CVXGEN, 2014-01-13 17:28:27 -0500.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: csolve.c. */
/* Description: mex-able file for running cvxgen solver. */
#include "mex.h"
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  int i, j;
  mxArray *xm, *cell, *xm_cell;
  double *src;
  double *dest;
  double *dest_cell;
  int valid_vars;
  int steps;
  int this_var_errors;
  int warned_diags;
  int prepare_for_c = 0;
  int extra_solves;
  const char *status_names[] = {"optval", "gap", "steps", "converged"};
  mwSize dims1x1of1[1] = {1};
  mwSize dims[1];
  const char *var_names[] = {"q_1", "q_2", "q_3", "q_4", "q_5", "q_6", "q_des_cur_1", "q_des_cur_2", "q_des_cur_3", "qd_1", "qd_2", "qd_3", "qd_4", "qd_5", "qd_6", "u_0", "u_1", "u_2", "q", "q_des_cur", "qd", "u"};
  const int num_var_names = 22;
  /* Avoid compiler warnings of unused variables by using a dummy assignment. */
  warned_diags = j = 0;
  extra_solves = 0;
  set_defaults();
  /* Check we got the right number of arguments. */
  if (nrhs == 0)
    mexErrMsgTxt("Not enough arguments: You need to specify at least the parameters.\n");
  if (nrhs > 1) {
    /* Assume that the second argument is the settings. */
    if (mxGetField(prhs[1], 0, "eps") != NULL)
      settings.eps = *mxGetPr(mxGetField(prhs[1], 0, "eps"));
    if (mxGetField(prhs[1], 0, "max_iters") != NULL)
      settings.max_iters = *mxGetPr(mxGetField(prhs[1], 0, "max_iters"));
    if (mxGetField(prhs[1], 0, "refine_steps") != NULL)
      settings.refine_steps = *mxGetPr(mxGetField(prhs[1], 0, "refine_steps"));
    if (mxGetField(prhs[1], 0, "verbose") != NULL)
      settings.verbose = *mxGetPr(mxGetField(prhs[1], 0, "verbose"));
    if (mxGetField(prhs[1], 0, "better_start") != NULL)
      settings.better_start = *mxGetPr(mxGetField(prhs[1], 0, "better_start"));
    if (mxGetField(prhs[1], 0, "verbose_refinement") != NULL)
      settings.verbose_refinement = *mxGetPr(mxGetField(prhs[1], 0,
            "verbose_refinement"));
    if (mxGetField(prhs[1], 0, "debug") != NULL)
      settings.debug = *mxGetPr(mxGetField(prhs[1], 0, "debug"));
    if (mxGetField(prhs[1], 0, "kkt_reg") != NULL)
      settings.kkt_reg = *mxGetPr(mxGetField(prhs[1], 0, "kkt_reg"));
    if (mxGetField(prhs[1], 0, "s_init") != NULL)
      settings.s_init = *mxGetPr(mxGetField(prhs[1], 0, "s_init"));
    if (mxGetField(prhs[1], 0, "z_init") != NULL)
      settings.z_init = *mxGetPr(mxGetField(prhs[1], 0, "z_init"));
    if (mxGetField(prhs[1], 0, "resid_tol") != NULL)
      settings.resid_tol = *mxGetPr(mxGetField(prhs[1], 0, "resid_tol"));
    if (mxGetField(prhs[1], 0, "extra_solves") != NULL)
      extra_solves = *mxGetPr(mxGetField(prhs[1], 0, "extra_solves"));
    else
      extra_solves = 0;
    if (mxGetField(prhs[1], 0, "prepare_for_c") != NULL)
      prepare_for_c = *mxGetPr(mxGetField(prhs[1], 0, "prepare_for_c"));
  }
  valid_vars = 0;
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_bl");
  if (xm == NULL) {
    printf("could not find params.A_bl.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("A_bl must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_bl must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_bl must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_bl must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_bl;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_br");
  if (xm == NULL) {
    printf("could not find params.A_br.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("A_br must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_br must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_br must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_br must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_br;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_tl");
  if (xm == NULL) {
    printf("could not find params.A_tl.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("A_tl must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_tl must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_tl must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_tl must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_tl;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "A_tr");
  if (xm == NULL) {
    printf("could not find params.A_tr.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("A_tr must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter A_tr must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter A_tr must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter A_tr must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.A_tr;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_b1");
  if (xm == NULL) {
    printf("could not find params.B_b1.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("B_b1 must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_b1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_b1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_b1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_b1;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_b2");
  if (xm == NULL) {
    printf("could not find params.B_b2.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("B_b2 must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_b2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_b2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_b2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_b2;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_b3");
  if (xm == NULL) {
    printf("could not find params.B_b3.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("B_b3 must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_b3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_b3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_b3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_b3;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_t1");
  if (xm == NULL) {
    printf("could not find params.B_t1.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("B_t1 must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_t1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_t1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_t1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_t1;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_t2");
  if (xm == NULL) {
    printf("could not find params.B_t2.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("B_t2 must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_t2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_t2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_t2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_t2;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "B_t3");
  if (xm == NULL) {
    printf("could not find params.B_t3.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("B_t3 must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter B_t3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter B_t3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter B_t3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.B_t3;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J");
  if (xm == NULL) {
    printf("could not find params.J.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("J must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J;
      src = mxGetPr(xm);
      for (i = 0; i < 21; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Kd");
  if (xm == NULL) {
    printf("could not find params.Kd.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("Kd must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Kd must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Kd must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Kd must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Kd;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Kp");
  if (xm == NULL) {
    printf("could not find params.Kp.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("Kp must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Kp must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Kp must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Kp must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Kp;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "alpha");
  if (xm == NULL) {
    printf("could not find params.alpha.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("alpha must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter alpha must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter alpha must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter alpha must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.alpha;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "beta");
  if (xm == NULL) {
    printf("could not find params.beta.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("beta must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter beta must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter beta must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter beta must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.beta;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "delta_f_max");
  if (xm == NULL) {
    printf("could not find params.delta_f_max.\n");
  } else {
    if (!((mxGetM(xm) == 10) && (mxGetN(xm) == 1))) {
      printf("delta_f_max must be size (10,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter delta_f_max must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter delta_f_max must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter delta_f_max must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.delta_f_max;
      src = mxGetPr(xm);
      for (i = 0; i < 10; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "delta_q_des");
  if (xm == NULL) {
    printf("could not find params.delta_q_des.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("delta_q_des must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter delta_q_des must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter delta_q_des must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter delta_q_des must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.delta_q_des;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "delta_x_des");
  if (xm == NULL) {
    printf("could not find params.delta_x_des.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("delta_x_des must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter delta_x_des must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter delta_x_des must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter delta_x_des must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.delta_x_des;
      src = mxGetPr(xm);
      for (i = 0; i < 3; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "kappa");
  if (xm == NULL) {
    printf("could not find params.kappa.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("kappa must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter kappa must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter kappa must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter kappa must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.kappa;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "mass");
  if (xm == NULL) {
    printf("could not find params.mass.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 7))) {
      printf("mass must be size (7,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter mass must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter mass must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter mass must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.mass;
      src = mxGetPr(xm);
      for (i = 0; i < 49; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "mu");
  if (xm == NULL) {
    printf("could not find params.mu.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("mu must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter mu must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter mu must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter mu must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.mu;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "n_K_J_all");
  if (xm == NULL) {
    printf("could not find params.n_K_J_all.\n");
  } else {
    if (!((mxGetM(xm) == 10) && (mxGetN(xm) == 7))) {
      printf("n_K_J_all must be size (10,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter n_K_J_all must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter n_K_J_all must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter n_K_J_all must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.n_K_J_all;
      src = mxGetPr(xm);
      for (i = 0; i < 70; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "posture_weight");
  if (xm == NULL) {
    printf("could not find params.posture_weight.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("posture_weight must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter posture_weight must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter posture_weight must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter posture_weight must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.posture_weight;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "q_0");
  if (xm == NULL) {
    printf("could not find params.q_0.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("q_0 must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter q_0 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter q_0 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter q_0 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.q_0;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "q_des_cur_0");
  if (xm == NULL) {
    printf("could not find params.q_des_cur_0.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("q_des_cur_0 must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter q_des_cur_0 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter q_des_cur_0 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter q_des_cur_0 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.q_des_cur_0;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "q_max");
  if (xm == NULL) {
    printf("could not find params.q_max.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("q_max must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter q_max must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter q_max must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter q_max must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.q_max;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "q_min");
  if (xm == NULL) {
    printf("could not find params.q_min.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("q_min must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter q_min must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter q_min must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter q_min must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.q_min;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "qd_0");
  if (xm == NULL) {
    printf("could not find params.qd_0.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("qd_0 must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter qd_0 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter qd_0 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter qd_0 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.qd_0;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tau_cont_sum_0");
  if (xm == NULL) {
    printf("could not find params.tau_cont_sum_0.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("tau_cont_sum_0 must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tau_cont_sum_0 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tau_cont_sum_0 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tau_cont_sum_0 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tau_cont_sum_0;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tau_max_delta_t");
  if (xm == NULL) {
    printf("could not find params.tau_max_delta_t.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("tau_max_delta_t must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tau_max_delta_t must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tau_max_delta_t must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tau_max_delta_t must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tau_max_delta_t;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "torque_max");
  if (xm == NULL) {
    printf("could not find params.torque_max.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("torque_max must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter torque_max must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter torque_max must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter torque_max must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.torque_max;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "torque_min");
  if (xm == NULL) {
    printf("could not find params.torque_min.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("torque_min must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter torque_min must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter torque_min must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter torque_min must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.torque_min;
      src = mxGetPr(xm);
      for (i = 0; i < 7; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "xyz_weight");
  if (xm == NULL) {
    printf("could not find params.xyz_weight.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("xyz_weight must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter xyz_weight must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter xyz_weight must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter xyz_weight must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.xyz_weight;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "zeta");
  if (xm == NULL) {
    printf("could not find params.zeta.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("zeta must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter zeta must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter zeta must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter zeta must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.zeta;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  if (valid_vars != 34) {
    printf("Error: %d parameters are invalid.\n", 34 - valid_vars);
    mexErrMsgTxt("invalid parameters found.");
  }
  if (prepare_for_c) {
    printf("settings.prepare_for_c == 1. thus, outputting for C.\n");
    for (i = 0; i < 1; i++)
      printf("  params.kappa[%d] = %.6g;\n", i, params.kappa[i]);
    for (i = 0; i < 49; i++)
      printf("  params.mass[%d] = %.6g;\n", i, params.mass[i]);
    for (i = 0; i < 7; i++)
      printf("  params.tau_max_delta_t[%d] = %.6g;\n", i, params.tau_max_delta_t[i]);
    for (i = 0; i < 1; i++)
      printf("  params.alpha[%d] = %.6g;\n", i, params.alpha[i]);
    for (i = 0; i < 1; i++)
      printf("  params.posture_weight[%d] = %.6g;\n", i, params.posture_weight[i]);
    for (i = 0; i < 7; i++)
      printf("  params.delta_q_des[%d] = %.6g;\n", i, params.delta_q_des[i]);
    for (i = 0; i < 7; i++)
      printf("  params.q_0[%d] = %.6g;\n", i, params.q_0[i]);
    for (i = 0; i < 1; i++)
      printf("  params.zeta[%d] = %.6g;\n", i, params.zeta[i]);
    for (i = 0; i < 1; i++)
      printf("  params.xyz_weight[%d] = %.6g;\n", i, params.xyz_weight[i]);
    for (i = 0; i < 3; i++)
      printf("  params.delta_x_des[%d] = %.6g;\n", i, params.delta_x_des[i]);
    for (i = 0; i < 21; i++)
      printf("  params.J[%d] = %.6g;\n", i, params.J[i]);
    for (i = 0; i < 1; i++)
      printf("  params.mu[%d] = %.6g;\n", i, params.mu[i]);
    for (i = 0; i < 1; i++)
      printf("  params.beta[%d] = %.6g;\n", i, params.beta[i]);
    for (i = 0; i < 70; i++)
      printf("  params.n_K_J_all[%d] = %.6g;\n", i, params.n_K_J_all[i]);
    for (i = 0; i < 10; i++)
      printf("  params.delta_f_max[%d] = %.6g;\n", i, params.delta_f_max[i]);
    for (i = 0; i < 49; i++)
      printf("  params.A_tl[%d] = %.6g;\n", i, params.A_tl[i]);
    for (i = 0; i < 7; i++)
      printf("  params.qd_0[%d] = %.6g;\n", i, params.qd_0[i]);
    for (i = 0; i < 49; i++)
      printf("  params.A_tr[%d] = %.6g;\n", i, params.A_tr[i]);
    for (i = 0; i < 49; i++)
      printf("  params.B_t1[%d] = %.6g;\n", i, params.B_t1[i]);
    for (i = 0; i < 7; i++)
      printf("  params.q_des_cur_0[%d] = %.6g;\n", i, params.q_des_cur_0[i]);
    for (i = 0; i < 49; i++)
      printf("  params.B_t2[%d] = %.6g;\n", i, params.B_t2[i]);
    for (i = 0; i < 7; i++)
      printf("  params.tau_cont_sum_0[%d] = %.6g;\n", i, params.tau_cont_sum_0[i]);
    for (i = 0; i < 49; i++)
      printf("  params.B_t3[%d] = %.6g;\n", i, params.B_t3[i]);
    for (i = 0; i < 49; i++)
      printf("  params.A_bl[%d] = %.6g;\n", i, params.A_bl[i]);
    for (i = 0; i < 49; i++)
      printf("  params.A_br[%d] = %.6g;\n", i, params.A_br[i]);
    for (i = 0; i < 49; i++)
      printf("  params.B_b1[%d] = %.6g;\n", i, params.B_b1[i]);
    for (i = 0; i < 49; i++)
      printf("  params.B_b2[%d] = %.6g;\n", i, params.B_b2[i]);
    for (i = 0; i < 49; i++)
      printf("  params.B_b3[%d] = %.6g;\n", i, params.B_b3[i]);
    for (i = 0; i < 7; i++)
      printf("  params.q_min[%d] = %.6g;\n", i, params.q_min[i]);
    for (i = 0; i < 7; i++)
      printf("  params.q_max[%d] = %.6g;\n", i, params.q_max[i]);
    for (i = 0; i < 7; i++)
      printf("  params.torque_min[%d] = %.6g;\n", i, params.torque_min[i]);
    for (i = 0; i < 49; i++)
      printf("  params.Kp[%d] = %.6g;\n", i, params.Kp[i]);
    for (i = 0; i < 49; i++)
      printf("  params.Kd[%d] = %.6g;\n", i, params.Kd[i]);
    for (i = 0; i < 7; i++)
      printf("  params.torque_max[%d] = %.6g;\n", i, params.torque_max[i]);
  }
  /* Perform the actual solve in here. */
  steps = solve();
  /* For profiling purposes, allow extra silent solves if desired. */
  settings.verbose = 0;
  for (i = 0; i < extra_solves; i++)
    solve();
  /* Update the status variables. */
  plhs[1] = mxCreateStructArray(1, dims1x1of1, 4, status_names);
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "optval", xm);
  *mxGetPr(xm) = work.optval;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "gap", xm);
  *mxGetPr(xm) = work.gap;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "steps", xm);
  *mxGetPr(xm) = steps;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "converged", xm);
  *mxGetPr(xm) = work.converged;
  /* Extract variable values. */
  plhs[0] = mxCreateStructArray(1, dims1x1of1, num_var_names, var_names);
  /* Create cell arrays for indexed variables. */
  dims[0] = 6;
  cell = mxCreateCellArray(1, dims);
  mxSetField(plhs[0], 0, "q", cell);
  dims[0] = 3;
  cell = mxCreateCellArray(1, dims);
  mxSetField(plhs[0], 0, "q_des_cur", cell);
  dims[0] = 6;
  cell = mxCreateCellArray(1, dims);
  mxSetField(plhs[0], 0, "qd", cell);
  dims[0] = 2;
  cell = mxCreateCellArray(1, dims);
  mxSetField(plhs[0], 0, "u", cell);
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "q_1", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "q");
  mxSetCell(cell, 0, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.q_1;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "q_2", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "q");
  mxSetCell(cell, 1, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.q_2;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "q_3", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "q");
  mxSetCell(cell, 2, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.q_3;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "q_4", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "q");
  mxSetCell(cell, 3, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.q_4;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "q_5", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "q");
  mxSetCell(cell, 4, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.q_5;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "q_6", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "q");
  mxSetCell(cell, 5, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.q_6;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "q_des_cur_1", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "q_des_cur");
  mxSetCell(cell, 0, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.q_des_cur_1;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "q_des_cur_2", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "q_des_cur");
  mxSetCell(cell, 1, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.q_des_cur_2;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "q_des_cur_3", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "q_des_cur");
  mxSetCell(cell, 2, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.q_des_cur_3;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "qd_1", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "qd");
  mxSetCell(cell, 0, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.qd_1;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "qd_2", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "qd");
  mxSetCell(cell, 1, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.qd_2;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "qd_3", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "qd");
  mxSetCell(cell, 2, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.qd_3;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "qd_4", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "qd");
  mxSetCell(cell, 3, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.qd_4;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "qd_5", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "qd");
  mxSetCell(cell, 4, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.qd_5;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "qd_6", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "qd");
  mxSetCell(cell, 5, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.qd_6;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_0", xm);
  dest = mxGetPr(xm);
  src = vars.u_0;
  for (i = 0; i < 7; i++) {
    *dest++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_1", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 0, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_1;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "u_2", xm);
  xm_cell = mxCreateDoubleMatrix(7, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "u");
  mxSetCell(cell, 1, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.u_2;
  for (i = 0; i < 7; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
}
