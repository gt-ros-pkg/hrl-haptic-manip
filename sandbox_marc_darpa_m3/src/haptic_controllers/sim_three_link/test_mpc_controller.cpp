/* Produced by CVXGEN, 2012-03-10 20:34:58 -0800.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */



#include "solver.h"

/* #include "hrl_haptic_manipulation_in_clutter_msgs/SkinContact.h" */
/* #include "hrl_haptic_manipulation_in_clutter_msgs/BodyDraw.h" */
/* #include "hrl_haptic_manipulation_in_clutter_msgs/TaxelArray.h" */
/* #include "hrl_haptic_manipulation_in_clutter_msgs/MechanicalImpedanceParams.h" */
//#include "roslib/Clock.h"
/* #include <tf/transform_broadcaster.h>   */
/* #include "rosgraph_msgs/Clock.h" */
/* #include <cmath> */
/* #include <vector> */
/* #include <string> */
/* #include <iostream> */
/* #include <set> */
/* #include <algorithm> */
/* #include <functional> */
/* #include <ode/ode.h> */
/* #include <sstream> */

Vars vars;
Params params;
Workspace work;
Settings settings;
ros::Publisher jep_pub;
ros::Publisher predicted_joint_angles_pub;
ros::ServiceServer service;
hrl_msgs::FloatArrayBare delta_jep;
hrl_msgs::FloatArrayBare predicted_joint_angles;

int main(int argc, char **argv)
{
  std::cerr<< "It's Running ..." << std::endl;
  ros::init(argc, argv, "fast_opt_mpc");
  ros::NodeHandle n;
  delta_jep.data.push_back(0.0);
  delta_jep.data.push_back(0.0);
  delta_jep.data.push_back(0.0);
  predicted_joint_angles.data.push_back(0.0);
  predicted_joint_angles.data.push_back(0.0);
  predicted_joint_angles.data.push_back(0.0);

  jep_pub = n.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/delta_jep_mpc_fast", 100);
  predicted_joint_angles_pub = n.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/predicted_joint_angles", 100);
  service = n.advertiseService("get_mpc_fast", get_delta_phi);

  set_defaults();
  setup_indexing();

  settings.verbose = 0;

  ros::spin();

  return 0;
}

bool get_delta_phi(sandbox_marc_darpa_m3::ServiceBasedMPC::Request &req,
		   sandbox_marc_darpa_m3::ServiceBasedMPC::Response &res )
{
  set_data(req);
  solve();

  if (work.converged == 1)
    {
      for (int ii = 0; ii < 3; ii++)
	{

	  delta_jep.data[ii] = vars.u[0][ii];
	  predicted_joint_angles.data[ii] = vars.x[1][ii];
	  res.delta_phi_opt[ii] = vars.u[0][ii];
	  res.predicted_joint_angles[ii] = vars.x[1][ii];
	}
      jep_pub.publish(delta_jep);
      predicted_joint_angles_pub.publish(predicted_joint_angles);
    }
  else
    {
      std::cerr << "it didn't converge!!"<< std::endl;
      for (int ii = 0; ii < 3; ii++)
	{
	  delta_jep.data[ii] = 0.0;
	  res.delta_phi_opt[ii] = 0.0;
	}
      jep_pub.publish(delta_jep);
      predicted_joint_angles_pub.publish(predicted_joint_angles);
    }

  return true;
}

void dataCallback(const sandbox_marc_darpa_m3::OptTest msg)
{
  std::cerr << "got into callback" << std::endl;
}

void set_data(sandbox_marc_darpa_m3::ServiceBasedMPC::Request &req)
{
  params.alpha[0] = req.alpha;
  params.beta[0] = req.beta;
  params.gamma[0] = req.gamma;

  // std::cerr << "size of delta_x_d :" << req.delta_x_d.size() << std::endl;
  for (uint ii=0; ii < req.delta_x_d.size(); ii++)
    {
      params.delta_x_d[ii] = req.delta_x_d[ii];
      // std::cerr << "delta_x_d_" <<  ii << ":\t" << req.delta_x_d[ii] << std::endl;
    }

  // std::cerr << "size of J :" << req.J.size() << std::endl;
  for (uint ii=0; ii < req.J.size(); ii++)
    {
      params.J[ii] = req.J[ii];
      // std::cerr << "J_" <<  ii << ":\t" << req.J[ii] << std::endl;
    }

  // std::cerr << "size of x_0 :" << req.x_0.size() << std::endl;
  for (uint ii=0; ii < req.x_0.size(); ii++)
    {
      params.x_0[ii] = req.x_0[ii];
      // std::cerr << "x_0_" <<  ii << ":\t" << req.x_0[ii] << std::endl;
    }

  // std::cerr << "size of KP_t_KP :" << req.KP_t_KP.size() << std::endl;
  for (uint ii=0; ii < req.KP_t_KP.size(); ii++)
    {
      params.KP_t_KP[ii] = req.KP_t_KP[ii];
      // std::cerr << "KP_t_KP_" <<  ii << ":\t" << req.KP_t_KP[ii] << std::endl;
    }


  // std::cerr << "size of B :" << req.B.size() << std::endl;
  for (uint ii=0; ii < req.B.size(); ii++)
    {
      params.B[ii] = req.B[ii];
      // std::cerr << "B_" <<  ii << ":\t" << req.B[ii] << std::endl;
    }

  // std::cerr << "sizes of q_min \t q_max \t u_min \t u_max :"  << std::endl;
  // std::cerr << req.q_min.size() << "\t" << req.q_max.size() << "\t" << req.u_min.size() << "\t" << req.u_max.size() << "\t" << std::endl;
  for (uint ii=0; ii < req.u_min.size(); ii++)
    {
      // std::cerr << req.q_min[ii] << "\t" << req.q_max[ii] << "\t" << req.u_min[ii] << "\t" << req.u_max[ii]<<std::endl;
      params.q_min[ii] = req.q_min[ii];
      params.q_max[ii] = req.q_max[ii];
      params.u_min[ii] = req.u_min[ii];
      params.u_max[ii] = req.u_max[ii];
    }

  // std::cerr << "size of f_min :" << req.f_min.size() << std::endl;
  // std::cerr << "size of f_max :" << req.f_max.size() << std::endl;
  for (uint ii=0; ii < 150; ii++)
    {
      params.desired_force_decrease[ii] = req.desired_force_decrease[ii];
      params.f_min[ii] = req.f_min[ii];
      params.f_max[ii] = req.f_max[ii];
      // std::cerr << "f_min_" <<  ii << ":\t" << req.f_min[ii] << std::endl;
      // std::cerr << "f_max_" <<  ii << ":\t" << req.f_max[ii] << std::endl;
    }

  // std::cerr << "size of n_K_ci_J_ci :" << req.n_K_ci_J_ci.size() << std::endl;
  for (uint ii=0; ii < 450; ii++)
    {
      params.n_K_ci_J_ci[ii] = req.n_K_ci_J_ci[ii];
      params.n_K_ci_J_ci_max[ii] = req.n_K_ci_J_ci_max[ii];

      // std::cerr << "n_K_ci_J_ci_" <<  ii << ":\t" << req.n_K_ci_J_ci[ii] << std::endl;
      // std::cerr << "n_K_ci_J_ci_max" <<  ii << ":\t" << req.n_K_ci_J_ci_max[ii] << std::endl;
    }
}


void get_data(sandbox_marc_darpa_m3::OptTestCall srv)
{

  std::cerr << "tried to load unavailable data" << std::endl;
  // for (uint ii=0; ii < srv.response.delta_x_d.size(); ii++)
  //   {
  //     params.delta_x_d[ii] = srv.response.delta_x_d[ii];
  //   }


  // for (uint ii=0; ii < srv.response.J.size(); ii++)
  //   {
  //     params.J[ii] = srv.response.J[ii];
  //   }


  // for (uint ii=0; ii < srv.response.x_0.size(); ii++)
  //   {
  //     params.x_0[ii] = srv.response.x_0[ii];
  //   }


  // // for (uint ii=0; ii < srv.response.I.size(); ii++)
  // //   {
  // //     params.I[ii] = srv.response.I[ii];
  // //   }

  // for (uint ii=0; ii < srv.response.KP_t_KP.size(); ii++)
  //   {
  //     params.KP_t_KP[ii] = srv.response.KP_t_KP[ii];
  //   }


  // for (uint ii=0; ii < srv.response.B.size(); ii++)
  //   {
  //     params.B[ii] = srv.response.B[ii];
  //   }


  // for (uint ii=0; ii < srv.response.q_min.size(); ii++)
  //   {
  //     params.q_min[ii] = srv.response.q_min[ii];
  //     params.q_max[ii] = srv.response.q_max[ii];
  //     params.u_min[ii] = srv.response.u_min[ii];
  //     params.u_max[ii] = srv.response.u_max[ii];
  //   }


  // for (uint ii=0; ii < 150; ii++)
  //   {
  //     if (ii < srv.response.f_min.size())
  // 	{
  // 	  params.f_min[ii] = srv.response.f_min[ii];
  // 	  params.f_max[ii] = srv.response.f_max[ii];
  // 	}
  //     else
  // 	{
  // 	  params.f_min[ii] = 0.0;
  // 	  params.f_max[ii] = 0.0;
  // 	}	
  //   }

  // for (uint ii=0; ii < 450; ii++)
  //   {
  //     if (ii < srv.response.n_K_ci_J_ci.size())
  // 	{
  // 	  params.n_K_ci_J_ci[ii] = srv.response.n_K_ci_J_ci[ii];
  // 	}
  //     else
  // 	{
  // 	  params.n_K_ci_J_ci[ii] = 0.0;
  // 	}	
  //   }
}


void load_default_data(void) {
  std::cerr << "nothing here now" << std::endl;
//   params.delta_x_d[0] = 0.2;
//   params.delta_x_d[1] = 0.1;
//   // params.delta_x_d[2] = -0.1;
//   params.J[0] = 1.0;
//   params.J[1] = 0.0;
//   params.J[2] = 0.0;
//   params.J[3] = 0.0;
//   params.J[4] = 1.0;
//   params.J[5] = 0.0;
//   // params.J[6] = 0.0;
//   // params.J[7] = 0.0;
//   // params.J[8] = 1.0;
//   params.x_0[0] = 0.496576190459043;
//   params.x_0[1] = 0.886050869408099;
//   params.x_0[2] = -0.705019607920525;
//   /* Make this a diagonal PSD matrix, even though it's not diagonal. */
//   // params.I[0] = 1.0;
//   // params.I[3] = 0;
//   // params.I[6] = 0;
//   // params.I[1] = 0;
//   // params.I[4] = 1.0;
//   // params.I[7] = 0;
//   // params.I[2] = 0;
//   // params.I[5] = 0;
//   // params.I[8] = 1.0;
//   /* Make this a diagonal PSD matrix, even though it's not diagonal. */
//   params.KP_t_KP[0] = 900;
//   params.KP_t_KP[3] = 0;
//   params.KP_t_KP[6] = 0;
//   params.KP_t_KP[1] = 0;
//   params.KP_t_KP[4] = 200;
//   params.KP_t_KP[7] = 0;
//   params.KP_t_KP[2] = 0;
//   params.KP_t_KP[5] = 0;
//   params.KP_t_KP[8] = 90;
//   params.B[0] = 1.0;
//   params.B[1] = 0.0;
//   params.B[2] = 0.0;
//   params.B[3] = 0.0;
//   params.B[4] = 1.0;
//   params.B[5] = 0.0;
//   params.B[6] = 0.0;
//   params.B[7] = 0.0;
//   params.B[8] = 1.0;
//   params.q_min[0] = -1.54;
//   params.q_min[1] = -1.54;
//   params.q_min[2] = -1.54;
//   params.q_max[0] = 1.54;
//   params.q_max[1] = 1.54;
//   params.q_max[2] = 1.54;
//   params.f_min[0] = -5.0;
//   params.f_min[1] = -5.0;
//   params.f_min[2] = -5.0;
//   params.f_min[3] = -5.0;
//   params.f_min[4] = -5.0;
//   params.f_min[5] = -5.0;
//   params.f_min[6] = -5.0;
//   params.f_min[7] = -5.0;
//   params.f_min[8] = -5.0;
//   params.f_min[9] = -5.0;
//   params.f_min[10] = -5.0;
//   params.f_min[11] = -5.0;
//   params.f_min[12] = -5.0;
//   params.f_min[13] = -5.0;
//   params.f_min[14] = -5.0;
//   params.f_min[15] = -5.0;
//   params.f_min[16] = -5.0;
//   params.f_min[17] = -5.0;
//   params.f_min[18] = -5.0;
//   params.f_min[19] = -5.0;
//   params.f_min[20] = 0.0;
//   params.f_min[21] = 0.0;
//   params.f_min[22] = 0.0;
//   params.f_min[23] = 0.0;
//   params.f_min[24] = 0.0;
//   params.f_min[25] = 0.0;
//   params.f_min[26] = 0.0;
//   params.f_min[27] = 0.0;
//   params.f_min[28] = 0.0;
//   params.f_min[29] = 0.0;
//   params.f_min[30] = 0.0;
//   params.f_min[31] = 0.0;
//   params.f_min[32] = 0.0;
//   params.f_min[33] = 0.0;
//   params.f_min[34] = 0.0;
//   params.f_min[35] = 0.0;
//   params.f_min[36] = 0.0;
//   params.f_min[37] = 0.0;
//   params.f_min[38] = 0.0;
//   params.f_min[39] = 0.0;
//   params.f_min[40] = 0.0;
//   params.f_min[41] = 0.0;
//   params.f_min[42] = 0.0;
//   params.f_min[43] = 0.0;
//   params.f_min[44] = 0.0;
//   params.f_min[45] = 0.0;
//   params.f_min[46] = 0.0;
//   params.f_min[47] = 0.0;
//   params.f_min[48] = 0.0;
//   params.f_min[49] = 0.0;
//   params.f_min[50] = 0.0;
//   params.f_min[51] = 0.0;
//   params.f_min[52] = 0.0;
//   params.f_min[53] = 0.0;
//   params.f_min[54] = 0.0;
//   params.f_min[55] = 0.0;
//   params.f_min[56] = 0.0;
//   params.f_min[57] = 0.0;
//   params.f_min[58] = 0.0;
//   params.f_min[59] = 0.0;
//   params.f_min[60] = 0.0;
//   params.f_min[61] = 0.0;
//   params.f_min[62] = 0.0;
//   params.f_min[63] = 0.0;
//   params.f_min[64] = 0.0;
//   params.f_min[65] = 0.0;
//   params.f_min[66] = 0.0;
//   params.f_min[67] = 0.0;
//   params.f_min[68] = 0.0;
//   params.f_min[69] = 0.0;
//   params.f_min[70] = 0.0;
//   params.f_min[71] = 0.0;
//   params.f_min[72] = 0.0;
//   params.f_min[73] = 0.0;
//   params.f_min[74] = 0.0;
//   params.f_min[75] = 0.0;
//   params.f_min[76] = 0.0;
//   params.f_min[77] = 0.0;
//   params.f_min[78] = 0.0;
//   params.f_min[79] = 0.0;
//   params.f_min[80] = 0.0;
//   params.f_min[81] = 0.0;
//   params.f_min[82] = 0.0;
//   params.f_min[83] = 0.0;
//   params.f_min[84] = 0.0;
//   params.f_min[85] = 0.0;
//   params.f_min[86] = 0.0;
//   params.f_min[87] = 0.0;
//   params.f_min[88] = 0.0;
//   params.f_min[89] = 0.0;
//   params.f_min[90] = 0.0;
//   params.f_min[91] = 0.0;
//   params.f_min[92] = 0.0;
//   params.f_min[93] = 0.0;
//   params.f_min[94] = 0.0;
//   params.f_min[95] = 0.0;
//   params.f_min[96] = 0.0;
//   params.f_min[97] = 0.0;
//   params.f_min[98] = 0.0;
//   params.f_min[99] = 0.0;
//   params.f_min[100] =0.0;
//   params.f_min[101] =0.0;
//   params.f_min[102] =0.0;
//   params.f_min[103] =0.0;
//   params.f_min[104] =0.0;
//   params.f_min[105] =0.0;
//   params.f_min[106] =0.0;
//   params.f_min[107] =0.0;
//   params.f_min[108] =0.0;
//   params.f_min[109] =0.0;
//   params.f_min[110] =0.0;
//   params.f_min[111] =0.0;
//   params.f_min[112] =0.0;
//   params.f_min[113] =0.0;
//   params.f_min[114] =0.0;
//   params.f_min[115] =0.0;
//   params.f_min[116] =0.0;
//   params.f_min[117] =0.0;
//   params.f_min[118] =0.0;
//   params.f_min[119] =0.0;
//   params.f_min[120] =0.0;
//   params.f_min[121] =0.0;
//   params.f_min[122] =0.0;
//   params.f_min[123] =0.0;
//   params.f_min[124] =0.0;
//   params.f_min[125] =0.0;
//   params.f_min[126] =0.0;
//   params.f_min[127] =0.0;
//   params.f_min[128] =0.0;
//   params.f_min[129] =0.0;
//   params.f_min[130] =0.0;
//   params.f_min[131] =0.0;
//   params.f_min[132] =0.0;
//   params.f_min[133] =0.0;
//   params.f_min[134] =0.0;
//   params.f_min[135] =0.0;
//   params.f_min[136] =0.0;
//   params.f_min[137] =0.0;
//   params.f_min[138] =0.0;
//   params.f_min[139] =0.0;
//   params.f_min[140] =0.0;
//   params.f_min[141] =0.0;
//   params.f_min[142] =0.0;
//   params.f_min[143] =0.0;
//   params.f_min[144] =0.0;
//   params.f_min[145] =0.0;
//   params.f_min[146] =0.0;
//   params.f_min[147] =0.0;
//   params.f_min[148] =0.0;
//   params.f_min[149] =0.0;
//   params.n_K_ci_J_ci[0] = 0.33;
//   params.n_K_ci_J_ci[1] = 0.33;
//   params.n_K_ci_J_ci[2] = 0.33;
//   params.n_K_ci_J_ci[3] = 0.33;
//   params.n_K_ci_J_ci[4] = 0.33;
//   params.n_K_ci_J_ci[5] = 0.33;
//   params.n_K_ci_J_ci[6] = 0.33;
//   params.n_K_ci_J_ci[7] = 0.33;
//   params.n_K_ci_J_ci[8] = 0.33;
//   params.n_K_ci_J_ci[9] = 0.33;
//   params.n_K_ci_J_ci[10] =0.33;
//   params.n_K_ci_J_ci[11] =0.33;
//   params.n_K_ci_J_ci[12] =0.33;
//   params.n_K_ci_J_ci[13] =0.33;
//   params.n_K_ci_J_ci[14] =0.33;
//   params.n_K_ci_J_ci[15] =0.33;
//   params.n_K_ci_J_ci[16] =0.33;
//   params.n_K_ci_J_ci[17] =0.33;
//   params.n_K_ci_J_ci[18] =0.33;
//   params.n_K_ci_J_ci[19] =0.33;
//   params.n_K_ci_J_ci[20] =0.33;
//   params.n_K_ci_J_ci[21] =0.33;
//   params.n_K_ci_J_ci[22] =0.33;
//   params.n_K_ci_J_ci[23] =0.33;
//   params.n_K_ci_J_ci[24] =0.33;
//   params.n_K_ci_J_ci[25] =0.33;
//   params.n_K_ci_J_ci[26] =0.33;
//   params.n_K_ci_J_ci[27] =0.33;
//   params.n_K_ci_J_ci[28] =0.33;
//   params.n_K_ci_J_ci[29] =0.33;
//   params.n_K_ci_J_ci[30] =0.33;
//   params.n_K_ci_J_ci[31] =0.33;
//   params.n_K_ci_J_ci[32] =0.33;
//   params.n_K_ci_J_ci[33] =0.33;
//   params.n_K_ci_J_ci[34] =0.33;
//   params.n_K_ci_J_ci[35] =0.33;
//   params.n_K_ci_J_ci[36] =0.33;
//   params.n_K_ci_J_ci[37] =0.33;
//   params.n_K_ci_J_ci[38] =0.33;
//   params.n_K_ci_J_ci[39] =0.33;
//   params.n_K_ci_J_ci[40] =0.33;
//   params.n_K_ci_J_ci[41] =0.33;
//   params.n_K_ci_J_ci[42] =0.33;
//   params.n_K_ci_J_ci[43] =0.33;
//   params.n_K_ci_J_ci[44] =0.33;
//   params.n_K_ci_J_ci[45] =0.33;
//   params.n_K_ci_J_ci[46] =0.33;
//   params.n_K_ci_J_ci[47] =0.33;
//   params.n_K_ci_J_ci[48] =0.33;
//   params.n_K_ci_J_ci[49] =0.33;
//   params.n_K_ci_J_ci[50] =0.33;
//   params.n_K_ci_J_ci[51] =0.33;
//   params.n_K_ci_J_ci[52] =0.33;
//   params.n_K_ci_J_ci[53] =0.33;
//   params.n_K_ci_J_ci[54] =0.33;
//   params.n_K_ci_J_ci[55] =0.33;
//   params.n_K_ci_J_ci[56] =0.33;
//   params.n_K_ci_J_ci[57] =0.33;
//   params.n_K_ci_J_ci[58] =0.33;
//   params.n_K_ci_J_ci[59] =0.33;
//   params.n_K_ci_J_ci[60] =0.0;
//   params.n_K_ci_J_ci[61] =0.0;
//   params.n_K_ci_J_ci[62] =0.0;
//   params.n_K_ci_J_ci[63] =0.0;
//   params.n_K_ci_J_ci[64] =0.0;
//   params.n_K_ci_J_ci[65] =0.0;
//   params.n_K_ci_J_ci[66] =0.0;
//   params.n_K_ci_J_ci[67] =0.0;
//   params.n_K_ci_J_ci[68] =0.0;
//   params.n_K_ci_J_ci[69] =0.0;
//   params.n_K_ci_J_ci[70] =0.0;
//   params.n_K_ci_J_ci[71] =0.0;
//   params.n_K_ci_J_ci[72] =0.0;
//   params.n_K_ci_J_ci[73] =0.0;
//   params.n_K_ci_J_ci[74] =0.0;
//   params.n_K_ci_J_ci[75] =0.0;
//   params.n_K_ci_J_ci[76] =0.0;
//   params.n_K_ci_J_ci[77] =0.0;
//   params.n_K_ci_J_ci[78] =0.0;
//   params.n_K_ci_J_ci[79] =0.0;
//   params.n_K_ci_J_ci[80] =0.0;
//   params.n_K_ci_J_ci[81] =0.0;
//   params.n_K_ci_J_ci[82] =0.0;
//   params.n_K_ci_J_ci[83] =0.0;
//   params.n_K_ci_J_ci[84] =0.0;
//   params.n_K_ci_J_ci[85] =0.0;
//   params.n_K_ci_J_ci[86] =0.0;
//   params.n_K_ci_J_ci[87] =0.0;
//   params.n_K_ci_J_ci[88] =0.0;
//   params.n_K_ci_J_ci[89] =0.0;
//   params.n_K_ci_J_ci[90] =0.0;
//   params.n_K_ci_J_ci[91] =0.0;
//   params.n_K_ci_J_ci[92] =0.0;
//   params.n_K_ci_J_ci[93] =0.0;
//   params.n_K_ci_J_ci[94] =0.0;
//   params.n_K_ci_J_ci[95] =0.0;
//   params.n_K_ci_J_ci[96] =0.0;
//   params.n_K_ci_J_ci[97] =0.0;
//   params.n_K_ci_J_ci[98] =0.0;
//   params.n_K_ci_J_ci[99] =0.0;
//   params.n_K_ci_J_ci[100] = 0.0;
//   params.n_K_ci_J_ci[101] = 0.0;
//   params.n_K_ci_J_ci[102] = 0.0;
//   params.n_K_ci_J_ci[103] = 0.0;
//   params.n_K_ci_J_ci[104] = 0.0;
//   params.n_K_ci_J_ci[105] = 0.0;
//   params.n_K_ci_J_ci[106] = 0.0;
//   params.n_K_ci_J_ci[107] = 0.0;
//   params.n_K_ci_J_ci[108] = 0.0;
//   params.n_K_ci_J_ci[109] = 0.0;
//   params.n_K_ci_J_ci[110] = 0.0;
//   params.n_K_ci_J_ci[111] = 0.0;
//   params.n_K_ci_J_ci[112] = 0.0;
//   params.n_K_ci_J_ci[113] = 0.0;
//   params.n_K_ci_J_ci[114] = 0.0;
//   params.n_K_ci_J_ci[115] = 0.0;
//   params.n_K_ci_J_ci[116] = 0.0;
//   params.n_K_ci_J_ci[117] = 0.0;
//   params.n_K_ci_J_ci[118] = 0.0;
//   params.n_K_ci_J_ci[119] = 0.0;
//   params.n_K_ci_J_ci[120] = 0.0;
//   params.n_K_ci_J_ci[121] = 0.0;
//   params.n_K_ci_J_ci[122] = 0.0;
//   params.n_K_ci_J_ci[123] = 0.0;
//   params.n_K_ci_J_ci[124] = 0.0;
//   params.n_K_ci_J_ci[125] = 0.0;
//   params.n_K_ci_J_ci[126] = 0.0;
//   params.n_K_ci_J_ci[127] = 0.0;
//   params.n_K_ci_J_ci[128] = 0.0;
//   params.n_K_ci_J_ci[129] = 0.0;
//   params.n_K_ci_J_ci[130] = 0.0;
//   params.n_K_ci_J_ci[131] = 0.0;
//   params.n_K_ci_J_ci[132] = 0.0;
//   params.n_K_ci_J_ci[133] = 0.0;
//   params.n_K_ci_J_ci[134] = 0.0;
//   params.n_K_ci_J_ci[135] = 0.0;
//   params.n_K_ci_J_ci[136] = 0.0;
//   params.n_K_ci_J_ci[137] = 0.0;
//   params.n_K_ci_J_ci[138] = 0.0;
//   params.n_K_ci_J_ci[139] = 0.0;
//   params.n_K_ci_J_ci[140] = 0.0;
//   params.n_K_ci_J_ci[141] = 0.0;
//   params.n_K_ci_J_ci[142] = 0.0;
//   params.n_K_ci_J_ci[143] = 0.0;
//   params.n_K_ci_J_ci[144] = 0.0;
//   params.n_K_ci_J_ci[145] = 0.0;
//   params.n_K_ci_J_ci[146] = 0.0;
//   params.n_K_ci_J_ci[147] = 0.0;
//   params.n_K_ci_J_ci[148] = 0.0;
//   params.n_K_ci_J_ci[149] = 0.0;
//   params.n_K_ci_J_ci[150] = 0.0;
//   params.n_K_ci_J_ci[151] = 0.0;
//   params.n_K_ci_J_ci[152] = 0.0;
//   params.n_K_ci_J_ci[153] = 0.0;
//   params.n_K_ci_J_ci[154] = 0.0;
//   params.n_K_ci_J_ci[155] = 0.0;
//   params.n_K_ci_J_ci[156] = 0.0;
//   params.n_K_ci_J_ci[157] = 0.0;
//   params.n_K_ci_J_ci[158] = 0.0;
//   params.n_K_ci_J_ci[159] = 0.0;
//   params.n_K_ci_J_ci[160] = 0.0;
//   params.n_K_ci_J_ci[161] = 0.0;
//   params.n_K_ci_J_ci[162] = 0.0;
//   params.n_K_ci_J_ci[163] = 0.0;
//   params.n_K_ci_J_ci[164] = 0.0;
//   params.n_K_ci_J_ci[165] = 0.0;
//   params.n_K_ci_J_ci[166] = 0.0;
//   params.n_K_ci_J_ci[167] = 0.0;
//   params.n_K_ci_J_ci[168] = 0.0;
//   params.n_K_ci_J_ci[169] = 0.0;
//   params.n_K_ci_J_ci[170] = 0.0;
//   params.n_K_ci_J_ci[171] = 0.0;
//   params.n_K_ci_J_ci[172] = 0.0;
//   params.n_K_ci_J_ci[173] = 0.0;
//   params.n_K_ci_J_ci[174] = 0.0;
//   params.n_K_ci_J_ci[175] = 0.0;
//   params.n_K_ci_J_ci[176] = 0.0;
//   params.n_K_ci_J_ci[177] = 0.0;
//   params.n_K_ci_J_ci[178] = 0.0;
//   params.n_K_ci_J_ci[179] = 0.0;
//   params.n_K_ci_J_ci[180] = 0.0;
//   params.n_K_ci_J_ci[181] = 0.0;
//   params.n_K_ci_J_ci[182] = 0.0;
//   params.n_K_ci_J_ci[183] = 0.0;
//   params.n_K_ci_J_ci[184] = 0.0;
//   params.n_K_ci_J_ci[185] = 0.0;
//   params.n_K_ci_J_ci[186] = 0.0;
//   params.n_K_ci_J_ci[187] = 0.0;
//   params.n_K_ci_J_ci[188] = 0.0;
//   params.n_K_ci_J_ci[189] = 0.0;
//   params.n_K_ci_J_ci[190] = 0.0;
//   params.n_K_ci_J_ci[191] = 0.0;
//   params.n_K_ci_J_ci[192] = 0.0;
//   params.n_K_ci_J_ci[193] = 0.0;
//   params.n_K_ci_J_ci[194] = 0.0;
//   params.n_K_ci_J_ci[195] = 0.0;
//   params.n_K_ci_J_ci[196] = 0.0;
//   params.n_K_ci_J_ci[197] = 0.0;
//   params.n_K_ci_J_ci[198] = 0.0;
//   params.n_K_ci_J_ci[199] = 0.0;
//   params.n_K_ci_J_ci[200] = 0.0;
//   params.n_K_ci_J_ci[201] = 0.0;
//   params.n_K_ci_J_ci[202] = 0.0;
//   params.n_K_ci_J_ci[203] = 0.0;
//   params.n_K_ci_J_ci[204] = 0.0;
//   params.n_K_ci_J_ci[205] = 0.0;
//   params.n_K_ci_J_ci[206] = 0.0;
//   params.n_K_ci_J_ci[207] = 0.0;
//   params.n_K_ci_J_ci[208] = 0.0;
//   params.n_K_ci_J_ci[209] = 0.0;
//   params.n_K_ci_J_ci[210] = 0.0;
//   params.n_K_ci_J_ci[211] = 0.0;
//   params.n_K_ci_J_ci[212] = 0.0;
//   params.n_K_ci_J_ci[213] = 0.0;
//   params.n_K_ci_J_ci[214] = 0.0;
//   params.n_K_ci_J_ci[215] = 0.0;
//   params.n_K_ci_J_ci[216] = 0.0;
//   params.n_K_ci_J_ci[217] = 0.0;
//   params.n_K_ci_J_ci[218] = 0.0;
//   params.n_K_ci_J_ci[219] = 0.0;
//   params.n_K_ci_J_ci[220] = 0.0;
//   params.n_K_ci_J_ci[221] = 0.0;
//   params.n_K_ci_J_ci[222] = 0.0;
//   params.n_K_ci_J_ci[223] = 0.0;
//   params.n_K_ci_J_ci[224] = 0.0;
//   params.n_K_ci_J_ci[225] = 0.0;
//   params.n_K_ci_J_ci[226] = 0.0;
//   params.n_K_ci_J_ci[227] = 0.0;
//   params.n_K_ci_J_ci[228] = 0.0;
//   params.n_K_ci_J_ci[229] = 0.0;
//   params.n_K_ci_J_ci[230] = 0.0;
//   params.n_K_ci_J_ci[231] = 0.0;
//   params.n_K_ci_J_ci[232] = 0.0;
//   params.n_K_ci_J_ci[233] = 0.0;
//   params.n_K_ci_J_ci[234] = 0.0;
//   params.n_K_ci_J_ci[235] = 0.0;
//   params.n_K_ci_J_ci[236] = 0.0;
//   params.n_K_ci_J_ci[237] = 0.0;
//   params.n_K_ci_J_ci[238] = 0.0;
//   params.n_K_ci_J_ci[239] = 0.0;
//   params.n_K_ci_J_ci[240] = 0.0;
//   params.n_K_ci_J_ci[241] = 0.0;
//   params.n_K_ci_J_ci[242] = 0.0;
//   params.n_K_ci_J_ci[243] = 0.0;
//   params.n_K_ci_J_ci[244] = 0.0;
//   params.n_K_ci_J_ci[245] = 0.0;
//   params.n_K_ci_J_ci[246] = 0.0;
//   params.n_K_ci_J_ci[247] = 0.0;
//   params.n_K_ci_J_ci[248] = 0.0;
//   params.n_K_ci_J_ci[249] = 0.0;
//   params.n_K_ci_J_ci[250] = 0.0;
//   params.n_K_ci_J_ci[251] = 0.0;
//   params.n_K_ci_J_ci[252] = 0.0;
//   params.n_K_ci_J_ci[253] = 0.0;
//   params.n_K_ci_J_ci[254] = 0.0;
//   params.n_K_ci_J_ci[255] = 0.0;
//   params.n_K_ci_J_ci[256] = 0.0;
//   params.n_K_ci_J_ci[257] = 0.0;
//   params.n_K_ci_J_ci[258] = 0.0;
//   params.n_K_ci_J_ci[259] = 0.0;
//   params.n_K_ci_J_ci[260] = 0.0;
//   params.n_K_ci_J_ci[261] = 0.0;
//   params.n_K_ci_J_ci[262] = 0.0;
//   params.n_K_ci_J_ci[263] = 0.0;
//   params.n_K_ci_J_ci[264] = 0.0;
//   params.n_K_ci_J_ci[265] = 0.0;
//   params.n_K_ci_J_ci[266] = 0.0;
//   params.n_K_ci_J_ci[267] = 0.0;
//   params.n_K_ci_J_ci[268] = 0.0;
//   params.n_K_ci_J_ci[269] = 0.0;
//   params.n_K_ci_J_ci[270] = 0.0;
//   params.n_K_ci_J_ci[271] = 0.0;
//   params.n_K_ci_J_ci[272] = 0.0;
//   params.n_K_ci_J_ci[273] = 0.0;
//   params.n_K_ci_J_ci[274] = 0.0;
//   params.n_K_ci_J_ci[275] = 0.0;
//   params.n_K_ci_J_ci[276] = 0.0;
//   params.n_K_ci_J_ci[277] = 0.0;
//   params.n_K_ci_J_ci[278] = 0.0;
//   params.n_K_ci_J_ci[279] = 0.0;
//   params.n_K_ci_J_ci[280] = 0.0;
//   params.n_K_ci_J_ci[281] = 0.0;
//   params.n_K_ci_J_ci[282] = 0.0;
//   params.n_K_ci_J_ci[283] = 0.0;
//   params.n_K_ci_J_ci[284] = 0.0;
//   params.n_K_ci_J_ci[285] = 0.0;
//   params.n_K_ci_J_ci[286] = 0.0;
//   params.n_K_ci_J_ci[287] = 0.0;
//   params.n_K_ci_J_ci[288] = 0.0;
//   params.n_K_ci_J_ci[289] = 0.0;
//   params.n_K_ci_J_ci[290] = 0.0;
//   params.n_K_ci_J_ci[291] = 0.0;
//   params.n_K_ci_J_ci[292] = 0.0;
//   params.n_K_ci_J_ci[293] = 0.0;
//   params.n_K_ci_J_ci[294] = 0.0;
//   params.n_K_ci_J_ci[295] = 0.0;
//   params.n_K_ci_J_ci[296] = 0.0;
//   params.n_K_ci_J_ci[297] = 0.0;
//   params.n_K_ci_J_ci[298] = 0.0;
//   params.n_K_ci_J_ci[299] = 0.0;
//   params.n_K_ci_J_ci[300] = 0.0;
//   params.n_K_ci_J_ci[301] = 0.0;
//   params.n_K_ci_J_ci[302] = 0.0;
//   params.n_K_ci_J_ci[303] = 0.0;
//   params.n_K_ci_J_ci[304] = 0.0;
//   params.n_K_ci_J_ci[305] = 0.0;
//   params.n_K_ci_J_ci[306] = 0.0;
//   params.n_K_ci_J_ci[307] = 0.0;
//   params.n_K_ci_J_ci[308] = 0.0;
//   params.n_K_ci_J_ci[309] = 0.0;
//   params.n_K_ci_J_ci[310] = 0.0;
//   params.n_K_ci_J_ci[311] = 0.0;
//   params.n_K_ci_J_ci[312] = 0.0;
//   params.n_K_ci_J_ci[313] = 0.0;
//   params.n_K_ci_J_ci[314] = 0.0;
//   params.n_K_ci_J_ci[315] = 0.0;
//   params.n_K_ci_J_ci[316] = 0.0;
//   params.n_K_ci_J_ci[317] = 0.0;
//   params.n_K_ci_J_ci[318] = 0.0;
//   params.n_K_ci_J_ci[319] = 0.0;
//   params.n_K_ci_J_ci[320] = 0.0;
//   params.n_K_ci_J_ci[321] = 0.0;
//   params.n_K_ci_J_ci[322] = 0.0;
//   params.n_K_ci_J_ci[323] = 0.0;
//   params.n_K_ci_J_ci[324] = 0.0;
//   params.n_K_ci_J_ci[325] = 0.0;
//   params.n_K_ci_J_ci[326] = 0.0;
//   params.n_K_ci_J_ci[327] = 0.0;
//   params.n_K_ci_J_ci[328] = 0.0;
//   params.n_K_ci_J_ci[329] = 0.0;
//   params.n_K_ci_J_ci[330] = 0.0;
//   params.n_K_ci_J_ci[331] = 0.0;
//   params.n_K_ci_J_ci[332] = 0.0;
//   params.n_K_ci_J_ci[333] = 0.0;
//   params.n_K_ci_J_ci[334] = 0.0;
//   params.n_K_ci_J_ci[335] = 0.0;
//   params.n_K_ci_J_ci[336] = 0.0;
//   params.n_K_ci_J_ci[337] = 0.0;
//   params.n_K_ci_J_ci[338] = 0.0;
//   params.n_K_ci_J_ci[339] = 0.0;
//   params.n_K_ci_J_ci[340] = 0.0;
//   params.n_K_ci_J_ci[341] = 0.0;
//   params.n_K_ci_J_ci[342] = 0.0;
//   params.n_K_ci_J_ci[343] = 0.0;
//   params.n_K_ci_J_ci[344] = 0.0;
//   params.n_K_ci_J_ci[345] = 0.0;
//   params.n_K_ci_J_ci[346] = 0.0;
//   params.n_K_ci_J_ci[347] = 0.0;
//   params.n_K_ci_J_ci[348] = 0.0;
//   params.n_K_ci_J_ci[349] = 0.0;
//   params.n_K_ci_J_ci[350] = 0.0;
//   params.n_K_ci_J_ci[351] = 0.0;
//   params.n_K_ci_J_ci[352] = 0.0;
//   params.n_K_ci_J_ci[353] = 0.0;
//   params.n_K_ci_J_ci[354] = 0.0;
//   params.n_K_ci_J_ci[355] = 0.0;
//   params.n_K_ci_J_ci[356] = 0.0;
//   params.n_K_ci_J_ci[357] = 0.0;
//   params.n_K_ci_J_ci[358] = 0.0;
//   params.n_K_ci_J_ci[359] = 0.0;
//   params.n_K_ci_J_ci[360] = 0.0;
//   params.n_K_ci_J_ci[361] = 0.0;
//   params.n_K_ci_J_ci[362] = 0.0;
//   params.n_K_ci_J_ci[363] = 0.0;
//   params.n_K_ci_J_ci[364] = 0.0;
//   params.n_K_ci_J_ci[365] = 0.0;
//   params.n_K_ci_J_ci[366] = 0.0;
//   params.n_K_ci_J_ci[367] = 0.0;
//   params.n_K_ci_J_ci[368] = 0.0;
//   params.n_K_ci_J_ci[369] = 0.0;
//   params.n_K_ci_J_ci[370] = 0.0;
//   params.n_K_ci_J_ci[371] = 0.0;
//   params.n_K_ci_J_ci[372] = 0.0;
//   params.n_K_ci_J_ci[373] = 0.0;
//   params.n_K_ci_J_ci[374] = 0.0;
//   params.n_K_ci_J_ci[375] = 0.0;
//   params.n_K_ci_J_ci[376] = 0.0;
//   params.n_K_ci_J_ci[377] = 0.0;
//   params.n_K_ci_J_ci[378] = 0.0;
//   params.n_K_ci_J_ci[379] = 0.0;
//   params.n_K_ci_J_ci[380] = 0.0;
//   params.n_K_ci_J_ci[381] = 0.0;
//   params.n_K_ci_J_ci[382] = 0.0;
//   params.n_K_ci_J_ci[383] = 0.0;
//   params.n_K_ci_J_ci[384] = 0.0;
//   params.n_K_ci_J_ci[385] = 0.0;
//   params.n_K_ci_J_ci[386] = 0.0;
//   params.n_K_ci_J_ci[387] = 0.0;
//   params.n_K_ci_J_ci[388] = 0.0;
//   params.n_K_ci_J_ci[389] = 0.0;
//   params.n_K_ci_J_ci[390] = 0.0;
//   params.n_K_ci_J_ci[391] = 0.0;
//   params.n_K_ci_J_ci[392] = 0.0;
//   params.n_K_ci_J_ci[393] = 0.0;
//   params.n_K_ci_J_ci[394] = 0.0;
//   params.n_K_ci_J_ci[395] = 0.0;
//   params.n_K_ci_J_ci[396] = 0.0;
//   params.n_K_ci_J_ci[397] = 0.0;
//   params.n_K_ci_J_ci[398] = 0.0;
//   params.n_K_ci_J_ci[399] = 0.0;
//   params.n_K_ci_J_ci[400] = 0.0;
//   params.n_K_ci_J_ci[401] = 0.0;
//   params.n_K_ci_J_ci[402] = 0.0;
//   params.n_K_ci_J_ci[403] = 0.0;
//   params.n_K_ci_J_ci[404] = 0.0;
//   params.n_K_ci_J_ci[405] = 0.0;
//   params.n_K_ci_J_ci[406] = 0.0;
//   params.n_K_ci_J_ci[407] = 0.0;
//   params.n_K_ci_J_ci[408] = 0.0;
//   params.n_K_ci_J_ci[409] = 0.0;
//   params.n_K_ci_J_ci[410] = 0.0;
//   params.n_K_ci_J_ci[411] = 0.0;
//   params.n_K_ci_J_ci[412] = 0.0;
//   params.n_K_ci_J_ci[413] = 0.0;
//   params.n_K_ci_J_ci[414] = 0.0;
//   params.n_K_ci_J_ci[415] = 0.0;
//   params.n_K_ci_J_ci[416] = 0.0;
//   params.n_K_ci_J_ci[417] = 0.0;
//   params.n_K_ci_J_ci[418] = 0.0;
//   params.n_K_ci_J_ci[419] = 0.0;
//   params.n_K_ci_J_ci[420] = 0.0;
//   params.n_K_ci_J_ci[421] = 0.0;
//   params.n_K_ci_J_ci[422] = 0.0;
//   params.n_K_ci_J_ci[423] = 0.0;
//   params.n_K_ci_J_ci[424] = 0.0;
//   params.n_K_ci_J_ci[425] = 0.0;
//   params.n_K_ci_J_ci[426] = 0.0;
//   params.n_K_ci_J_ci[427] = 0.0;
//   params.n_K_ci_J_ci[428] = 0.0;
//   params.n_K_ci_J_ci[429] = 0.0;
//   params.n_K_ci_J_ci[430] = 0.0;
//   params.n_K_ci_J_ci[431] = 0.0;
//   params.n_K_ci_J_ci[432] = 0.0;
//   params.n_K_ci_J_ci[433] = 0.0;
//   params.n_K_ci_J_ci[434] = 0.0;
//   params.n_K_ci_J_ci[435] = 0.0;
//   params.n_K_ci_J_ci[436] = 0.0;
//   params.n_K_ci_J_ci[437] = 0.0;
//   params.n_K_ci_J_ci[438] = 0.0;
//   params.n_K_ci_J_ci[439] = 0.0;
//   params.n_K_ci_J_ci[440] = 0.0;
//   params.n_K_ci_J_ci[441] = 0.0;
//   params.n_K_ci_J_ci[442] = 0.0;
//   params.n_K_ci_J_ci[443] = 0.0;
//   params.n_K_ci_J_ci[444] = 0.0;
//   params.n_K_ci_J_ci[445] = 0.0;
//   params.n_K_ci_J_ci[446] = 0.0;
//   params.n_K_ci_J_ci[447] = 0.0;
//   params.n_K_ci_J_ci[448] = 0.0;
//   params.n_K_ci_J_ci[449] = 0.0;
//   params.f_max[0] = 5.0;
//   params.f_max[1] = 5.0;
//   params.f_max[2] = 5.0;
//   params.f_max[3] = 5.0;
//   params.f_max[4] = 5.0;
//   params.f_max[5] = 5.0;
//   params.f_max[6] = 5.0;
//   params.f_max[7] = 5.0 ;
//   params.f_max[8] = 5.0 ;
//   params.f_max[9] = 5.0 ;
//   params.f_max[10] = 5.0 ;
//   params.f_max[11] = 5.0 ;
//   params.f_max[12] = 5.0 ;
//   params.f_max[13] = 5.0 ;
//   params.f_max[14] = 5.0 ;
//   params.f_max[15] = 5.0 ;
//   params.f_max[16] = 5.0 ;
//   params.f_max[17] = 5.0 ;
//   params.f_max[18] = 5.0 ;
//   params.f_max[19] = 5.0 ;
//   params.f_max[20] = 0.0;
//   params.f_max[21] = 0.0;
//   params.f_max[22] = 0.0;
//   params.f_max[23] = 0.0;
//   params.f_max[24] = 0.0;
//   params.f_max[25] = 0.0;
//   params.f_max[26] = 0.0;
//   params.f_max[27] = 0.0;
//   params.f_max[28] = 0.0;
//   params.f_max[29] = 0.0;
//   params.f_max[30] = 0.0;
//   params.f_max[31] = 0.0;
//   params.f_max[32] = 0.0;
//   params.f_max[33] = 0.0;
//   params.f_max[34] = 0.0;
//   params.f_max[35] = 0.0;
//   params.f_max[36] = 0.0;
//   params.f_max[37] = 0.0;
//   params.f_max[38] = 0.0;
//   params.f_max[39] = 0.0;
//   params.f_max[40] = 0.0;
//   params.f_max[41] = 0.0;
//   params.f_max[42] = 0.0;
//   params.f_max[43] = 0.0;
//   params.f_max[44] = 0.0;
//   params.f_max[45] = 0.0;
//   params.f_max[46] = 0.0;
//   params.f_max[47] = 0.0;
//   params.f_max[48] = 0.0;
//   params.f_max[49] = 0.0;
//   params.f_max[50] = 0.0;
//   params.f_max[51] = 0.0;
//   params.f_max[52] = 0.0;
//   params.f_max[53] = 0.0;
//   params.f_max[54] = 0.0;
//   params.f_max[55] = 0.0;
//   params.f_max[56] = 0.0;
//   params.f_max[57] = 0.0;
//   params.f_max[58] = 0.0;
//   params.f_max[59] = 0.0;
//   params.f_max[60] = 0.0;
//   params.f_max[61] = 0.0;
//   params.f_max[62] = 0.0;
//   params.f_max[63] = 0.0;
//   params.f_max[64] = 0.0;
//   params.f_max[65] = 0.0;
//   params.f_max[66] = 0.0;
//   params.f_max[67] = 0.0;
//   params.f_max[68] = 0.0;
//   params.f_max[69] = 0.0;
//   params.f_max[70] = 0.0;
//   params.f_max[71] = 0.0;
//   params.f_max[72] = 0.0;
//   params.f_max[73] = 0.0;
//   params.f_max[74] = 0.0;
//   params.f_max[75] = 0.0;
//   params.f_max[76] = 0.0;
//   params.f_max[77] = 0.0;
//   params.f_max[78] = 0.0;
//   params.f_max[79] = 0.0;
//   params.f_max[80] = 0.0;
//   params.f_max[81] = 0.0;
//   params.f_max[82] = 0.0;
//   params.f_max[83] = 0.0;
//   params.f_max[84] = 0.0;
//   params.f_max[85] = 0.0;
//   params.f_max[86] = 0.0;
//   params.f_max[87] = 0.0;
//   params.f_max[88] = 0.0;
//   params.f_max[89] = 0.0;
//   params.f_max[90] = 0.0;
//   params.f_max[91] = 0.0;
//   params.f_max[92] = 0.0;
//   params.f_max[93] = 0.0;
//   params.f_max[94] = 0.0;
//   params.f_max[95] = 0.0;
//   params.f_max[96] = 0.0;
//   params.f_max[97] = 0.0;
//   params.f_max[98] = 0.0;
//   params.f_max[99] = 0.0;
//   params.f_max[100] = 0.0;
//   params.f_max[101] = 0.0;
//   params.f_max[102] = 0.0;
//   params.f_max[103] = 0.0;
//   params.f_max[104] = 0.0;
//   params.f_max[105] = 0.0;
//   params.f_max[106] = 0.0;
//   params.f_max[107] = 0.0;
//   params.f_max[108] = 0.0;
//   params.f_max[109] = 0.0;
//   params.f_max[110] = 0.0;
//   params.f_max[111] = 0.0;
//   params.f_max[112] = 0.0;
//   params.f_max[113] = 0.0;
//   params.f_max[114] = 0.0;
//   params.f_max[115] = 0.0;
//   params.f_max[116] = 0.0;
//   params.f_max[117] = 0.0;
//   params.f_max[118] = 0.0;
//   params.f_max[119] = 0.0;
//   params.f_max[120] = 0.0;
//   params.f_max[121] = 0.0;
//   params.f_max[122] = 0.0;
//   params.f_max[123] = 0.0;
//   params.f_max[124] = 0.0;
//   params.f_max[125] = 0.0;
//   params.f_max[126] = 0.0;
//   params.f_max[127] = 0.0;
//   params.f_max[128] = 0.0;
//   params.f_max[129] = 0.0;
//   params.f_max[130] = 0.0;
//   params.f_max[131] = 0.0;
//   params.f_max[132] = 0.0;
//   params.f_max[133] = 0.0;
//   params.f_max[134] = 0.0;
//   params.f_max[135] = 0.0;
//   params.f_max[136] = 0.0;
//   params.f_max[137] = 0.0;
//   params.f_max[138] = 0.0;
//   params.f_max[139] = 0.0;
//   params.f_max[140] = 0.0;
//   params.f_max[141] = 0.0;
//   params.f_max[142] = 0.0;
//   params.f_max[143] = 0.0;
//   params.f_max[144] = 0.0;
//   params.f_max[145] = 0.0;
//   params.f_max[146] = 0.0;
//   params.f_max[147] = 0.0;
//   params.f_max[148] = 0.0;
//   params.f_max[149] = 0.0;
//   params.u_min[0] = -1.54;
//   params.u_min[1] = -1.54;
//   params.u_min[2] = -1.54;
//   params.u_max[0] = 1.54;
//   params.u_max[1] = 1.54;
//   params.u_max[2] = 1.54;
}
