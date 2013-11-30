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

  for (int ii = 0; ii < 7; ii++)
    {
      delta_jep.data.push_back(0.0);
      predicted_joint_angles.data.push_back(0.0);
    }

  jep_pub = n.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/delta_jep_mpc_fast", 100);
  predicted_joint_angles_pub = n.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/predicted_joint_angles", 100);
  service = n.advertiseService("get_mpc_fast", get_delta_phi);

  set_defaults();
  setup_indexing();

  settings.verbose = 0;

  ros::spin();

  return 0;
}

bool get_delta_phi(hrl_haptic_manipulation_in_clutter_srvs::ServiceBasedMPC::Request &req,
		   hrl_haptic_manipulation_in_clutter_srvs::ServiceBasedMPC::Response &res )
{
  set_data(req);
  solve();

  if (work.converged == 1)
    {
      for (int ii = 0; ii < 7; ii++)
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
      for (int ii = 0; ii < 7; ii++)
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

void set_data(hrl_haptic_manipulation_in_clutter_srvs::ServiceBasedMPC::Request &req)
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

  for (uint ii=0; ii < req.Q.size(); ii++)
    {
      params.Q[ii] = req.Q[ii];
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
  for (uint ii=0; ii < 60; ii++)
    {
      params.desired_force_decrease[ii] = req.desired_force_decrease[ii];
      params.f_min[ii] = req.f_min[ii];
      params.f_max[ii] = req.f_max[ii];
      // std::cerr << "f_min_" <<  ii << ":\t" << req.f_min[ii] << std::endl;
      // std::cerr << "f_max_" <<  ii << ":\t" << req.f_max[ii] << std::endl;
    }

  // std::cerr << "size of n_K_ci_J_ci :" << req.n_K_ci_J_ci.size() << std::endl;
  for (uint ii=0; ii < 420; ii++)
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
}
