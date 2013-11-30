/* Produced by CVXGEN, 2012-03-10 20:34:58 -0800.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */

#define FORCE_ONE true
#define SIZEOF_ARRAY( a ) (sizeof( a ) / sizeof( a[ 0 ] ))

#include <sys/time.h>
#include "one_link_mpc_lin_dyn.h"
#include "mpc_dyn_controller.h"

// Vars vars;
// Params params;
// Workspace work;
// Settings settings;
// double storage[3];

// void print_all()
// {
//   std::cerr << "params.alpha = "<< params.alpha[0] << std::endl;
//   //std::cerr << "params.constr_weight = "<< params.constr_weight[0] << std::endl;
//   //std::cerr << "params.force_weight = "<< params.force_weight[0] << std::endl;
//   //std::cerr << "params.delta_t = "<< params.delta_t[0] << std::endl;

//   std::cerr <<"params.delta_x_d = [";
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.delta_x_d); ii++)
//     {
//       std::cerr << params.delta_x_d[ii]<<",\n";
//     }
//   std::cerr << "];\n\n";

//   std::cerr <<"params.J = [";
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.J); ii++)
//     {
//       std::cerr << params.J[ii] <<",\n";
//     }
//   std::cerr << "];\n\n";

//   std::cerr <<"params.q_0 = [";	      
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.q_0); ii++)
//     {
//       std::cerr << params.q_0[ii] <<",\n";
//     }
//   std::cerr << "];\n\n";

//   std::cerr <<"params.qd_0 = [";	      
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.qd_0); ii++)
//     {
//       std::cerr << params.qd_0[ii] <<",\n";
//     }
//   std::cerr << "];\n\n";

//   std::cerr <<"params.q_des_cur_0 = [";	      
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.q_des_cur_0); ii++)
//     {
//       std::cerr << params.q_des_cur_0[ii] <<",\n";
//     }
//   std::cerr << "];\n\n";

//   std::cerr <<"params.q_min = [";	      
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.q_min); ii++)
//     {
//       std::cerr << params.q_min[ii] <<",\n";
//     }
//   std::cerr << "];\n\n";


//   std::cerr <<"params.q_max = [";	      
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.q_max); ii++)
//     {
//       std::cerr << params.q_max[ii] <<",\n";
//     }
//   std::cerr << "];\n\n";


//   // std::cerr <<"params.qd_min = [";	      
//   // for (uint ii=0; ii < SIZEOF_ARRAY(params.qd_min); ii++)
//   //   {
//   //     std::cerr << params.qd_min[ii] <<",\n";
//   //   }
//   // std::cerr << "];\n\n";


//   std::cerr <<"params.qd_max = [";	      
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.qd_max); ii++)
//     {
//       std::cerr << params.qd_max[ii] <<",\n";
//     }
//   std::cerr << "];\n\n";


//   std::cerr <<"params.u_max = [";	      
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.u_max); ii++)
//     {
//       std::cerr << params.u_max[ii] <<",\n";
//     }
//   std::cerr << "];\n\n";

//   // std::cerr <<"params.u_min = [";	      
//   // for (uint ii=0; ii < SIZEOF_ARRAY(params.u_min); ii++)
//   //   {
//   //     std::cerr << params.u_min[ii] <<",\n";
//   //   }
//   // std::cerr << "];\n\n";


//   std::cerr <<"params.tau_cont_sum_0 = [";	      
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.tau_cont_sum_0); ii++)
//     {
//       std::cerr << params.tau_cont_sum_0[ii] <<",\n";
//     }
//   std::cerr << "];\n\n";



//   // std::cerr <<"params.Kp = [";	      
//   // for (uint ii=0; ii < SIZEOF_ARRAY(params.Kp); ii++)
//   //   {
//   //     std::cerr << params.Kp[ii] <<",\n";
//   //   }
//   // std::cerr << "];\n\n";


//   // std::cerr <<"params.Kd = [";	      
//   // for (uint ii=0; ii < SIZEOF_ARRAY(params.Kd); ii++)
//   //   {
//   //     std::cerr << params.Kd[ii] <<",\n";
//   //   }
//   // std::cerr << "];\n\n";


//   // std::cerr <<"params.M = [";	      
//   // for (uint ii=0; ii < SIZEOF_ARRAY(params.M); ii++)
//   //   {
//   //     std::cerr << params.M[ii] <<",\n";
//   //   }
//   // std::cerr << "];\n\n";

//   // std::cerr <<"params.C = [";	      
//   // for (uint ii=0; ii < SIZEOF_ARRAY(params.C); ii++)
//   //   {
//   //     std::cerr << params.C[ii] <<",\n";
//   //   }
//   // std::cerr << "];\n\n";

//   std::cerr <<"params.all_J_T_K_J = [";	      
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.all_J_T_K_J); ii++)
//     {
//       std::cerr << params.all_J_T_K_J[ii] <<",\n";
//     }
//   std::cerr << "];\n\n";

//   std::cerr <<"params.delta_f_max = [";	      
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.delta_f_max); ii++)
//     {
//       std::cerr << params.delta_f_max[ii] <<",\n";
//     }
//   std::cerr << "];\n\n";

//   std::cerr <<"params.n_K_ci_J_ci = [";	      
//   for (uint ii=0; ii < SIZEOF_ARRAY(params.n_K_J_all); ii++)
//     {
//       std::cerr << params.n_K_J_all[ii] <<",\n";
//     }
//   std::cerr << "];\n\n";
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_opt_mpc");
  ros::NodeHandle n;
  
  MpcDynController controller(n, params);

  //////code should look something like this ....

  //init an object of mpc controller type that takes topic name as input from param server
  //inside the object is a subscriber to robot state message
  
  //subscribe here to go, start and pause messages for controller (was epc_stop, stuff)

  //then run in rated controlled loop

  //service = n.advertiseService("get_mpc_fast", get_delta_phi);
  // add a subscriber that gets robot state (using "set_data" of sorts)

  //set_data(req);

  // storage[0] = 0;
  // storage[1] = 0;
  // storage[2] = 0;

  // timespec start;
  // timespec now;
  // double diff;
  
  // clock_gettime(CLOCK_REALTIME, &start);
  // clock_gettime(CLOCK_REALTIME, &now);

  std::cerr<< "It's Spinning ..." << std::endl;
  ros::spin();

  // ros::Rate r(100);
  // while (ros::ok())
  //   {
  //     clock_gettime(CLOCK_REALTIME, &now);
  //     diff = ((double)now.tv_nsec - (double)start.tv_nsec)/1000000000.;
  //     start = now;
  //     std::cout << "RATE IS :" << diff << '\n';
  //     if (true)  // this should be the check for if the controller should stop or run, could be in this file, I would prefer it be in the controller object though.
  // 	{
  // 	  //calls method in controller object that calculates and sends new jeps.
	  
  // 	  solve();

  // 	  if (work.converged == 1)
  // 	    {
  // 	      for (int ii = 0; ii < 3; ii++)
  // 		{
  // 		  controller.delta_jep.data[ii] = vars.u[0][ii];
  // 		  controller.predicted_joint_angles.data[ii] = params.q[0][ii] - storage[ii]; //vars.q[1][ii];
  // 		}
  // 	      std::cerr << "actual: \t" << controller.predicted_joint_angles.data[0] << "\t" << controller.predicted_joint_angles.data[1] << "\t" << controller.predicted_joint_angles.data[2] << "\n";
  // 	      std::cerr << "#######################################################################\n";
  // 	      std::cerr << "predicted: \t" << vars.q[1][0]- params.q[0][0] << "\t" << vars.q[1][1]- params.q[0][1] << "\t" << vars.q[1][2]- params.q[0][2] << "\n";
  // 	      //print_all();

  // 	      controller.jep_pub.publish(controller.delta_jep);
  // 	      controller.predicted_joint_angles_pub.publish(controller.predicted_joint_angles);
  // 	      storage[0] = params.q[0][0];
  // 	      storage[1] = params.q[0][1];
  // 	      storage[2] = params.q[0][2];
  // 	    }
  // 	  else
  // 	    {
  // 	      std::cerr << "it didn't converge!!"<< std::endl;
  // 	      //print_all();

  // 	      for (int ii = 0; ii < 3; ii++)
  // 		{
  // 		  controller.delta_jep.data[ii] = 0.0;
  // 		}
  // 	      controller.jep_pub.publish(controller.delta_jep);
  // 	      controller.predicted_joint_angles_pub.publish(controller.predicted_joint_angles);
  // 	    }
  // 	}

  //     r.sleep();
  //     ros::spinOnce();
  //   }

  return 1;
}

