/* Produced by CVXGEN, 2012-03-10 20:34:58 -0800.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */

#define PROX_THREE true

#include "proximity_3_link_qs_mpc.h"
#include "mpc_qs_controller.h"

Vars vars;
Params params;
Workspace work;
Settings settings;

int main(int argc, char **argv)
{
  std::cerr<< "It's Running ..." << std::endl;
  ros::init(argc, argv, "fast_opt_mpc");
  ros::NodeHandle n;
  
  MPC_QS_Controller controller(n, params);

  set_defaults();
  setup_indexing();

  settings.verbose = 0;

  //////code should look something like this ....

  //init an object of mpc controller type that takes topic name as input from param server
  //inside the object is a subscriber to robot state message
  
  //subscribe here to go, start and pause messages for controller (was epc_stop, stuff)

  //then run in rated controlled loop

  //service = n.advertiseService("get_mpc_fast", get_delta_phi);
  // add a subscriber that gets robot state (using "set_data" of sorts)

  //set_data(req);

  ros::Rate r(100);
  while (ros::ok())
    {
      if (true)  // this should be the check for if the controller should stop or run, could be in this file, I would prefer it be in the controller object though.
	{
	  //calls method in controller object that calculates and sends new jeps.
	  solve();

	  if (work.converged == 1)
	    {
	      for (int ii = 0; ii < 3; ii++)
		{
		  controller.delta_jep.data[ii] = vars.u[0][ii];
		  controller.predicted_joint_angles.data[ii] = vars.x[1][ii];
		}
	      controller.jep_pub.publish(controller.delta_jep);
	      controller.predicted_joint_angles_pub.publish(controller.predicted_joint_angles);
	    }
	  else
	    {
	      std::cerr << "it didn't converge!!"<< std::endl;
	      for (int ii = 0; ii < 3; ii++)
		{
		  controller.delta_jep.data[ii] = 0.0;
		}
	      controller.jep_pub.publish(controller.delta_jep);
	      controller.predicted_joint_angles_pub.publish(controller.predicted_joint_angles);
	    }
	}

      ros::spinOnce();
      r.sleep();
    }

  return 1;
}

