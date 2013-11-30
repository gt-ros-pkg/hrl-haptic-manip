#ifdef FORCE_SEVEN
#include "seven_link_mpc_lin_dyn.h"
#define NUM_JOINTS 7
#endif

#ifdef FORCE_THREE
#define NUM_JOINTS 3
#include "three_link_mpc_lin_dyn.h"
#endif

#ifdef FORCE_ONE
#define NUM_JOINTS 1
#include "one_link_mpc_lin_dyn.h"
#endif


/* /\* ROS HEADERS TO INCLUDE *\/ */
#include "ros/ros.h"

//ROS message types 
#include "hrl_haptic_manipulation_in_clutter_msgs/MpcDynFormattedData.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/RobotHapticState.h"
#include "hrl_haptic_manipulation_in_clutter_srvs/MpcFormatted.h"
#include "hrl_msgs/FloatArrayBare.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

//Using boost to lock some threads to avoid race conditions.
#include <boost/thread/mutex.hpp>
#include <sys/time.h>


Vars vars;
Params params;
Workspace work;
Settings settings;


using namespace std;

class MpcDynController{
 public:
    MpcDynController(ros::NodeHandle &nh, Params &params);
    ~MpcDynController();
    //void formatted_dataCallback(const hrl_haptic_manipulation_in_clutter_msgs::MpcDynFormattedData &msg);
    bool formatted_dataCallback(hrl_haptic_manipulation_in_clutter_srvs::MpcFormatted::Request &req, 
				hrl_haptic_manipulation_in_clutter_srvs::MpcFormatted::Response &res);
    void dataCallback(const hrl_haptic_manipulation_in_clutter_msgs::RobotHapticState &msg);
    void goalCallback(const geometry_msgs::PoseStamped &msg);
    void load_default_data(); 
    //public functions go here
    ros::Publisher jep_pub;
    ros::Publisher predicted_joint_angles_pub;

    //ros::Subscriber sub_robot_state;
    ros::Subscriber sub_cur_goal;
    //ros::Subscriber sub_formatted_data;

    float input_cost;
    hrl_msgs::FloatArrayBare delta_jep;
    hrl_msgs::FloatArrayBare total_input_change;
    hrl_msgs::FloatArrayBare final_q;
    //hrl_msgs::FloatArrayBare qd_eps;
    hrl_msgs::FloatArrayBare f_eps;
    hrl_msgs::FloatArrayBare predicted_joint_angles;
    hrl_msgs::FloatArrayBare q_storage;
    Params *params_local;

 protected:
    //functions here
	
    ros::Time start_cb; 
    ros::Time now_cb;
    ros::Time start_solver;
    ros::Time now_solver;
 
    //variables here
    /* timespec start; */
    /* timespec solve_start; */
    /* timespec now; */
    /* timespec now_solve; */

    ros::ServiceServer service;
    ros::NodeHandle nh_;
    geometry_msgs::Pose cur_goal;
    boost::mutex m;
    hrl_haptic_manipulation_in_clutter_msgs::RobotHapticState local_robot_state;
	
};

MpcDynController::MpcDynController(ros::NodeHandle &nh, Params &params) :
    nh_(nh)
{
  //instantiate all public and private variables here

    /* angles_pub = nh_.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/joint_angles", 100); */
    /* angle_rates_pub = nh_.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/joint_angle_rates", 100);   */


  //add code to get alpha, beta, etc. from params server.
  /* while (nh_.getParam("/m3/software_testbed/linkage/num_links", num_links) == false) */
  /*   sleep(0.1); */

  set_defaults();
  setup_indexing();
  settings.verbose = 0;
  settings.max_iters = 30;
  settings.eps = 1e-5;  
  settings.resid_tol = 1e-4; 
  //settings.kkt_reg = 1e-4;

  start_cb = ros::Time::now();
  now_cb = ros::Time::now();
  start_solver = ros::Time::now();
  now_solver = ros::Time::now();

  /* clock_gettime(CLOCK_REALTIME, &start); */
  /* clock_gettime(CLOCK_REALTIME, &solve_start); */
  /* clock_gettime(CLOCK_REALTIME, &now); */
  /* clock_gettime(CLOCK_REALTIME, &now_solve); */

  params_local = &params;
  jep_pub = nh_.advertise<hrl_msgs::FloatArrayBare>("/delta_jep_mpc_cvxgen", 100);
  predicted_joint_angles_pub = nh_.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/predicted_joint_angles", 100);
  service = nh_.advertiseService("solve_qp_problem", &MpcDynController::formatted_dataCallback, this);
  //service = nh_.advertiseService("solve_qp_problem", MpcDynController::formatted_dataCallback);

  input_cost = 0;
  m.lock();
  /* for (int ii = 0; ii < 10; ii++) */
  /*   { */
  /*     f_eps.data.push_back(0.0); */
  /*   } */


  for (int ii = 0; ii < NUM_JOINTS; ii++)
    {
      q_storage.data.push_back(0.0);
      predicted_joint_angles.data.push_back(0.0);
      delta_jep.data.push_back(0.0);
      total_input_change.data.push_back(0.0);
      final_q.data.push_back(0.0);
      //qd_eps.data.push_back(0.0);
    }
  m.unlock();

  // see callback definition for why this is currently not used.
  // sub_robot_state = nh_.subscribe("/robot_haptic_state", 100, &MpcDynController::dataCallback, this);

  //sub_formatted_data = nh_.subscribe("/formatted_mpc_data", 0, &MpcDynController::formatted_dataCallback, this);
  sub_cur_goal = nh_.subscribe("/cur_goal", 100, &MpcDynController::goalCallback, this);

  //ros::topic::waitForMessage<hrl_haptic_manipulation_in_clutter_msgs::MpcDynFormattedData>("/formatted_mpc_data");

  
}

MpcDynController::~MpcDynController()
{
}

void MpcDynController::goalCallback(const geometry_msgs::PoseStamped &msg)
{
  //std::cout << "got into subscriber callback\n";
  cur_goal.position = msg.pose.position;
  cur_goal.orientation = msg.pose.orientation;
}


// this will be used when I do all data formatting in c++, right now some is 
// still in python for ease of coding
void MpcDynController::dataCallback(const hrl_haptic_manipulation_in_clutter_msgs::RobotHapticState &msg)
{
  //std::cerr << "got into callback" << std::endl;
  local_robot_state = msg;
  /* local_robot_state.header = msg.header; */
  /* local_robot_state.joint_names = msg.joint_names; */
  /* local_robot_state.joint_angles = msg.joint_angles; */
  /* local_robot_state.joint_velocities = msg.joint_velocities; */
  /* local_robot_state.joint_stiffness = msg.joint_stiffness; */
  /* local_robot_state.joint_damping = msg.joint_damping; */
  /* local_robot_state.hand_pose = msg.hand_pose; */
  /* local_robot_state.contact_jacobians = msg.contact_jacobians; */
  /* local_robot_state.skins = msg.skins; */
  
  for (uint ii=0; ii < msg.skins.size(); ii++)
    {
      if (msg.skins[ii].sensor_type == "bosch_proximity")
	{
	  std::cerr << "nothing done yet, but need to format constraints for bosch sensor" << std::endl;
	  assert(false);
	}
      else if (msg.skins[ii].sensor_type == "fabric_sensor")
	{
	  std::cerr << "nothing done yet, but need to format constraints for fabric sensor" << std::endl;
	  assert(false);
	}
      else if (msg.skins[ii].sensor_type == "meka_sensor")
	{
	  std::cerr << "nothing done yet, but need to format constraints for meka sensor" << std::endl;
	  assert(false);
	}
    }

  std::cerr << "this callback is not yet implemented, aborting ... " << std::endl;
  assert(false);

}

//void MpcDynController::formatted_dataCallback(const hrl_haptic_manipulation_in_clutter_msgs::MpcDynFormattedData &msg)
bool MpcDynController::formatted_dataCallback(hrl_haptic_manipulation_in_clutter_srvs::MpcFormatted::Request &req, 
					      hrl_haptic_manipulation_in_clutter_srvs::MpcFormatted::Response &res)
{
  //std::cout << "GOT IN TO OPT\n";
  now_cb = ros::Time::now();
  double diff;
  diff = now_cb.toSec() - start_cb.toSec();
  //std::cout << "SUBSCRIBER RATE IS :" << diff << '\n';   
  start_cb = ros::Time::now();

  /* double diff; */
  /* double diff_solver; */
  
  /* clock_gettime(CLOCK_REALTIME, &now); */
  /* diff = ((double)now.tv_nsec - (double)start.tv_nsec)/1000000000.; */
  /* start = now; */

  //these should be from a param server as well as any other fixed numbers!!
  params_local->alpha[0] = req.opt_data.alpha;
  //params_local->state_scaling[0] = req.opt_data.state_scaling;
  params_local->beta[0] = req.opt_data.beta;
  //params_local->gamma[0] = req.opt_data.gamma;
  params_local->zeta[0] = req.opt_data.zeta;
  params_local->mu[0] = req.opt_data.mu;
  //params_local->beta[0] = req.opt_data.beta;
  //params_local->gamma[0] = req.opt_data.gamma;
  //params_local->constr_weight[0] = req.opt_data.constr_weight;
  //params_local->force_weight[0] = req.opt_data.force_weight;
  //params_local->delta_t[0] = req.opt_data.delta_t;

  //std::cerr << "size of delta_x_d :" << req.opt_data.delta_x_d.size() << std::endl;
  for (uint ii=0; ii < req.opt_data.delta_x_d.size(); ii++)
    {
      params_local->delta_x_d[ii] = req.opt_data.delta_x_d[ii];
      // std::cerr << "delta_x_d_" <<  ii << ":\t" << req.opt_data.delta_x_d[ii] << std::endl;
    }

  for (uint ii=0; ii < req.opt_data.J.size(); ii++)
    {
      params_local->J[ii] = req.opt_data.J[ii];
    }

  for (uint ii=0; ii < req.opt_data.A_tl.size(); ii++)
    {
      params_local->A_tl[ii] = req.opt_data.A_tl[ii];
      params_local->A_tr[ii] = req.opt_data.A_tr[ii];
      params_local->A_bl[ii] = req.opt_data.A_bl[ii];
      params_local->A_br[ii] = req.opt_data.A_br[ii];
    }

  for (uint ii=0; ii < req.opt_data.B_t1.size(); ii++)
    {
      params_local->B_t1[ii] = req.opt_data.B_t1[ii];
      params_local->B_t2[ii] = req.opt_data.B_t2[ii];
      params_local->B_t3[ii] = req.opt_data.B_t3[ii];
      params_local->B_b1[ii] = req.opt_data.B_b1[ii];
      params_local->B_b2[ii] = req.opt_data.B_b2[ii];
      params_local->B_b3[ii] = req.opt_data.B_b3[ii];
    }

  // std::cerr << "size of x_0 :" << req.opt_data.x_0.size() << std::endl;
  for (uint ii=0; ii < req.opt_data.q_0.size(); ii++)
    {
      params_local->q_0[ii] = req.opt_data.q_0[ii];
      params_local->qd_0[ii] = req.opt_data.qd_0[ii];
      /* if (params_local->qd_0[ii]>0.4) */
      /* 	{ */
      /* 	  std::cout << "going too fast at " << ros::Time::now().toSec() << "seconds\n"; */
      /* 	} */
      //params_local->vel_norm_J[ii] = req.opt_data.vel_norm_J[ii];
      params_local->q_des_cur_0[ii] = req.opt_data.q_des_cur_0[ii];
      params_local->q_min[ii] = req.opt_data.q_min[ii];
      params_local->q_max[ii] = req.opt_data.q_max[ii];
      //params_local->qd_max[ii] = req.opt_data.qd_max[ii];
      params_local->u_max[ii] = req.opt_data.u_max[ii];
      params_local->tau_max_delta_t[ii] = req.opt_data.tau_max_delta_t[ii];
      //params_local->f_max_delta_t[ii] = req.opt_data.f_max_delta_t[ii];
      params_local->tau_cont_sum_0[ii] = req.opt_data.tau_cont_sum_0[ii];
      // std::cerr << "x_0_" <<  ii << ":\t" << req.opt_data.x_0[ii] << std::endl;
    }

  // std::cerr << "size of KP_t_KP :" << req.opt_data.KP_t_KP.size() << std::endl;
  for (uint ii=0; ii < req.opt_data.mass.size(); ii++)
    {
      /* params_local->Kp[ii] = req.opt_data.Kp[ii]; */
      /* params_local->Kd[ii] = req.opt_data.Kd[ii]; */
      //params_local->all_J_T_K_J[ii] = req.opt_data.all_J_T_K_J[ii];
      //params_local->mass_n_J_com[ii] = req.opt_data.mass_n_J_com[ii];
      params_local->mass[ii] = req.opt_data.mass[ii];
      //params_local->vel_norm_J[ii] = req.opt_data.vel_norm_J[ii];
    }

  for (uint ii=0; ii < req.opt_data.delta_f_max.size(); ii++)
    {
      params_local->delta_f_max[ii] = req.opt_data.delta_f_max[ii];
      params_local->delta_rate_f_max[ii] = req.opt_data.delta_rate_f_max[ii];
      //params_local->zeros[ii] = 0.0;
      //params_local->delta_f_des[ii] = req.opt_data.delta_f_des[ii];
      //params_local->delta_cont_vel_max[ii] = req.opt_data.delta_cont_vel_max[ii];
    }

  for (uint ii=0; ii < req.opt_data.n_K_J_all.size(); ii++)
    {
      params_local->n_K_J_all[ii] = req.opt_data.n_K_J_all[ii];
      //params_local->n_J_all[ii] = req.opt_data.n_J_all[ii];
      //params_local->n_K_J_over[ii] = req.opt_data.n_K_J_over[ii];
    }

  m.lock();
  now_solver = ros::Time::now();
  /* clock_gettime(CLOCK_REALTIME, &now_solve); */

  solve();

  double diff_solver;
  diff_solver = now_solver.toSec() - start_solver.toSec();
  //std::cout << "SOLVER RATE IS :" << diff_solver << '\n';   
  start_solver = ros::Time::now();

  /* diff_solver = ((double)now_solve.tv_nsec - (double)solve_start.tv_nsec)/1000000000.; */
  /* solve_start.tv_nsec = now_solve.tv_nsec; */
  /* solve_start.tv_sec = now_solve.tv_sec; */
  /* std::cout << "SOLVER RATE IS :" << diff_solver << '\n'; */
  m.unlock();
  
  if (work.converged == 1)
    {
      for (int ii = 0; ii < NUM_JOINTS; ii++)
	{
	  delta_jep.data[ii] = vars.u[0][ii];
	  predicted_joint_angles.data[ii] = params_local->q[0][ii] - q_storage.data[ii]; //vars.q[1][ii];
	  q_storage.data[ii] = params_local->q[0][ii];
	  //qd_eps.data[ii] = vars.qd_eps[ii];
	  //final_q.data[ii] = vars.q[6][ii];
	}


      /* for (int ii = 0; ii < 10; ii++) */
      /* 	{ */
      /* 	  std::cerr << "f_eps[1]["<<ii<< "] :\t" << vars.f_eps[1][ii] << "\n"; */
      /* 	  std::cerr << "f_eps[2]["<<ii<< "] :\t" << vars.f_eps[2][ii] << "\n"; */
      /* 	  std::cerr << "f_eps[3]["<<ii<< "] :\t" <<  vars.f_eps[3][ii] << "\n"; */
      /* 	  std::cerr << "f_eps[4]["<<ii<< "] :\t" << vars.f_eps[4][ii] << "\n"; */
      /* 	  std::cerr << "f_eps[5]["<<ii<< "] :\t" << vars.f_eps[5][ii] << "\n"; */
      /* 	  std::cerr << "f_eps[6]["<<ii<< "] :\t" << vars.f_eps[6][ii] << "\n"; */
      /* 	} */

      /* total_input_change.data[0] = 0; */
      /* total_input_change.data[1] = 0; */
      /* total_input_change.data[2] = 0; */
      /* input_cost = 0; */
      
      /* for (int ii = 0; ii < 6; ii++) */
      /* 	{ */
      /* 	  total_input_change.data[0]  = total_input_change.data[0] + vars.u[ii][0]; */
      /* 	  total_input_change.data[1]  = total_input_change.data[1] + vars.u[ii][1]; */
      /* 	  total_input_change.data[2]  = total_input_change.data[2] + vars.u[ii][2]; */
      /* 	  input_cost = input_cost +  (vars.u[ii][0]* vars.u[ii][0] + vars.u[ii][1]* vars.u[ii][1]+ vars.u[ii][2]* vars.u[ii][2]); */
      /* 	} */

      /* for (int ii = 0; ii < 40; ii++) */
      /* 	{ */
      /* 	  f_eps.data[ii] = vars.f_eps[ii]; */
      /* 	} */


      /* std::cerr << "actual: \t" << predicted_joint_angles.data[0] << "\t" << predicted_joint_angles.data[1] << "\t" << predicted_joint_angles.data[2] << "\n"; */
      /* std::cerr << "#######################################################################\n"; */
      /* std::cerr << "predicted: \t" << vars.q[1][0]- params_local->q[0][0] << "\t" << vars.q[1][1]- params_local->q[0][1] << "\t" << vars.q[1][2]- params_local->q[0][2] << "\n"; */
      //print_all();

      //      jep_pub.publish(delta_jep);
      //predicted_joint_angles_pub.publish(predicted_joint_angles);

    }
  else
    {
      //std::cerr << "it didn't converge!!"<< std::endl;
      //print_all();

      for (int ii = 0; ii < NUM_JOINTS; ii++)
	{
	  delta_jep.data[ii] = 0.0;
	}
      //jep_pub.publish(delta_jep);
      //predicted_joint_angles_pub.publish(predicted_joint_angles);
    }
  
  /* std::cerr << "\n\n LOOK HERE FOR CURRENT SOLUTION \n\n"; */
  /* std::cerr << "delta_jep is :" << delta_jep.data[0] << "\t" << delta_jep.data[1] << "\t" << delta_jep.data[2] << "\n"; */


  res.delta_jep = delta_jep;
  res.final_q = final_q;
  res.total_input_change = total_input_change;
  res.input_cost = input_cost;
  //res.f_eps = f_eps;
  //res.qd_eps = qd_eps;

  return true;

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






}


void MpcDynController::load_default_data() 
{
  std::cerr<<"there is no default data here yet, aborting ... " << std::endl;
}
