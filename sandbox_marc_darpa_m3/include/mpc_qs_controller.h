#ifdef PROX_SEVEN
#include "proximity_7_link_qs_mpc.h"
#define NUM_JOINTS 7
#endif

#ifdef PROX_THREE
#define NUM_JOINTS 3
#include "proximity_3_link_qs_mpc.h"
#endif

/* /\* ROS HEADERS TO INCLUDE *\/ */
#include "ros/ros.h"

//ROS message types 
#include "hrl_haptic_manipulation_in_clutter_msgs/MPC_FormattedData.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/RobotHapticState.h"
#include "hrl_msgs/FloatArrayBare.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
//#include "sandbox_marc_darpa_m3/ServiceBasedMPC.h"

//Using boost to lock some threads to avoid race conditions.
#include <boost/thread/mutex.hpp>

using namespace std;

class MPC_QS_Controller{
 public:
    MPC_QS_Controller(ros::NodeHandle &nh, Params &params);
    ~MPC_QS_Controller();
    void formatted_dataCallback(const hrl_haptic_manipulation_in_clutter_msgs::MPC_FormattedData &msg);
    void dataCallback(const hrl_haptic_manipulation_in_clutter_msgs::RobotHapticState &msg);
    void goalCallback(const geometry_msgs::PoseStamped &msg);
    void load_default_data(); 
    //public functions go here
    ros::Publisher jep_pub;
    ros::Publisher predicted_joint_angles_pub;

    //ros::Subscriber sub_robot_state;
    ros::Subscriber sub_cur_goal;
    ros::Subscriber sub_formatted_data;

    hrl_msgs::FloatArrayBare delta_jep;
    hrl_msgs::FloatArrayBare predicted_joint_angles;
    Params *params_local;


 protected:
    //functions here
	
    //variables here
    ros::NodeHandle nh_;
    geometry_msgs::Pose cur_goal;
    boost::mutex m;
    hrl_haptic_manipulation_in_clutter_msgs::RobotHapticState local_robot_state;
	
};

MPC_QS_Controller::MPC_QS_Controller(ros::NodeHandle &nh, Params &params) :
    nh_(nh)
{
  //instantiate all public and private variables here

    /* angles_pub = nh_.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/joint_angles", 100); */
    /* angle_rates_pub = nh_.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/joint_angle_rates", 100);   */


  //add code to get alpha, beta, etc. from params server.
  /* while (nh_.getParam("/m3/software_testbed/linkage/num_links", num_links) == false) */
  /*   sleep(0.1); */

  params_local = &params;
  jep_pub = nh_.advertise<hrl_msgs::FloatArrayBare>("/delta_jep_mpc_cvxgen", 100);
  predicted_joint_angles_pub = nh_.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/predicted_joint_angles", 100);

  m.lock();
  for (int ii = 0; ii < NUM_JOINTS; ii++)
    {
      delta_jep.data.push_back(0.0);
      predicted_joint_angles.data.push_back(0.0);
    }
  m.unlock();

  // see callback definition for why this is currently not used.
  // sub_robot_state = nh_.subscribe("/robot_haptic_state", 100, &MPC_QS_Controller::dataCallback, this);

  sub_formatted_data = nh_.subscribe("/formatted_mpc_data", 100, &MPC_QS_Controller::formatted_dataCallback, this);
  sub_cur_goal = nh_.subscribe("/cur_goal", 100, &MPC_QS_Controller::goalCallback, this);

  ros::topic::waitForMessage<hrl_haptic_manipulation_in_clutter_msgs::MPC_FormattedData>("/formatted_mpc_data");

  
}

MPC_QS_Controller::~MPC_QS_Controller()
{
}

void MPC_QS_Controller::goalCallback(const geometry_msgs::PoseStamped &msg)
{
  cur_goal.position = msg.pose.position;
  cur_goal.orientation = msg.pose.orientation;
}


// this will be used when I do all data formatting in c++, right now some is 
// still in python for ease of coding
void MPC_QS_Controller::dataCallback(const hrl_haptic_manipulation_in_clutter_msgs::RobotHapticState &msg)
{
  std::cerr << "got into callback" << std::endl;
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

void MPC_QS_Controller::formatted_dataCallback(const hrl_haptic_manipulation_in_clutter_msgs::MPC_FormattedData &msg)
{
  //these should be from a param server to a
  params_local->alpha[0] = msg.alpha;
  params_local->beta[0] = msg.beta;
  params_local->gamma[0] = msg.gamma;

  // std::cerr << "size of delta_x_d :" << msg.delta_x_d.size() << std::endl;
  for (uint ii=0; ii < msg.delta_x_d.size(); ii++)
    {
      params_local->delta_x_d[ii] = msg.delta_x_d[ii];
      // std::cerr << "delta_x_d_" <<  ii << ":\t" << msg.delta_x_d[ii] << std::endl;
    }

  for (uint ii=0; ii < msg.Q.size(); ii++)
    {
      params_local->Q[ii] = msg.Q[ii];
    }

  // std::cerr << "size of J :" << msg.J.size() << std::endl;
  for (uint ii=0; ii < msg.J.size(); ii++)
    {
      params_local->J[ii] = msg.J[ii];
      // std::cerr << "J_" <<  ii << ":\t" << msg.J[ii] << std::endl;
    }

  // std::cerr << "size of x_0 :" << msg.x_0.size() << std::endl;
  for (uint ii=0; ii < msg.x_0.size(); ii++)
    {
      params_local->x_0[ii] = msg.x_0[ii];
      // std::cerr << "x_0_" <<  ii << ":\t" << msg.x_0[ii] << std::endl;
    }

  // std::cerr << "size of KP_t_KP :" << msg.KP_t_KP.size() << std::endl;
  for (uint ii=0; ii < msg.KP_t_KP.size(); ii++)
    {
      params_local->KP_t_KP[ii] = msg.KP_t_KP[ii];
      // std::cerr << "KP_t_KP_" <<  ii << ":\t" << msg.KP_t_KP[ii] << std::endl;
    }

  // std::cerr << "sizes of q_min \t q_max \t u_min \t u_max :"  << std::endl;
  // std::cerr << msg.q_min.size() << "\t" << msg.q_max.size() << "\t" << msg.u_min.size() << "\t" << msg.u_max.size() << "\t" << std::endl;
  for (uint ii=0; ii < msg.u_min.size(); ii++)
    {
      // std::cerr << msg.q_min[ii] << "\t" << msg.q_max[ii] << "\t" << msg.u_min[ii] << "\t" << msg.u_max[ii]<<std::endl;
      params_local->q_min[ii] = msg.q_min[ii];
      params_local->q_max[ii] = msg.q_max[ii];
      params_local->u_min[ii] = msg.u_min[ii];
      params_local->u_max[ii] = msg.u_max[ii];
    }

  /* std::cerr << "size of dist_max :" << msg.dist_max.size() << std::endl; */
  /* std::cerr << "size of dist_min :" << msg.dist_min.size() << std::endl; */
  /* std::cerr << "size of desired_dist_increase :" << msg.desired_dist_increase.size() << std::endl; */

  for (uint ii=0; ii < msg.desired_dist_increase.size(); ii++)
    {
      params_local->desired_dist_increase[ii] = msg.desired_dist_increase[ii];
      params_local->dist_min[ii] = msg.dist_min[ii];
      params_local->dist_max[ii] = msg.dist_max[ii];
      // std::cerr << "f_min_" <<  ii << ":\t" << msg.f_min[ii] << std::endl;
      // std::cerr << "f_max_" <<  ii << ":\t" << msg.f_max[ii] << std::endl;
    }

  for (uint ii=0; ii < msg.n_J_ci.size(); ii++)
    {
      params_local->n_J_ci[ii] = msg.n_J_ci[ii];
      params_local->n_J_ci_max[ii] = msg.n_J_ci_max[ii];
    }

}


void MPC_QS_Controller::load_default_data() 
{
  std::cerr<<"there is no default data here yet, aborting ... " << std::endl;
}
