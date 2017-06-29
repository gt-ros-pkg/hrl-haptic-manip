#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <boost/thread/mutex.hpp>
#include "hrl_haptic_manipulation_in_clutter_msgs/SkinContact.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/BodyDraw.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/TaxelArray.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/MechanicalImpedanceParams.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/ContactTable.h"
#include "hrl_msgs/FloatArrayBare.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32MultiArray.h"
//#include "roslib/Clock.h"
#include <tf/transform_broadcaster.h>  
#include "rosgraph_msgs/Clock.h"
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <set>
#include <algorithm>
#include <functional>
#include <ode/ode.h>
#include <src/collision_util.h>
#include <sstream>
#include <map> 

//#include <drawstuff/drawstuff.h>
//#include "texturepath.h"


#ifdef dDOUBLE
#define MAX_CONTACTS 20          // maximum number of contact points per body
#define MAX_FEEDBACKNUM 100
#define NUM_OBST 1000
#define MAX_NUM_REV 30
#define MAX_NUM_PRISM 30
#define MAX_NUM_UNI 30
#define PI 3.14159265
#endif

#define FIXED_OBST_COST 100.0
#define MOVABLE_OBST_COST 1.0
#define COMPLIANT_OBST_COST 1.0

int num_used_movable = NUM_OBST;
int num_used_fixed = NUM_OBST;
int num_used_compliant = NUM_OBST;

boost::mutex m;


struct MyFeedback {
    dJointFeedback fb;
};

static int obst_table_ID[NUM_OBST];
static bool obst_table_contact[NUM_OBST];

class Simulator{
    public:
        //  bool got_image;
        Simulator(ros::NodeHandle &nh);
        ~Simulator();
	void controller_running_cb(const std_msgs::Bool msg);
	//void opt_finished_cb(const std_msgs::Bool msg);
        void JepCallback(const hrl_msgs::FloatArrayBare msg);
        void ImpedanceCallback(const hrl_haptic_manipulation_in_clutter_msgs::MechanicalImpedanceParams msg);
        void BaseEpCallback(const hrl_msgs::FloatArrayBare msg);
        void BaseImpedanceCallback(const hrl_haptic_manipulation_in_clutter_msgs::MechanicalImpedanceParams msg);
        void create_fixed_obstacles();
        void create_movable_obstacles();
        void create_compliant_obstacles();
        void create_robot();
        void sense_forces();
        void go_initial_position();
        void calc_torques();
        void set_torques();
        static void nearCallback (void *data, dGeomID o1, dGeomID o2);
        void classCallback (dGeomID o1, dGeomID o2);
        void publish_angle_data();
        void publish_base_pos_data();
        void publish_imped_skin_viz();
        void update_linkage_viz();
        void inner_torque_loop();
        void update_friction_and_obstacles();
        void get_joint_data();	
        void clear();
        void update_taxel_simulation(); 
        void setup_current_taxel_config(hrl_haptic_manipulation_in_clutter_msgs::TaxelArray &taxel);
        void update_proximity_simulation();
        ros::Publisher clock_pub; 
	double last_jep_time;
	bool controller_running;
        void publish_contact_table();
        static void init_contact_table(int index, int id, int contact);
        double get_dist(double x1, double y1, double x2, double y2, double radius);
	/* bool running; */
	/* ros::Publisher waiting_pub; */
	/* ros::Publisher done_waiting_pub; */

        dWorld world;
        dSimpleSpace space;
        dReal timestep;
        double cur_time;    

        bool use_mobile_base;

        hrl_msgs::FloatArrayBare angles;
        hrl_msgs::FloatArrayBare angle_rates;
        hrl_msgs::FloatArrayBare jep_ros;

        hrl_msgs::FloatArrayBare mobile_base_pos;
        hrl_msgs::FloatArrayBare mobile_base_pos_rates;
        hrl_msgs::FloatArrayBare mobile_base_ep_ros;
        
    protected:
        dHingeJoint manip_rev_jts[MAX_NUM_REV];
        dHingeJoint base_rev_jts[MAX_NUM_REV];
        dSliderJoint manip_pris_jts[MAX_NUM_PRISM];
        dSliderJoint base_pris_jts[MAX_NUM_PRISM];
        dUniversalJoint manip_uni_jts[MAX_NUM_UNI];
        dUniversalJoint base_uni_jts[MAX_NUM_UNI];

        dBody obstacles[NUM_OBST];
        dBody fixed_obstacles[NUM_OBST];
        dBody compliant_obstacles[NUM_OBST];
        double obst_home[NUM_OBST][3];
        double obst_stiffness[NUM_OBST];
        double obst_damping[NUM_OBST];

        dBody links_arr[100];  // just make a really big number ????, or resize the array if necessary instead
        double links_dim[100][3];
        std::string links_shape[100];
        dBox g_link_box[100];
        dCapsule g_link_cap[100];
        dSphere g_link_sph[100];

        int num_links;
        std::string links_name[100];
        std::string jts_type[100];

        dBodyID link_ids[100];
        //dBodyID fixed_obst_ids[NUM_OBST];
        //dBodyID movable_obst_ids[NUM_OBST];
        dJointID plane2d_joint_ids[NUM_OBST];
        dJointID compliant_plane2d_joint_ids[NUM_OBST];
        dJointID fixed_joint_ids[NUM_OBST];

        hrl_haptic_manipulation_in_clutter_msgs::ContactTable contact_table;

        dBodyID base_ids[100];
        dBody body_mobile_base[100];
        dBox geom_mobile_base[100];
        //dJointID mobile_base_plane2d_jt_id;

	bool use_prox_sensor;
        int num_jts;
	double resolution;
        ros::NodeHandle nh_;
        MyFeedback feedbacks[MAX_FEEDBACKNUM];
        MyFeedback frict_feedbacks[NUM_OBST];
        int fbnum;
        int force_group;
        double max_friction;
        double max_tor_friction;
        std::vector<double> q;
        std::vector<double> q_dot;
        std::vector<double> jep;
        std::vector<double> k_p;
        std::vector<double> k_d;
        std::vector<double> torques;

        std::vector<double> mobile_base_x;
        std::vector<double> mobile_base_x_dot;
        std::vector<double> mobile_base_ep; // x, y, theta
        std::vector<double> mobile_base_k_p;
        std::vector<double> mobile_base_k_d;
        std::vector<double> mobile_base_forces; // Fx, Fy, Tz

        //  std::vector<std::string> names[MAX_FEEDBACKNUM];
        std::vector<int> force_grouping;
        std::vector<int> force_sign;
        std::vector<double> pt_x;
        std::vector<double> pt_y;
        std::vector<double> pt_z;

        //Temporary variables that should be cleaned up with TF at some point///////
        std::vector<double> x_c;
        std::vector<double> y_c;
        std::vector<double> z_c;
        std::vector<double> x_n;
        std::vector<double> y_n;
        std::vector<double> z_n;

        dJointGroup joints;
        dBody *env;

        hrl_haptic_manipulation_in_clutter_msgs::SkinContact skin;
        hrl_haptic_manipulation_in_clutter_msgs::BodyDraw draw;
        hrl_haptic_manipulation_in_clutter_msgs::TaxelArray force_taxel;
        hrl_haptic_manipulation_in_clutter_msgs::TaxelArray proximity_taxel;
        hrl_haptic_manipulation_in_clutter_msgs::MechanicalImpedanceParams impedance_params;

        dSliderJoint *slider_x;
        dSliderJoint *slider_y;
        dFixedJoint *fixed_joint;
        dContactJoint *jt_contact;
        ros::Publisher angles_pub;
        ros::Publisher angle_rates_pub;
        ros::Publisher bodies_draw;
        ros::Publisher force_taxel_pub;
        ros::Publisher proximity_taxel_pub;
        ros::Publisher imped_pub;
        ros::Publisher skin_pub;
        ros::Publisher jep_pub;
        ros::Publisher controller_time_pub;
        ros::Publisher contact_table_pub;

        ros::Publisher mobile_base_pos_pub;
        ros::Publisher mobile_base_pos_rates_pub;
        ros::Publisher mobile_base_pos_ep_pub;

        
        ros::Subscriber controller_running_sub;

	boost::mutex m;

};

Simulator::Simulator(ros::NodeHandle &nh) :
    nh_(nh)
{
    num_used_movable = NUM_OBST;
    num_used_fixed = NUM_OBST;
    num_used_compliant = NUM_OBST;

    timestep = 0.0005; //0.0005;
    cur_time = 0.0;
    

    resolution = 0;
    use_prox_sensor = false;
    use_mobile_base = false;
    num_links = 0;
    num_jts = 0;
    while (nh_.getParam("/m3/software_testbed/linkage/num_links", num_links) == false)
        sleep(0.1);
    while (nh_.getParam("/m3/software_testbed/joints/num_joints", num_jts) == false)
        sleep(0.1);
    while (nh_.getParam("/use_prox_sensor", use_prox_sensor) == false)
        sleep(0.1);
    while (nh_.getParam("/use_mobile_base", use_mobile_base) == false)
        sleep(0.1);
    while (nh_.getParam("/m3/software_testbed/resolution", resolution) == false)
        sleep(0.1);

    fbnum=0;
    force_group=0;
    max_friction = 2;
    max_tor_friction = 0.5;
    angles_pub = nh_.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/joint_angles", 100);
    angle_rates_pub = nh_.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/joint_angle_rates", 100);  
    bodies_draw = nh_.advertise<hrl_haptic_manipulation_in_clutter_msgs::BodyDraw>("/sim_arm/bodies_visualization", 100);
    force_taxel_pub = nh_.advertise<hrl_haptic_manipulation_in_clutter_msgs::TaxelArray>("/skin/taxel_array", 100);
    proximity_taxel_pub = nh_.advertise<hrl_haptic_manipulation_in_clutter_msgs::TaxelArray>("/haptic_mpc/simulation/proximity/taxel_array", 100);
    imped_pub = nh_.advertise<hrl_haptic_manipulation_in_clutter_msgs::MechanicalImpedanceParams>("sim_arm/joint_impedance", 100);
    skin_pub = nh_.advertise<hrl_haptic_manipulation_in_clutter_msgs::SkinContact>("/skin/contacts", 100);
    jep_pub = nh_.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/jep", 100);
    clock_pub = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1/timestep);
    controller_time_pub = nh_.advertise<std_msgs::Float64>("/controller_time_step", 100);
    controller_running_sub = nh_.subscribe("/epc_skin/local_controller/controller_running", 100, &Simulator::controller_running_cb, this);
    contact_table_pub = nh_.advertise<hrl_haptic_manipulation_in_clutter_msgs::ContactTable>("/sim_arm/contact_table", 100);

    mobile_base_pos_pub       = nh_.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/base_pos", 100);
    mobile_base_pos_rates_pub = nh_.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/base_pos_rates", 100);  
    mobile_base_pos_ep_pub    = nh_.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/base_ep", 100);

    m.lock();
    for (int ii = 0; ii < num_jts; ii++)
    {
        q.push_back(0);
        q_dot.push_back(0);
        jep.push_back(0);
        k_p.push_back(0);
        k_d.push_back(0);
        torques.push_back(0);
    }
    
    for (int ii = 0; ii < 2; ii++)
    {
        mobile_base_x.push_back(0);
        mobile_base_x_dot.push_back(0);
        mobile_base_ep.push_back(0);
        mobile_base_k_p.push_back(100.0);
        mobile_base_k_d.push_back(1.0);
        mobile_base_forces.push_back(0);
    }

    m.unlock();
}

Simulator::~Simulator()
{
}

using namespace std;

void Simulator::controller_running_cb(const std_msgs::Bool msg)
{
  controller_running = msg.data;
  /* if (msg.data) */
  /*   { */
  /*     controller_running=true; */
  /*   } */
}


void Simulator::JepCallback(const hrl_msgs::FloatArrayBare msg)
{
    m.lock();
    jep = msg.data;
    std_msgs::Float64 time_step_cont;
    time_step_cont.data = cur_time-last_jep_time;
    controller_time_pub.publish(time_step_cont);
    last_jep_time = cur_time;
    m.unlock();
}

void Simulator::BaseEpCallback(const hrl_msgs::FloatArrayBare msg)
{
    m.lock();
    mobile_base_ep = msg.data;
    //std_msgs::Float64 time_step_cont;
    //time_step_cont.data = cur_time-last_jep_time;
    //controller_time_pub.publish(time_step_cont);
    //last_jep_time = cur_time;
    m.unlock();
}


void Simulator::ImpedanceCallback(const hrl_haptic_manipulation_in_clutter_msgs::MechanicalImpedanceParams msg)
{
    m.lock();
    k_p = msg.k_p.data;
    k_d = msg.k_d.data;
    m.unlock();
}

void Simulator::BaseImpedanceCallback(const hrl_haptic_manipulation_in_clutter_msgs::MechanicalImpedanceParams msg)
{
    m.lock();
    mobile_base_k_p = msg.k_p.data;
    mobile_base_k_d = msg.k_d.data;
    m.unlock();
}

void Simulator::nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    Simulator* obj = (Simulator*) data; // it looks unnecessarilly heavy object.
    int i;
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

    bool b1_is_link = false;
    bool b2_is_link = false;

    for (int i=0; i<obj->num_links; i++)
    {
        if (obj->link_ids[i] == b1)
            b1_is_link = true;
        if (obj->link_ids[i] == b2)
            b2_is_link = true;
    }

    // ignoring link-link contact.
    if (b1_is_link && b2_is_link)
        return;

    // ignoring base contact.
    for (int i=0; i<2; i++)
    {
        if (obj->base_ids[i] == b1)
            return;
        if (obj->base_ids[i] == b2)
            return;
    }

    bool arm_contact = false;
    stringstream ss;
    bool is_b1 = false;
    bool is_b2 = false;

    for (int i=0; i<obj->num_links; i++)
    {
        if (obj->link_ids[i] == b1 || obj->link_ids[i] == b2)
        {
            arm_contact = true;
            ss <<"link"<<i+1;
            if (obj->link_ids[i] == b1)
                is_b1 = true;
            else
                is_b2 = true;
        }
    }

    // Check object ids
    if (is_b1){
      //printf("%d\n", b2);
      for (int i=0; i<num_used_movable+num_used_fixed; i++){
        if (obst_table_ID[i] == (intptr_t)b2){
          obst_table_contact[i] = true;
          break;
        }
      }
    }
    else{
      //printf("%d\n", b1);
      for (int i=0; i<num_used_movable+num_used_fixed; i++){
        if (obst_table_ID[i] == (intptr_t)b1){
          obst_table_contact[i] = true;
          break;
        }
      }
    }
    

    dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
    int numc = dCollide (o1, o2, MAX_CONTACTS, &(contact[0].geom), sizeof(dContact));
    if (numc > 0)
    {
        if (arm_contact == false)
        {// here contact is between two objects.
            for (int i=0; i<numc; i++)
            {
                contact[i].surface.mode = dContactSoftCFM | dContactApprox1 | dContactSoftERP;
                contact[i].surface.mu = 0.2;
                contact[i].surface.mu2 = 0.2;
                //contact[i].surface.bounce = 0.01;
                contact[i].surface.soft_cfm = 0.04;
                contact[i].surface.soft_erp = 0.96;

                dJointID c = dJointCreateContact (obj->world,obj->joints.id(), &contact[i]);
                dJointAttach (c, dGeomGetBody(contact[i].geom.g1),
                        dGeomGetBody(contact[i].geom.g2));
            }
        }
        else
        { // here contact is between a link of the arm and an object.
            dVector3 contact_loc = {0, 0, 0};
            obj->skin.link_names.push_back(ss.str());

            for (i=0; i<numc; i++) 
            {
                contact[i].surface.mode = dContactSoftCFM | dContactApprox1 | dContactSoftERP;
                contact[i].surface.mu = 0.4;
                contact[i].surface.mu2 = 0.4;
                //contact[i].surface.bounce = 0.01;
                contact[i].surface.soft_cfm = 0.04;
                contact[i].surface.soft_erp = 0.96;

                contact_loc[0] = contact_loc[0]+contact[i].geom.pos[0];
                contact_loc[1] = contact_loc[1]+contact[i].geom.pos[1];
                contact_loc[2] = contact_loc[2]+contact[i].geom.pos[2];

                obj->pt_x.push_back(contact[i].geom.pos[0]);
                obj->pt_y.push_back(contact[i].geom.pos[1]);
                obj->pt_z.push_back(contact[i].geom.pos[2]);
                
                dJointID c = dJointCreateContact (obj->world,obj->joints.id(),&contact[i]);
                dJointAttach (c,b1,b2);

                if (obj->fbnum < MAX_FEEDBACKNUM)
                {
                    dJointSetFeedback (c, &obj->feedbacks[obj->fbnum++].fb);
                    obj->force_grouping.push_back(obj->force_group);
                    if (is_b1 == true)
                        obj->force_sign.push_back(1);
                    else
                        obj->force_sign.push_back(-1);
                }
            }

            contact_loc[0] = contact_loc[0]/numc;
            contact_loc[1] = contact_loc[1]/numc;
            contact_loc[2] = contact_loc[2]/numc;
            geometry_msgs::Point con_pt;
            con_pt.x = contact_loc[0];
            con_pt.y = contact_loc[1];
            con_pt.z = contact_loc[2];


            obj->skin.locations.push_back(con_pt);
            hrl_msgs::FloatArrayBare pts_x_ar;
            pts_x_ar.data = obj->pt_x;
            obj->skin.pts_x.push_back(pts_x_ar);
            hrl_msgs::FloatArrayBare pts_y_ar;
            pts_y_ar.data = obj->pt_y;
            obj->skin.pts_y.push_back(pts_y_ar);
            hrl_msgs::FloatArrayBare pts_z_ar;
            pts_z_ar.data = obj->pt_z;
            obj->skin.pts_z.push_back(pts_z_ar);
            obj->pt_x.clear();
            obj->pt_y.clear();
            obj->pt_z.clear();
            obj->force_group += 1;            

            // Check object ids
            float contact_cost;
            if (is_b1){
                if (obst_table_ID[0] <= (intptr_t)b2 &&
                    obst_table_ID[num_used_movable] > (intptr_t)b2)
                    contact_cost = MOVABLE_OBST_COST;
                else if (obst_table_ID[num_used_movable] <= (intptr_t)b2 &&
                         obst_table_ID[num_used_movable+num_used_compliant] > (intptr_t)b2)
                    contact_cost = COMPLIANT_OBST_COST;
                else
                    contact_cost = FIXED_OBST_COST;
            }
            else{
                if (obst_table_ID[0] <= (intptr_t)b1 &&
                    obst_table_ID[num_used_movable] > (intptr_t)b1)
                    contact_cost = MOVABLE_OBST_COST;
                else if (obst_table_ID[num_used_movable] <= (intptr_t)b1 &&
                         obst_table_ID[num_used_movable+num_used_compliant] > (intptr_t)b1)
                    contact_cost = COMPLIANT_OBST_COST;
                else
                    contact_cost = FIXED_OBST_COST;
            }
            obj->skin.costs.push_back(contact_cost);
        }
    }
}

void Simulator::publish_angle_data()
{
    angle_rates_pub.publish(angle_rates);
    angles_pub.publish(angles);
    jep_pub.publish(jep_ros);
}

void Simulator::publish_base_pos_data()
{
    mobile_base_pos_rates_pub.publish(mobile_base_pos_rates);
    mobile_base_pos_pub.publish(mobile_base_pos);
    mobile_base_pos_ep_pub.publish(mobile_base_ep_ros);
}

void Simulator::go_initial_position()
{
    XmlRpc::XmlRpcValue init_angle;
    while (nh_.getParam("/m3/software_testbed/joints/init_angle", init_angle) == false)
        sleep(0.1);

    m.lock();
    for (int ii = 0; ii < num_jts ; ii++)
    {
        jep[ii] = (double)init_angle[ii];
    }
    m.unlock();

    if (use_mobile_base)
    {
        XmlRpc::XmlRpcValue init_pos;
        while (nh_.getParam("/m3/software_testbed/joints/init_base_pos", init_pos) == false)
            sleep(0.1);
     
        m.lock();
        mobile_base_ep[0] = (double)init_pos[0];
        mobile_base_ep[1] = (double)init_pos[1];
        //mobile_base_ep[2] = 0.0; // angle is not implemented
        m.unlock();
    }


    float error = 1.;
    float error_thresh = 0.005;
    int torque_step = 0;

    while (error > error_thresh)
    {
        world.step(timestep);
        torque_step++;

        get_joint_data();
        calc_torques();
        set_torques();

        error = 0.;
        for (int ii = 0; ii < num_jts ; ii++)
        {
            if (jts_type[ii] == "hinge")
            {
                q[ii] = manip_rev_jts[ii].getAngle();
            }
            else if(jts_type[ii] == "slider"){
                q[ii] = manip_pris_jts[ii].getPosition();
            }                    

            error += (q[ii]-jep[ii])*(q[ii]-jep[ii]);
        }

        if (use_mobile_base)
        {
            for (int ii = 0; ii < 2 ; ii++)
            {
                mobile_base_x[ii] = base_pris_jts[ii].getPosition();
                error += (mobile_base_x[ii]-mobile_base_ep[ii])*(mobile_base_x[ii]-mobile_base_ep[ii]);
            }
        }

        /***********KEEP this for visualization if I ever need it while tuning gains*****/
        // if (torque_step >= 0.001/timestep)
        // {
        //     for (int jj = 0; jj < num_jts ; jj++)
        //     {
        //         ///////////////////////////////will this work?////////////////////////
        //         rosgraph_msgs::Clock c;
        //         cur_time += timestep;
        //         c.clock.sec = int(cur_time);
        //         c.clock.nsec = int(1000000000*(cur_time-int(cur_time)));
        //         clock_pub.publish(c);
        //         update_linkage_viz();
        //         publish_imped_skin_viz();
        //         clear();
        //         ///////////////////////////////will this work?////////////////////////

        //     }
        //     torque_step = 0;
        // }
        /***********KEEP this for visualization if I ever need it while tuning gains*****/
    }
}

void Simulator::sense_forces()
{
    geometry_msgs::Vector3 force;	
    geometry_msgs::Vector3 normal;	
    if (fbnum>MAX_FEEDBACKNUM)
    {
        printf("joint feedback buffer overflow!\n");
        assert(false);
    }
    else
    {
        dVector3 sum = {0, 0, 0};
        for (int i=0; i<fbnum; i++) 
        {
            dReal *f = feedbacks[i].fb.f2;
            //printf("force 1 %f %f %f\n", feedbacks[i].fb.f1[0], feedbacks[i].fb.f1[1], feedbacks[i].fb.f1[2]);
            //printf("force 2 %f %f %f\n", feedbacks[i].fb.f2[0], feedbacks[i].fb.f2[1], feedbacks[i].fb.f2[2]);
            sum[0] += f[0] * force_sign[i];
            sum[1] += f[1] * force_sign[i];
            sum[2] += f[2] * force_sign[i];
            if (i < fbnum-1)
            {
                if (force_grouping[i] != force_grouping[i+1])
                {
                    force.x = sum[0];
                    force.y = sum[1];
                    force.z = sum[2];
                    skin.forces.push_back(force);

                    // hacky code by Advait to add (fake) normals to the
                    // SkinContact message. The normal is not the
                    // vector normal to the surface of the arm.
                    double f_mag = sqrt(sum[0]*sum[0]+sum[1]*sum[1]+sum[2]*sum[2]);
                    normal.x = force.x / f_mag;
                    normal.y = force.y / f_mag;
                    normal.z = force.z / f_mag;
                    skin.normals.push_back(normal);
                    sum[0] = 0;
                    sum[1] = 0;
                    sum[2] = 0;
                }
                else
                {
                    ROS_WARN("Advait believes that this should never happen\n");
                    exit(0);
                }
            }			
            else
            {
                force.x = sum[0];
                force.y = sum[1];
                force.z = sum[2];
                skin.forces.push_back(force);

                // hacky code by Advait to add normals to the
                // SkinContact message
                double f_mag = sqrt(sum[0]*sum[0]+sum[1]*sum[1]+sum[2]*sum[2]);
                normal.x = force.x / f_mag;
                normal.y = force.y / f_mag;
                normal.z = force.z / f_mag;
                skin.normals.push_back(normal);

                sum[0] = 0;
                sum[1] = 0;
                sum[2] = 0;
            }
        }

    }
}

void Simulator::create_robot()
{
    //int num_links;
    XmlRpc::XmlRpcValue link_dimensions;
    while (nh_.getParam("/m3/software_testbed/linkage/dimensions", link_dimensions) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue link_pos;
    while (nh_.getParam("/m3/software_testbed/linkage/positions", link_pos) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue link_shapes;
    while (nh_.getParam("/m3/software_testbed/linkage/shapes", link_shapes) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue link_rot;
    while (nh_.getParam("/m3/software_testbed/linkage/rotations", link_rot) == false)
        sleep(0.1);
    
    XmlRpc::XmlRpcValue link_masses;
    while (nh_.getParam("/m3/software_testbed/linkage/mass", link_masses) == false)
        sleep(0.1);
    while (nh_.getParam("/m3/software_testbed/linkage/num_links", num_links) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue link_names;
    while (nh_.getParam("/m3/software_testbed/linkage/names", link_names) == false)
        sleep(0.1);


    //int num_jts;
    while (nh_.getParam("/m3/software_testbed/joints/num_joints", num_jts) == false)
        sleep(0.1);

    XmlRpc::XmlRpcValue jt_stiffness;
    while (nh_.getParam("/m3/software_testbed/joints/imped_params_stiffness", jt_stiffness) == false)
        sleep(0.1);

    XmlRpc::XmlRpcValue jt_damping;
    while (nh_.getParam("/m3/software_testbed/joints/imped_params_damping", jt_damping) == false)
        sleep(0.1);

    if ((uint)jt_stiffness.size() != k_p.size())
    {
      std::cerr << "THE JOINT STIFFNESS VECTOR AND NUMBER OF JOINTS IS NOT EQUAL" << std::endl;
      assert(false);
    }

    if ((uint)jt_damping.size() != k_d.size())
    {
      std::cerr << "THE JOINT DAMPING VECTOR AND NUMBER OF JOINTS IS NOT EQUAL" << std::endl;
      assert(false);
    }

    m.lock();
    for (int ii = 0; ii < num_jts; ii++)
    {
      k_p[ii] = (double)jt_stiffness[ii];
      k_d[ii] = (double)jt_damping[ii];
    }
    m.unlock();

    XmlRpc::XmlRpcValue jt_min;
    while (nh_.getParam("/m3/software_testbed/joints/min", jt_min) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue jt_max;
    while (nh_.getParam("/m3/software_testbed/joints/max", jt_max) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue jt_axes;
    while (nh_.getParam("/m3/software_testbed/joints/axes", jt_axes) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue jt_anchor;
    while (nh_.getParam("/m3/software_testbed/joints/anchor", jt_anchor) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue jt_attach;
    while (nh_.getParam("m3/software_testbed/joints/attach", jt_attach) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue jt_type;
    while (nh_.getParam("m3/software_testbed/joints/types", jt_type) == false)
        sleep(0.1);


    XmlRpc::XmlRpcValue base_jt_stiffness;
    XmlRpc::XmlRpcValue base_jt_damping;
    XmlRpc::XmlRpcValue base_jt_min;
    XmlRpc::XmlRpcValue base_jt_max;

    if (use_mobile_base)
    {
        while (nh_.getParam("/m3/software_testbed/joints/imped_params_base_stiffness", base_jt_stiffness) == false)
            sleep(0.1);
        while (nh_.getParam("/m3/software_testbed/joints/imped_params_base_damping", base_jt_damping) == false)
            sleep(0.1);
        while (nh_.getParam("/m3/software_testbed/joints/base_min", base_jt_min) == false)
            sleep(0.1);
        while (nh_.getParam("/m3/software_testbed/joints/base_max", base_jt_max) == false)
            sleep(0.1);

        m.lock();
        for (int ii = 0; ii < 2; ii++)
        {
            mobile_base_k_p[ii] = (double)base_jt_stiffness[ii];
            mobile_base_k_d[ii] = (double)base_jt_damping[ii];
        }
        m.unlock();

    }


    dMatrix3 body_rotate = {1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0};

    // Create mobile base
    if (use_mobile_base)
    {
        for (int ii = 0; ii < 2; ii++)
        { 
            dMass mass;            
            body_mobile_base[ii].create(world);
            body_mobile_base[ii].setPosition(0.0,0.0,0.0);  
            dMassSetBoxTotal(&mass, 10000.0, 0.01, 0.01, 0.01);	    
            body_mobile_base[ii].setMass(mass);
            geom_mobile_base[ii].create(space, 0.01, 0.01, 0.01);
            geom_mobile_base[ii].setBody(body_mobile_base[ii]);
            
            base_ids[ii] = body_mobile_base[ii].id();
        }
    }

    for (int ii = 0; ii < num_links; ii++)
    {
        //dBody link;
        dMass mass;
        links_shape[ii] = (string)link_shapes[ii];
        links_arr[ii].create(world);
        links_arr[ii].setPosition((double)link_pos[ii][0], (double)link_pos[ii][1], (double)link_pos[ii][2]);  
        links_dim[ii][0] = (double)link_dimensions[ii][0];
        links_dim[ii][1] = (double)link_dimensions[ii][1];
        links_dim[ii][2] = (double)link_dimensions[ii][2];
        links_name[ii] = (string)link_names[ii];

        //dBodySetPosition(link_ids[ii], (double)link_pos[ii][0], (double)link_pos[ii][1], (double)link_pos[ii][2]);  
        if (links_shape[ii] == "cube")
        {
            dMassSetBoxTotal(&mass, (double)link_masses[ii], (double)link_dimensions[ii][0], (double)link_dimensions[ii][1], (double)link_dimensions[ii][2]);	    
            links_arr[ii].setMass(mass);
            g_link_box[ii].create(space, (double)link_dimensions[ii][0], (double)link_dimensions[ii][1], (double)link_dimensions[ii][2]);
            g_link_box[ii].setBody(links_arr[ii]);

        }
        else if (links_shape[ii] == "capsule")
        {
            dMassSetCapsuleTotal(&mass, (double)link_masses[ii], 3, (double)link_dimensions[ii][0]/2.0, (double)link_dimensions[ii][2]);
            links_arr[ii].setMass(mass);
            g_link_cap[ii].create(space, (double)link_dimensions[ii][0]/2.0, (double)link_dimensions[ii][2]);
            g_link_cap[ii].setBody(links_arr[ii]);
        }
        else if (links_shape[ii] == "sphere")
        {
            dMassSetSphereTotal(&mass, (double)link_masses[ii], (double)link_dimensions[ii][2]/2.0);
            links_arr[ii].setMass(mass);
            g_link_sph[ii].create(space, (double)link_dimensions[ii][2]/2.0);
            g_link_sph[ii].setBody(links_arr[ii]);
        }
        else
        {
            std::cerr<<"wrong type of link shape was defined in config file,"<<links_shape[ii]<<" does not exist \n";
            assert(false);
        }

        // TODO!! multi-axis rotation
        if (((double)link_rot[ii][0] != 0.0) || ((double)link_rot[ii][1] != 0.0) || ((double)link_rot[ii][2] != 0.0))
        {
            if ((double)link_rot[ii][0] != 0.0)
              dRFromAxisAndAngle(body_rotate, 1, 0, 0, (double)link_rot[ii][0]);
            else if ((double)link_rot[ii][1] != 0.0)
              dRFromAxisAndAngle(body_rotate, 0, 1, 0, (double)link_rot[ii][1]);
            else if ((double)link_rot[ii][2] != 0.0)
              dRFromAxisAndAngle(body_rotate, 0, 0, 1, (double)link_rot[ii][2]);
            links_arr[ii].setRotation(body_rotate);
        }
        
        link_ids[ii] = links_arr[ii].id();
    }

    for (int ii = 0; ii < num_jts; ii++)
    {
        jts_type[ii] = (string)jt_type[ii];

        if (jt_type[ii] == "hinge")
        {
            manip_rev_jts[ii].create(world);
            int attach1, attach2;

            attach1 = (int)jt_attach[ii][0]; // Current Joint?
            attach2 = (int)jt_attach[ii][1]; // Last Joint?
            if (attach1 == -1 or attach2 == -1)
            {
                if (attach1 == -1)
                {
                    if (use_mobile_base == true)
                    {
                        manip_rev_jts[ii].attach(body_mobile_base[1], links_arr[attach2]);
                    }
                    else
                    {
                        manip_rev_jts[ii].attach(0, links_arr[attach2]);
                    }
                }
                else
                {
                    if (use_mobile_base == true)
                    {
                        manip_rev_jts[ii].attach(links_arr[attach1], body_mobile_base[1]);
                    }
                    else
                    {
                        manip_rev_jts[ii].attach(links_arr[attach1], 0);
                    }

                }

            }
            else
            {
                manip_rev_jts[ii].attach(links_arr[attach1], links_arr[attach2]);
            }

            manip_rev_jts[ii].setAnchor((double)jt_anchor[ii][0], (double)jt_anchor[ii][1], (double)jt_anchor[ii][2]);
            manip_rev_jts[ii].setAxis((double)jt_axes[ii][0], (double)jt_axes[ii][1], (double)jt_axes[ii][2]);
            dJointSetHingeParam(manip_rev_jts[ii].id(),dParamLoStop, (double)jt_min[ii]);
            dJointSetHingeParam(manip_rev_jts[ii].id(),dParamHiStop, (double)jt_max[ii]);
            // std::vector<dHingeJoint> manip_rev_jts;
            // std::vector<dSliderJoint> manip_pris_jts;
            // std::vector<dHingeJoint> base_rev_jts;
            // std::vector<dSliderJoint> base_pris_jts;

        }
        else if (jt_type[ii] == "slider")
        {
            manip_pris_jts[ii].create(world);
            int attach1, attach2;

            attach1 = (int)jt_attach[ii][0]; // Current Joint?
            attach2 = (int)jt_attach[ii][1]; // Last Joint?
            if (attach1 == -1 or attach2 == -1)
            {
                if (attach1 == -1)
                {
                    manip_pris_jts[ii].attach(0, links_arr[attach2]);
                }
                else
                {
                    manip_pris_jts[ii].attach(links_arr[attach1], 0);
                }
            }
            else
            {
                manip_pris_jts[ii].attach(links_arr[attach1], links_arr[attach2]);
            }

            manip_pris_jts[ii].setAxis((double)jt_axes[ii][0], (double)jt_axes[ii][1], (double)jt_axes[ii][2]);
            dJointSetSliderParam(manip_pris_jts[ii].id(),dParamLoStop, (double)jt_min[ii]);
            dJointSetSliderParam(manip_pris_jts[ii].id(),dParamHiStop, (double)jt_max[ii]);
            // std::vector<dHingeJoint> manip_rev_jts;
            // std::vector<dSliderJoint> manip_pris_jts;
            // std::vector<dHingeJoint> base_rev_jts;
            // std::vector<dSliderJoint> base_pris_jts;

        }
        else if (false)
        {
            manip_rev_jts[ii].create(world);
            int attach1, attach2;
            attach1 = (int)jt_attach[ii][0];
            attach2 = (int)jt_attach[ii][1];
            if (attach1 == -1 or attach2 == -1)
            {
                if (attach1 == -1)
                {
                    manip_rev_jts[ii].attach(0, links_arr[attach2]);
                }
                else
                {
                    manip_rev_jts[ii].attach(links_arr[attach1], 0);
                }
            }
            else
            {
                manip_rev_jts[ii].attach(links_arr[attach1], links_arr[attach2]);
            }
            manip_rev_jts[ii].setAnchor((double)jt_anchor[ii][0], (double)jt_anchor[ii][1], (double)jt_anchor[ii][2]);
            manip_rev_jts[ii].setAxis((double)jt_axes[ii][0], (double)jt_axes[ii][1], (double)jt_axes[ii][2]);
            dJointSetHingeParam(manip_rev_jts[ii].id(),dParamLoStop, (double)jt_min[ii]);
            dJointSetHingeParam(manip_rev_jts[ii].id(),dParamHiStop, (double)jt_max[ii]);

        }
    }

    if (use_mobile_base == true)
    {
        base_pris_jts[0].create(world);
        base_pris_jts[0].attach(0, body_mobile_base[0]);

        base_pris_jts[0].setAxis(-1.0, 0.0, 0.0);

        dJointSetSliderParam(base_pris_jts[0].id(),dParamLoStop, (double)base_jt_min[0]);
        dJointSetSliderParam(base_pris_jts[0].id(),dParamHiStop, (double)base_jt_max[0]);

        base_pris_jts[1].create(world);
        base_pris_jts[1].attach(body_mobile_base[0], body_mobile_base[1]);

        base_pris_jts[1].setAxis(0.0, -1.0, 0.0);
        dJointSetSliderParam(base_pris_jts[1].id(),dParamLoStop, (double)base_jt_min[1]);
        dJointSetSliderParam(base_pris_jts[1].id(),dParamHiStop, (double)base_jt_max[1]);
    }

    joints.create();

}

void Simulator::calc_torques()
{
    m.lock();
    for (int ii = 0; ii < num_jts; ii++)
    {
        torques[ii]=(-k_p[ii]*(q[ii]-jep[ii]) - k_d[ii]*q_dot[ii]);	
    }

    if (use_mobile_base)
    {
        for (int ii = 0; ii < 2; ii++)
        {
            mobile_base_forces[ii]=(-mobile_base_k_p[ii]*(mobile_base_x[ii]-mobile_base_ep[ii]) - mobile_base_k_d[ii]*mobile_base_x_dot[ii]);	
        }
    }

    m.unlock();
}

void Simulator::set_torques()
{
    //applying torques to joints
    for (int ii = 0; ii < num_jts ; ii++)
    {
        if (jts_type[ii] == "hinge")
        {
            manip_rev_jts[ii].addTorque(torques[ii]);
        }
        else if(jts_type[ii] == "slider")
        {
            manip_pris_jts[ii].addForce(torques[ii]);
        }
    }

    if (use_mobile_base)
    {
        for (int ii = 0; ii < 2; ii++)
        {
            base_pris_jts[ii].addForce(mobile_base_forces[ii]);
        }
    }

    // manip_rev_jts[0].addTorque(torques[0]);
    // manip_rev_jts[1].addTorque(torques[1]);
    // manip_rev_jts[2].addTorque(torques[2]);
}

void Simulator::update_linkage_viz()
{
    hrl_msgs::FloatArrayBare link_pos_ar;
    hrl_msgs::FloatArrayBare link_rot_ar;

    const dReal *position = links_arr[0].getPosition();
    const dReal *rotation = links_arr[0].getRotation();
    std::vector<double> pos_vec(3);
    std::vector<double> rot_vec(12);
    for (int l = 0; l<num_links; l++)
    {
        position = dBodyGetPosition(link_ids[l]);
        rotation = dBodyGetRotation(link_ids[l]);
        pos_vec[0] = position[0];
        pos_vec[1] = position[1];
        pos_vec[2] = position[2];
        link_pos_ar.data = pos_vec;
        draw.link_loc.push_back(link_pos_ar);
        for (int k = 0; k<12; k++)
        {
            rot_vec[k] = rotation[k];
        }
        link_rot_ar.data = rot_vec;
        draw.link_rot.push_back(link_rot_ar);
    }
}

void Simulator::update_friction_and_obstacles()
{
    hrl_msgs::FloatArrayBare obst_pos_ar;
    hrl_msgs::FloatArrayBare obst_rot_ar;
    const dReal *position;
    const dReal *rotation;

    std::vector<double> pos_vec(3);
    std::vector<double> rot_vec(12);

    for (int l = 0; l<num_used_movable; l++)
    {
        const dReal *tot_force = dBodyGetForce(obstacles[l].id());
        dReal *fric_force = frict_feedbacks[l].fb.f1;
        double force_xy_mag = sqrt((tot_force[0]-fric_force[0])*(tot_force[0]-fric_force[0])
                +(tot_force[1]-fric_force[1])*(tot_force[1]-fric_force[1]));
        if (force_xy_mag>0)
        {
            dJointSetPlane2DXParam(plane2d_joint_ids[l], dParamFMax, 
                    max_friction*abs(tot_force[0]-fric_force[0])/force_xy_mag);
            dJointSetPlane2DYParam(plane2d_joint_ids[l], dParamFMax, 
                    max_friction*abs(tot_force[1]-fric_force[1])/force_xy_mag);
        }
        else
        {
            dJointSetPlane2DXParam(plane2d_joint_ids[l], dParamFMax, 
                    max_friction*0.707);
            dJointSetPlane2DYParam(plane2d_joint_ids[l], dParamFMax, 
                    max_friction*0.707);
        }

        dJointSetPlane2DAngleParam(plane2d_joint_ids[l], dParamFMax, max_tor_friction);
        dJointSetPlane2DXParam(plane2d_joint_ids[l], dParamVel, 0.0);
        dJointSetPlane2DYParam(plane2d_joint_ids[l], dParamVel, 0.0);
        dJointSetPlane2DAngleParam(plane2d_joint_ids[l], dParamVel, 0.0);

        position = dBodyGetPosition(obstacles[l].id());
        rotation = dBodyGetRotation(obstacles[l].id());
        pos_vec[0] = position[0];
        pos_vec[1] = position[1];
        pos_vec[2] = position[2];
        obst_pos_ar.data = pos_vec;
        draw.obst_loc.push_back(obst_pos_ar);
        for (int k = 0; k<12; k++)
        {
            rot_vec[k] = rotation[k];
        }
        obst_rot_ar.data = rot_vec;
        draw.obst_rot.push_back(obst_rot_ar);
    }

    for (int l = 0; l<num_used_compliant; l++)
    {
        dJointSetPlane2DXParam(compliant_plane2d_joint_ids[l], dParamFMax, 0);
        dJointSetPlane2DYParam(compliant_plane2d_joint_ids[l], dParamFMax, 0);
        const dReal *cur_pos;
        cur_pos = dBodyGetPosition(compliant_obstacles[l].id());
        const dReal *cur_vel;
        cur_vel = dBodyGetLinearVel(compliant_obstacles[l].id());
        rotation = dBodyGetRotation(compliant_obstacles[l].id());  //this is to initialize correctly

        //this is the control law used to simulate compliant objects with critical damping
        dReal Fx = (obst_home[l][0]-cur_pos[0])*obst_stiffness[l] - cur_vel[0]*obst_damping[l];
        dReal Fy = (obst_home[l][1]-cur_pos[1])*obst_stiffness[l] - cur_vel[1]*obst_damping[l];

        dBodyAddForce(compliant_obstacles[l].id(), Fx, Fy, 0);
        //dBodyAddForce(compliant_obstacles[l].id(), 0, 0, 0);

        pos_vec[0] = cur_pos[0];
        pos_vec[1] = cur_pos[1];
        pos_vec[2] = cur_pos[2];

        obst_pos_ar.data = pos_vec;
        draw.obst_loc.push_back(obst_pos_ar);

        for (int k = 0; k<12; k++)
        {
            rot_vec[k] = rotation[k];
        }

        obst_rot_ar.data = rot_vec;
        draw.obst_rot.push_back(obst_rot_ar);
    }

    for (int l = 0; l<num_used_fixed; l++)
    {
        position = dBodyGetPosition(fixed_obstacles[l].id());
        rotation = dBodyGetRotation(fixed_obstacles[l].id());
        pos_vec[0] = position[0];
        pos_vec[1] = position[1];
        pos_vec[2] = position[2];
        obst_pos_ar.data = pos_vec;
        draw.obst_loc.push_back(obst_pos_ar);
        for (int k = 0; k<12; k++)
        {
            rot_vec[k] = rotation[k];
        }
        obst_rot_ar.data = rot_vec;
        draw.obst_rot.push_back(obst_rot_ar);
    }
}

void Simulator::publish_imped_skin_viz()
{
    impedance_params.header.frame_id = "/world";  //"/torso_lift_link";
    impedance_params.header.stamp = ros::Time::now();
    m.lock();
    impedance_params.k_p.data = k_p;
    impedance_params.k_d.data = k_d;
    m.unlock();
    imped_pub.publish(impedance_params);
    skin.header.frame_id = "/world"; //"/torso_lift_link";
    skin.header.stamp = ros::Time::now();
    skin_pub.publish(skin);
    draw.header.frame_id = "/world";
    draw.header.stamp = ros::Time::now();
    bodies_draw.publish(draw);
    force_taxel_pub.publish(force_taxel);
    proximity_taxel_pub.publish(proximity_taxel);
}

void Simulator::setup_current_taxel_config(hrl_haptic_manipulation_in_clutter_msgs::TaxelArray &taxel)
{
    taxel.header.frame_id = "/world";
    taxel.header.stamp = ros::Time::now();
    std::vector < string > taxel_links;
    

    for (int ii = 0; ii<num_links; ii++)
    {
        int k = 0;
        float link_width = links_dim[ii][0];
        float link_length = links_dim[ii][2];
        int num = floor(resolution*links_dim[ii][2])+1;  //LOOK HERE: this means that I need to rotate boxes too, or use y value instead?
        float step = 1.0/resolution;
        std::stringstream link_name;
        link_name << "link" << (ii+1);
        dVector3 global_pt;
        dVector3 global_n;

        dVector3 local_pt;
        dVector3 local_n;

        dVector3 base_pt;
        if (use_mobile_base)
        {
          base_pt[0] = 0.0; //mobile_base_pos.data[0];
          base_pt[1] = 0.0; //mobile_base_pos.data[1];
          base_pt[2] = 0.0;
        }
        else
        {
          base_pt[0] = 0.0;
          base_pt[1] = 0.0;
          base_pt[2] = 0.0;
        }


        while (k < num)
        {
            if (k < 2)
            {
                dBodyGetRelPointPos(links_arr[ii].id(), link_width/2.0,  0.0, (k/2)*step, global_pt);
                dVector3Subtract(global_pt,base_pt,local_pt);
                taxel.centers_x.push_back(local_pt[0]);
                taxel.centers_y.push_back(local_pt[1]);
                taxel.centers_z.push_back(local_pt[2]);
                taxel.link_names.push_back(link_name.str());
                taxel_links.push_back(link_name.str());

                dBodyGetRelPointPos(links_arr[ii].id(), -link_width/2.0, 0.0, (k/2)*step, global_pt);
                dVector3Subtract(global_pt,base_pt,local_pt);
                taxel.centers_x.push_back(local_pt[0]);
                taxel.centers_y.push_back(local_pt[1]);
                taxel.centers_z.push_back(local_pt[2]);
                taxel.link_names.push_back(link_name.str());
                taxel_links.push_back(link_name.str());

                dBodyVectorToWorld (links_arr[ii].id(), 1.0, 0.0, 0.0, global_n);
                dVector3Subtract(global_n,base_pt,local_n);
                taxel.normals_x.push_back(local_n[0]);
                taxel.normals_y.push_back(local_n[1]);
                taxel.normals_z.push_back(local_n[2]);

                dBodyVectorToWorld (links_arr[ii].id(), -1.0, 0.0, 0.0, global_n);
                dVector3Subtract(global_n,base_pt,local_n);
                taxel.normals_x.push_back(local_n[0]);
                taxel.normals_y.push_back(local_n[1]);
                taxel.normals_z.push_back(local_n[2]);
            }
            else
            {
                dBodyGetRelPointPos(links_arr[ii].id(), link_width/2.0, 0.0, (k/2)*step, global_pt);
                dVector3Subtract(global_pt,base_pt,local_pt);
                taxel.centers_x.push_back(local_pt[0]);
                taxel.centers_y.push_back(local_pt[1]);
                taxel.centers_z.push_back(local_pt[2]);
                taxel.link_names.push_back(link_name.str());
                taxel_links.push_back(link_name.str());

                dBodyGetRelPointPos(links_arr[ii].id(), -link_width/2.0, 0.0, (k/2)*step, global_pt);
                dVector3Subtract(global_pt,base_pt,local_pt);
                taxel.centers_x.push_back(local_pt[0]);
                taxel.centers_y.push_back(local_pt[1]);
                taxel.centers_z.push_back(local_pt[2]);
                taxel.link_names.push_back(link_name.str());
                taxel_links.push_back(link_name.str());


                dBodyGetRelPointPos(links_arr[ii].id(), link_width/2.0, 0.0, -(k/2)*step, global_pt);
                dVector3Subtract(global_pt,base_pt,local_pt);
                taxel.centers_x.push_back(local_pt[0]);
                taxel.centers_y.push_back(local_pt[1]);
                taxel.centers_z.push_back(local_pt[2]);
                taxel.link_names.push_back(link_name.str());
                taxel_links.push_back(link_name.str());

                dBodyGetRelPointPos(links_arr[ii].id(), -link_width/2.0, 0.0, -(k/2)*step, global_pt);
                dVector3Subtract(global_pt,base_pt,local_pt);
                taxel.centers_x.push_back(local_pt[0]);
                taxel.centers_y.push_back(local_pt[1]);
                taxel.centers_z.push_back(local_pt[2]);
                taxel.link_names.push_back(link_name.str());
                taxel_links.push_back(link_name.str());

                dBodyVectorToWorld (links_arr[ii].id(), 1.0, 0.0, 0.0, global_n);
                dVector3Subtract(global_n,base_pt,local_n);
                taxel.normals_x.push_back(local_n[0]);
                taxel.normals_y.push_back(local_n[1]);
                taxel.normals_z.push_back(local_n[2]);

                dBodyVectorToWorld (links_arr[ii].id(), -1.0, 0.0, 0.0, global_n);
                dVector3Subtract(global_n,base_pt,local_n);
                taxel.normals_x.push_back(local_n[0]);
                taxel.normals_y.push_back(local_n[1]);
                taxel.normals_z.push_back(local_n[2]);

                dBodyVectorToWorld (links_arr[ii].id(), 1.0, 0.0, 0.0, global_n);
                dVector3Subtract(global_n,base_pt,local_n);
                taxel.normals_x.push_back(local_n[0]);
                taxel.normals_y.push_back(local_n[1]);
                taxel.normals_z.push_back(local_n[2]);

                dBodyVectorToWorld (links_arr[ii].id(), -1.0, 0.0, 0.0, global_n);
                dVector3Subtract(global_n,base_pt,local_n);
                taxel.normals_x.push_back(local_n[0]);
                taxel.normals_y.push_back(local_n[1]);
                taxel.normals_z.push_back(local_n[2]);
            }
            k = k+2;
        }

        if (links_shape[ii] == "cube")
        {
            k = 0;
            num = floor(resolution*(link_width+0.0001))+1;

            while (k < num)
            {
                if (k < 2)
                {
                    dBodyGetRelPointPos(links_arr[ii].id(), (k/2)*step,  0.0, link_length/2.0, global_pt);
                    dVector3Subtract(global_pt,base_pt,local_pt);
                    taxel.centers_x.push_back(local_pt[0]);
                    taxel.centers_y.push_back(local_pt[1]);
                    taxel.centers_z.push_back(local_pt[2]);
                    taxel.link_names.push_back(link_name.str());
                    taxel_links.push_back(link_name.str());

                    dBodyGetRelPointPos(links_arr[ii].id(), (k/2)*step, 0.0, -link_length/2.0, global_pt);
                    dVector3Subtract(global_pt,base_pt,local_pt);
                    taxel.centers_x.push_back(local_pt[0]);
                    taxel.centers_y.push_back(local_pt[1]);
                    taxel.centers_z.push_back(local_pt[2]);
                    taxel.link_names.push_back(link_name.str());
                    taxel_links.push_back(link_name.str());

                    dBodyVectorToWorld (links_arr[ii].id(), 0.0, 0.0, 1.0, global_n);
                    dVector3Subtract(global_n,base_pt,local_n);
                    taxel.normals_x.push_back(local_n[0]);
                    taxel.normals_y.push_back(local_n[1]);
                    taxel.normals_z.push_back(local_n[2]);

                    dBodyVectorToWorld (links_arr[ii].id(), 0.0, 0.0, -1.0, global_n);
                    dVector3Subtract(global_n,base_pt,local_n);
                    taxel.normals_x.push_back(local_n[0]);
                    taxel.normals_y.push_back(local_n[1]);
                    taxel.normals_z.push_back(local_n[2]);
                }
                else
                {
                    dBodyGetRelPointPos(links_arr[ii].id(), (k/2)*step, 0.0, link_length/2.0, global_pt);
                    dVector3Subtract(global_pt,base_pt,local_pt);
                    taxel.centers_x.push_back(local_pt[0]);
                    taxel.centers_y.push_back(local_pt[1]);
                    taxel.centers_z.push_back(local_pt[2]);
                    taxel.link_names.push_back(link_name.str());
                    taxel_links.push_back(link_name.str());

                    dBodyGetRelPointPos(links_arr[ii].id(), (k/2)*step, 0.0, -link_length/2.0, global_pt);
                    dVector3Subtract(global_pt,base_pt,local_pt);
                    taxel.centers_x.push_back(local_pt[0]);
                    taxel.centers_y.push_back(local_pt[1]);
                    taxel.centers_z.push_back(local_pt[2]);
                    taxel.link_names.push_back(link_name.str());
                    taxel_links.push_back(link_name.str());


                    dBodyGetRelPointPos(links_arr[ii].id(), -(k/2)*step, 0.0, link_length/2.0, global_pt);
                    dVector3Subtract(global_pt,base_pt,local_pt);
                    taxel.centers_x.push_back(local_pt[0]);
                    taxel.centers_y.push_back(local_pt[1]);
                    taxel.centers_z.push_back(local_pt[2]);
                    taxel.link_names.push_back(link_name.str());
                    taxel_links.push_back(link_name.str());

                    dBodyGetRelPointPos(links_arr[ii].id(), -(k/2)*step, 0.0, -link_length/2.0, global_pt);
                    dVector3Subtract(global_pt,base_pt,local_pt);
                    taxel.centers_x.push_back(local_pt[0]);
                    taxel.centers_y.push_back(local_pt[1]);
                    taxel.centers_z.push_back(local_pt[2]);
                    taxel.link_names.push_back(link_name.str());
                    taxel_links.push_back(link_name.str());

                    dBodyVectorToWorld (links_arr[ii].id(), 0.0, 0.0, 1.0, global_n);
                    dVector3Subtract(global_n,base_pt,local_n);
                    taxel.normals_x.push_back(local_n[0]);
                    taxel.normals_y.push_back(local_n[1]);
                    taxel.normals_z.push_back(local_n[2]);

                    dBodyVectorToWorld (links_arr[ii].id(), 0.0, 0.0, -1.0, global_n);
                    dVector3Subtract(global_n,base_pt,local_n);
                    taxel.normals_x.push_back(local_n[0]);
                    taxel.normals_y.push_back(local_n[1]);
                    taxel.normals_z.push_back(local_n[2]);

                    dBodyVectorToWorld (links_arr[ii].id(), 0.0, 0.0, 1.0, global_n);
                    dVector3Subtract(global_n,base_pt,local_n);
                    taxel.normals_x.push_back(local_n[0]);
                    taxel.normals_y.push_back(local_n[1]);
                    taxel.normals_z.push_back(local_n[2]);

                    dBodyVectorToWorld (links_arr[ii].id(), 0.0, 0.0, -1.0, global_n);
                    dVector3Subtract(global_n,base_pt,local_n);
                    taxel.normals_x.push_back(local_n[0]);
                    taxel.normals_y.push_back(local_n[1]);
                    taxel.normals_z.push_back(local_n[2]);
                }
                k = k+2;
            }
        }
        else if (links_shape[ii] == "capsule")
        {
            float radius = link_width/2.0;
            k = 0;
            num = floor(resolution*(PI*radius))+1;
            float ang_step = PI/(num);
            while (k < num)
            {
                if (k < 2)
                {
                    dBodyGetRelPointPos(links_arr[ii].id(), 0.0,  0.0, link_length/2.0+radius, global_pt);
                    dVector3Subtract(global_pt,base_pt,local_pt);
                    taxel.centers_x.push_back(local_pt[0]);
                    taxel.centers_y.push_back(local_pt[1]);
                    taxel.centers_z.push_back(local_pt[2]);
                    taxel.link_names.push_back(link_name.str());
                    taxel_links.push_back(link_name.str());

                    dBodyGetRelPointPos(links_arr[ii].id(), 0.0, 0.0, -link_length/2.0-radius, global_pt);
                    dVector3Subtract(global_pt,base_pt,local_pt);
                    taxel.centers_x.push_back(local_pt[0]);
                    taxel.centers_y.push_back(local_pt[1]);
                    taxel.centers_z.push_back(local_pt[2]);
                    taxel.link_names.push_back(link_name.str());
                    taxel_links.push_back(link_name.str());

                    dBodyVectorToWorld (links_arr[ii].id(), 0.0, 0.0, 1.0, global_n);
                    dVector3Subtract(global_n,base_pt,local_n);
                    taxel.normals_x.push_back(local_n[0]);
                    taxel.normals_y.push_back(local_n[1]);
                    taxel.normals_z.push_back(local_n[2]);

                    dBodyVectorToWorld (links_arr[ii].id(), 0.0, 0.0, -1.0, global_n);
                    dVector3Subtract(global_n,base_pt,local_n);
                    taxel.normals_x.push_back(local_n[0]);
                    taxel.normals_y.push_back(local_n[1]);
                    taxel.normals_z.push_back(local_n[2]);
                }
                else
                {
                    dBodyGetRelPointPos(links_arr[ii].id(), radius*sin(ang_step*(k/2)),  0.0, link_length/2.0+radius*cos(ang_step*(k/2)), global_pt);
                    dVector3Subtract(global_pt,base_pt,local_pt);
                    taxel.centers_x.push_back(local_pt[0]);
                    taxel.centers_y.push_back(local_pt[1]);
                    taxel.centers_z.push_back(local_pt[2]);
                    taxel.link_names.push_back(link_name.str());
                    taxel_links.push_back(link_name.str());

                    dBodyGetRelPointPos(links_arr[ii].id(), radius*sin(ang_step*(k/2)),  0.0, -link_length/2.0-radius*cos(ang_step*(k/2)), global_pt);
                    dVector3Subtract(global_pt,base_pt,local_pt);
                    taxel.centers_x.push_back(local_pt[0]);
                    taxel.centers_y.push_back(local_pt[1]);
                    taxel.centers_z.push_back(local_pt[2]);
                    taxel.link_names.push_back(link_name.str());
                    taxel_links.push_back(link_name.str());

                    dBodyGetRelPointPos(links_arr[ii].id(), radius*sin(-ang_step*(k/2)),  0.0, link_length/2.0+radius*cos(-ang_step*(k/2)), global_pt);
                    dVector3Subtract(global_pt,base_pt,local_pt);
                    taxel.centers_x.push_back(local_pt[0]);
                    taxel.centers_y.push_back(local_pt[1]);
                    taxel.centers_z.push_back(local_pt[2]);
                    taxel.link_names.push_back(link_name.str());
                    taxel_links.push_back(link_name.str());

                    dBodyGetRelPointPos(links_arr[ii].id(), radius*sin(-ang_step*(k/2)),  0.0, -link_length/2.0-radius*cos(-ang_step*(k/2)), global_pt);
                    dVector3Subtract(global_pt,base_pt,local_pt);
                    taxel.centers_x.push_back(local_pt[0]);
                    taxel.centers_y.push_back(local_pt[1]);
                    taxel.centers_z.push_back(local_pt[2]);
                    taxel.link_names.push_back(link_name.str());
                    taxel_links.push_back(link_name.str());

                    dBodyVectorToWorld (links_arr[ii].id(), sin(ang_step*(k/2)), 0.0, cos(ang_step*(k/2)), global_n);
                    dVector3Subtract(global_n,base_pt,local_n);
                    taxel.normals_x.push_back(local_n[0]);
                    taxel.normals_y.push_back(local_n[1]);
                    taxel.normals_z.push_back(local_n[2]);

                    dBodyVectorToWorld (links_arr[ii].id(), sin(ang_step*(k/2)), 0.0, -cos(ang_step*(k/2)), global_n);
                    dVector3Subtract(global_n,base_pt,local_n);
                    taxel.normals_x.push_back(local_n[0]);
                    taxel.normals_y.push_back(local_n[1]);
                    taxel.normals_z.push_back(local_n[2]);

                    dBodyVectorToWorld (links_arr[ii].id(), sin(-ang_step*(k/2)), 0.0, cos(-ang_step*(k/2)), global_n);
                    dVector3Subtract(global_n,base_pt,local_n);
                    taxel.normals_x.push_back(local_n[0]);
                    taxel.normals_y.push_back(local_n[1]);
                    taxel.normals_z.push_back(local_n[2]);

                    dBodyVectorToWorld (links_arr[ii].id(), sin(-ang_step*(k/2)), 0.0, -cos(-ang_step*(k/2)), global_n);
                    dVector3Subtract(global_n,base_pt,local_n);
                    taxel.normals_x.push_back(local_n[0]);
                    taxel.normals_y.push_back(local_n[1]);
                    taxel.normals_z.push_back(local_n[2]);
                }
                k = k+2;
            }
        }
    }
}

double Simulator::get_dist(double x1, double y1, double x2, double y2, double radius)
{
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) - radius;
}

void Simulator::update_proximity_simulation()
{
  if (use_prox_sensor == true)
    {
      setup_current_taxel_config(proximity_taxel);
      proximity_taxel.sensor_type = "distance";
      
      const dReal *position;
      for (uint i=0; i < proximity_taxel.link_names.size(); i++)
	{
	  double min_dist = 0.35;
	  double x_taxel = proximity_taxel.centers_x[i];
	  double y_taxel = proximity_taxel.centers_y[i];
	  double nrml_x =  proximity_taxel.normals_x[i];
	  double nrml_y =  proximity_taxel.normals_y[i];

	  //this should be dynamic for reach object in future versions
	  double radius = 0.01;

	  //check every movable, fixed and compliant obstacle within 45 degree fov of taxel for min normal dist
	  for (int j=0; j < num_used_movable ; j++)
	    {
	      position = dBodyGetPosition(obstacles[j].id());
	      double cur_dist = get_dist(position[0], position[1], x_taxel, y_taxel, radius);
	      if ( cur_dist< min_dist)
		{
		  /* std::cerr << "cur_dist is smaller !!!!!!!!!!!!!!!!!!!!!!!! "<< std::endl; */
		  /* std::cerr<<"cur dist is : \t"<< cur_dist << std::endl; */
		  double dot_product = ((position[0]-x_taxel)*nrml_x + (position[1]-y_taxel)*nrml_y)/sqrt(pow(position[0]-x_taxel, 2) + pow(position[1]-y_taxel, 2));

		  if (dot_product > 0.7071)
		    {
		      min_dist = cur_dist;
		    }
		}
	    }
	  for (int j=0; j < num_used_fixed ; j++)
	    {
	      position = dBodyGetPosition(fixed_obstacles[j].id());
	      double cur_dist = get_dist(position[0], position[1], x_taxel, y_taxel, radius);
	      if ( cur_dist< min_dist)
		{
		  double dot_product = ((position[0]-x_taxel)*nrml_x + (position[1]-y_taxel)*nrml_y)/sqrt(pow(position[0]-x_taxel, 2) + pow(position[1]-y_taxel, 2));
		  if (dot_product > 0.7071)
		    {
		      min_dist = cur_dist;
		    }
		}
	    }
	  for (int j=0; j < num_used_compliant ; j++)
	    {
	      position = dBodyGetPosition(compliant_obstacles[j].id());
	      double cur_dist = get_dist(position[0], position[1], x_taxel, y_taxel, radius);
	      if ( cur_dist< min_dist)
		{
		  double dot_product = ((position[0]-x_taxel)*nrml_x + (position[1]-y_taxel)*nrml_y)/sqrt(pow(position[0]-x_taxel, 2) + pow(position[1]-y_taxel, 2));
		  if (dot_product > 0.7071)
		    {
		      min_dist = cur_dist;
		    }
		}
	    }
	
	  proximity_taxel.values_x.push_back(min_dist*nrml_x);
	  proximity_taxel.values_y.push_back(min_dist*nrml_y);
	  proximity_taxel.values_z.push_back(0.);
	}
    }    
}	       

void Simulator::update_taxel_simulation()
{
    setup_current_taxel_config(force_taxel);
    force_taxel.sensor_type = "force";

    //doing nearest neighbor to assign forces to discrete taxels
    std::vector < int > f_ind;
    for (unsigned int j = 0; j < skin.forces.size(); j++)
    {
        float min_distance = 10000;
        // sometimes the magnitude of the contact force is zero. This
        // causes ind_buf to be erroneously set to 0. In the skin
        // client (python code -- epc_skin.py), Advait is ignoring
        // forces less than 0.1N.
        // Previously, Advait ignored low forces within
        // demo_kinematic, initialized ind_buf to a -ve number and had
        // an assertion to detect strange things.
        // Unfortunately, only skin.forces and normals is getting
        // populated here. The rest happens in the nearCallback and so
        // there is no good way that Advait can think of of keeping
        // the link_names, locations and forces consistent. Removing
        // low forces in the python skin client appears to be an
        // okayish solution.
        int ind_buf = -134241;

        for (unsigned int k = 0; k <  force_taxel.centers_x.size(); k++)
        {
            float distance = sqrt(pow((force_taxel.centers_x[k]-skin.locations[j].x),2)+pow((force_taxel.centers_y[k]-skin.locations[j].y),2)+pow((force_taxel.centers_z[k]-skin.locations[j].z),2));
            if (distance < min_distance && force_taxel.link_names[k] == skin.link_names[j])
            {
                double mag_norm = sqrt(force_taxel.normals_x[k]*force_taxel.normals_x[k]+force_taxel.normals_y[k]*force_taxel.normals_y[k]);
                double mag_force = sqrt(skin.forces[j].x*skin.forces[j].x + skin.forces[j].y*skin.forces[j].y);

                double dot_prod = (force_taxel.normals_x[k]*skin.forces[j].x + force_taxel.normals_y[k]*skin.forces[j].y);
                double cos_angle = dot_prod/(mag_norm*mag_force);
                double abs_angle_deg = abs(acos(cos_angle) * 180.0 / PI);

                if (mag_force < 0.01)
                    abs_angle_deg = 0.;

                // here the force is close to a corner. only
                // using taxels for which the angle b/w the normal
                // and the force vector is less than some
                // threshold.
                if (abs_angle_deg <= 45.0)
                {
                    // we are guaranteed to have atleast one such
                    // taxel, because we have at least one taxel
                    // on both the front and side surfaces of the
                    // arm.
                    min_distance = distance;
                    ind_buf = k;
                }
            }
        }

        assert(ind_buf >= 0);
        f_ind.push_back(ind_buf);
    }

    // it is better to use other initialization method.
    for (unsigned int k = 0; k < force_taxel.centers_x.size(); k++)
    {
        force_taxel.values_x.push_back(0.0);
        force_taxel.values_y.push_back(0.0);
        force_taxel.values_z.push_back(0.0);
        force_taxel.contact_cost.push_back(0.0);
    }


    for (unsigned int j = 0; j <  f_ind.size(); j++)
    {
//        std::cerr << "f_ind[j] = " << f_ind[j]
//                  << ", contact_types_list.size() = " << contact_types_list.size() 
//                  << ", "
        
        force_taxel.values_x[f_ind[j]] = force_taxel.values_x[f_ind[j]] + skin.forces[j].x;
        force_taxel.values_y[f_ind[j]] = force_taxel.values_y[f_ind[j]] + skin.forces[j].y;
        force_taxel.values_z[f_ind[j]] = force_taxel.values_z[f_ind[j]] + skin.forces[j].z;
        force_taxel.contact_cost[f_ind[j]] = skin.costs[j];
    }

    f_ind.clear();
    /*************************************************************************************/
    /*end of really bad code at least, all code above for taxel stuff needs drastic rework*/
    /****************************************************************************************/
}

void Simulator::clear()
{
    draw.link_loc.clear();
    draw.link_rot.clear();
    draw.obst_loc.clear();
    draw.obst_rot.clear();

    force_taxel.centers_x.clear();
    force_taxel.centers_y.clear();
    force_taxel.centers_z.clear();
    force_taxel.normals_x.clear();
    force_taxel.normals_y.clear();
    force_taxel.normals_z.clear();
    force_taxel.values_x.clear();
    force_taxel.values_y.clear();
    force_taxel.values_z.clear();
    force_taxel.link_names.clear();
    force_taxel.contact_cost.clear();

    proximity_taxel.centers_x.clear();
    proximity_taxel.centers_y.clear();
    proximity_taxel.centers_z.clear();
    proximity_taxel.normals_x.clear();
    proximity_taxel.normals_y.clear();
    proximity_taxel.normals_z.clear();
    proximity_taxel.values_x.clear();
    proximity_taxel.values_y.clear();
    proximity_taxel.values_z.clear();
    proximity_taxel.link_names.clear();

    skin.pts_x.clear();
    skin.pts_y.clear();
    skin.pts_z.clear();
    skin.link_names.clear();
    skin.locations.clear();
    skin.forces.clear();
    skin.normals.clear();
    skin.costs.clear();
    force_grouping.clear();
    force_sign.clear();
    joints.clear();

    fbnum = 0;
    force_group = 0;
}

void Simulator::get_joint_data()	
{
    for (int ii = 0; ii < num_jts ; ii++)
    {
        if (jts_type[ii] == "hinge")
        {
            q[ii] = manip_rev_jts[ii].getAngle();
            q_dot[ii] = manip_rev_jts[ii].getAngleRate();
        }
        else if(jts_type[ii] == "slider"){
            q[ii] = manip_pris_jts[ii].getPosition();
            q_dot[ii] = manip_pris_jts[ii].getPositionRate();
        }
    }

    if (use_mobile_base)
    {
        for (int ii = 0; ii < 2 ; ii++)
        {
            mobile_base_x[ii] = base_pris_jts[ii].getPosition();
            mobile_base_x_dot[ii] = base_pris_jts[ii].getPositionRate();
        }
    }

    angles.data = q;
    angle_rates.data = q_dot;

    if (use_mobile_base)
    {
        mobile_base_pos.data = mobile_base_x;
        mobile_base_pos_rates.data = mobile_base_x_dot;
    }

    m.lock();   
    jep_ros.data = jep;
    if (use_mobile_base)
    {
        mobile_base_ep_ros.data = mobile_base_ep;
    }
    m.unlock();
}

void Simulator::create_movable_obstacles()
{
    float obstacle_mass = 1.;

    // this needs to be the first param that is read for
    // synchronization with obstacles.py
    XmlRpc::XmlRpcValue cylinders_dim;
    while (nh_.getParam("/m3/software_testbed/movable_dimen", cylinders_dim) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue cylinders_pos;
    while (nh_.getParam("/m3/software_testbed/movable_position", cylinders_pos) == false)
        sleep(0.1);
    while(nh_.getParam("/m3/software_testbed/num_movable", num_used_movable) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue obst_IDs;
    obst_IDs.setSize(num_used_fixed);

    XmlRpc::XmlRpcValue cylinders_max_force;     
    bool got_max_force;
    got_max_force = nh_.getParam("/m3/software_testbed/movable_max_force", cylinders_max_force);

    for (int i = 0; i < num_used_movable; i++)
    {
        dMass m_obst;
        dCapsule *geom_cyl;

        dMassSetCapsuleTotal(&m_obst, obstacle_mass, 3,
                (double)cylinders_dim[i][0], (double)cylinders_dim[i][2]);

        obstacles[i].create(world);
        obstacles[i].setPosition((double)cylinders_pos[i][0], (double)cylinders_pos[i][1], (double)cylinders_pos[i][2]);
        obstacles[i].setMass(&m_obst);
        plane2d_joint_ids[i] = dJointCreatePlane2D(world.id(), 0);
        dJointAttach(plane2d_joint_ids[i], obstacles[i].id(), 0);

        if (got_max_force == false)
        {
            dJointSetPlane2DXParam(plane2d_joint_ids[i], dParamFMax, max_friction*0.707);
            dJointSetPlane2DYParam(plane2d_joint_ids[i], dParamFMax, max_friction*0.707);
            dJointSetPlane2DAngleParam(plane2d_joint_ids[i], dParamFMax, max_tor_friction);
        }
        else
        {
            dJointSetPlane2DXParam(plane2d_joint_ids[i], dParamFMax, (double)cylinders_max_force[i]*0.707);
            dJointSetPlane2DYParam(plane2d_joint_ids[i], dParamFMax, (double)cylinders_max_force[i]*0.707);
            dJointSetPlane2DAngleParam(plane2d_joint_ids[i], dParamFMax, max_tor_friction);
        }

        dJointSetPlane2DXParam(plane2d_joint_ids[i], dParamVel, 0.0);
        dJointSetPlane2DYParam(plane2d_joint_ids[i], dParamVel, 0.0);
        dJointSetPlane2DAngleParam(plane2d_joint_ids[i], dParamVel, 0.0);

        geom_cyl = new dCapsule(space, (double)cylinders_dim[i][0], (double)cylinders_dim[i][2]);
        geom_cyl->setBody(obstacles[i]);

        dJointSetFeedback(plane2d_joint_ids[i], &frict_feedbacks[i].fb);
        
        dBodyID current_body_id = obstacles[i].id();
        
        //obstacles.push_back(obstacle);
        
        //movable_obst_ids[i] = obstacles[i].id();
        //obst_table_ID[i] = (int)(obstacles[i].id());
        init_contact_table(i, (intptr_t)(obstacles[i].id()), 0);
        obst_IDs[i] = (int)((intptr_t)(obstacles[i].id()));
    }
    nh_.setParam("/m3/software_testbed/movable_id", obst_IDs);
}

void Simulator::create_compliant_obstacles()
{
    float obstacle_mass = 1.;

    XmlRpc::XmlRpcValue cylinders_dim;
    while (nh_.getParam("/m3/software_testbed/compliant_dimen", cylinders_dim) == false)
        sleep(0.1);

    XmlRpc::XmlRpcValue cylinders_pos;
    nh_.getParam("/m3/software_testbed/compliant_position", cylinders_pos);

    XmlRpc::XmlRpcValue cylinders_stiffness;
    bool got_stiffness;
    got_stiffness = nh_.getParam("/m3/software_testbed/compliant_stiffness_value", cylinders_stiffness);

    nh_.getParam("/m3/software_testbed/num_compliant", num_used_compliant);

    for (int i = 0; i < num_used_compliant; i++)
    {
        dMass m_obst;
        dCapsule *geom_cyl;

        if(got_stiffness == true)
        {
            obst_stiffness[i]= (double)cylinders_stiffness[i];

            //we calculate required damping using a
            //damping ratio of 1 (critically damped)
            obst_damping[i] = 2*sqrt(obst_stiffness[i]*obstacle_mass);
        }
        else
        {
            obst_stiffness[i]= -1;
        }

        dMassSetCapsuleTotal(&m_obst, obstacle_mass, 3, (double)cylinders_dim[i][0], (double)cylinders_dim[i][2]);
        compliant_obstacles[i].create(world);
        compliant_obstacles[i].setPosition((double)cylinders_pos[i][0], (double)cylinders_pos[i][1], (double)cylinders_pos[i][2]);
        obst_home[i][0] = (double)cylinders_pos[i][0];
        obst_home[i][1] = (double)cylinders_pos[i][1];
        obst_home[i][2] = (double)cylinders_pos[i][2];
        compliant_obstacles[i].setMass(&m_obst);
        compliant_plane2d_joint_ids[i] = dJointCreatePlane2D(world.id(), 0);
        dJointAttach(compliant_plane2d_joint_ids[i], compliant_obstacles[i].id(), 0);
        dJointSetPlane2DXParam(compliant_plane2d_joint_ids[i], dParamVel, 0.0);
        dJointSetPlane2DYParam(compliant_plane2d_joint_ids[i], dParamVel, 0.0);
        dJointSetPlane2DAngleParam(compliant_plane2d_joint_ids[i], dParamVel, 0.0);

        geom_cyl = new dCapsule(space, (double)cylinders_dim[i][0], (double)cylinders_dim[i][2]);
        geom_cyl->setBody(compliant_obstacles[i]);
        dBodyID current_body_id = compliant_obstacles[i].id();
    }
}

void Simulator::create_fixed_obstacles()
{
    float obstacle_mass = 1.;

    while(nh_.getParam("/m3/software_testbed/num_fixed", num_used_fixed) == false)
        sleep(0.1);

    XmlRpc::XmlRpcValue fixed_pos;
    nh_.getParam("/m3/software_testbed/fixed_position", fixed_pos);
    XmlRpc::XmlRpcValue fixed_dim;
    nh_.getParam("/m3/software_testbed/fixed_dimen", fixed_dim);
    XmlRpc::XmlRpcValue fixed_ctype;
    nh_.getParam("/m3/software_testbed/fixed_ctype", fixed_ctype);
    XmlRpc::XmlRpcValue obst_IDs;
    obst_IDs.setSize(num_used_fixed);

    for (int i = 0; i < num_used_fixed; i++)
    {
        dMass m_obst;
        dMassSetCapsuleTotal(&m_obst, obstacle_mass, 3,
                (double)fixed_dim[i][0], (double)fixed_dim[i][2]);

        fixed_obstacles[i].create(world);
        fixed_obstacles[i].setPosition((double)fixed_pos[i][0], (double)fixed_pos[i][1], (double)fixed_pos[i][2]);
        fixed_obstacles[i].setMass(&m_obst);

	// It looks temporal code that conflicts with wall type obstacles. 
        //fixed_joint_ids[i] = dJointCreateFixed(world.id(), 0);
        //dJointAttach(fixed_joint_ids[i], fixed_obstacles[i].id(), 0);
        //dJointSetFixed(fixed_joint_ids[i]);

        //dCapsule *geom_fixed;
        //geom_fixed = new dCapsule(space, (double)fixed_dim[i][0], (double)fixed_dim[i][2]);
        //geom_fixed->setBody(fixed_obstacles[i]);
        ////	fixed_obstacles.push_back(fixed_obstacle);

        if (static_cast<std::string>(fixed_ctype[i]) == "wall")
        {
            dBox *geom_fixed;
            double theta = (double)fixed_pos[i][3];
            dMatrix3 obstacle_rotate = {cos(theta),-sin(theta),0,0,sin(theta),cos(theta),0,0,0,0,1.0,0};
            fixed_obstacles[i].setRotation(obstacle_rotate);

            fixed_joint_ids[i] = dJointCreateFixed(world.id(), 0);
            dJointAttach(fixed_joint_ids[i], fixed_obstacles[i].id(), 0);        
            dJointSetFixed(fixed_joint_ids[i]);

            geom_fixed = new dBox(space, (double)fixed_dim[i][0], (double)fixed_dim[i][1], (double)fixed_dim[i][2]);
            geom_fixed->setBody(fixed_obstacles[i]);
        }
        else
        {
            dCapsule *geom_fixed;

            fixed_joint_ids[i] = dJointCreateFixed(world.id(), 0);
            dJointAttach(fixed_joint_ids[i], fixed_obstacles[i].id(), 0);        
            dJointSetFixed(fixed_joint_ids[i]);

            geom_fixed = new dCapsule(space, (double)fixed_dim[i][0], (double)fixed_dim[i][2]);
            geom_fixed->setBody(fixed_obstacles[i]);
        }
        
        // Add every body id to the contact type map specifying these as rigid.
        dBodyID current_body_id = fixed_obstacles[i].id();
        
        
        //	fixed_obstacles.push_back(fixed_obstacle);

        //fixed_obst_ids[i] = fixed_obstacles[i].id();
        //obst_table_ID[num_used_movable+i] = (int)(fixed_obstacles[i].id());
        init_contact_table(num_used_movable+i, (intptr_t)(fixed_obstacles[i].id()), 0);
        obst_IDs[i] = (int)((intptr_t)(fixed_obstacles[i].id()));
    }

    nh_.setParam("/m3/software_testbed/fixed_id", obst_IDs);
}

void Simulator::publish_contact_table()
{
    contact_table.id.data.clear();
    contact_table.contact.data.clear();
     
    for (int i=0; i<num_used_movable+num_used_fixed; i++)
    {
      contact_table.id.data.push_back(i);
      contact_table.contact.data.push_back(obst_table_contact[i]);
    }

    contact_table_pub.publish(contact_table);
}

void Simulator::init_contact_table(int index, int id, int contact)
{
    obst_table_ID[index]      = id;
    obst_table_contact[index] = contact;
}

double get_wall_clock_time()
{
    timeval tim;
    gettimeofday(&tim, NULL);
    double t1=tim.tv_sec+(tim.tv_usec/1000000.0);
    return t1;
}


