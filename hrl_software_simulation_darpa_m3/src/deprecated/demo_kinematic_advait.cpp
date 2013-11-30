
#include <sys/time.h>

#include "ros/ros.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/SkinContact.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/BodyDraw.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/TaxelArray.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/MechanicalImpedanceParams.h"
#include "hrl_msgs/FloatArrayBare.h"
#include "std_msgs/String.h"
//#include "roslib/Clock.h"
#include "rosgraph_msgs/Clock.h"
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <set>
#include <algorithm>
#include <functional>
#include <ode/ode.h>
#include <sstream>
//#include <drawstuff/drawstuff.h>
//#include "texturepath.h"

#include <tf/transform_broadcaster.h>

#ifdef dDOUBLE
#define MAX_CONTACTS 20          // maximum number of contact points per body
#define MAX_FEEDBACKNUM 100
#define NUM_OBST 1000
#define PI 3.14159265
#endif


struct MyFeedback {
    dJointFeedback fb;
};

static MyFeedback feedbacks[MAX_FEEDBACKNUM];
static MyFeedback frict_feedbacks[NUM_OBST];
static int fbnum=0;
static int force_group=0;
double max_friction = 2;
double max_tor_friction = 0.5;
int num_used_movable = NUM_OBST;
int num_used_fixed = NUM_OBST;
int num_used_compliant = NUM_OBST;

using namespace std;
std::vector<double> q(3, 0);
std::vector<double> q_dot(3, 0);
std::vector<std::string> names(MAX_FEEDBACKNUM);
std::vector<int> force_grouping;
std::vector<int> force_sign;
std::vector<double> pt_x;
std::vector<double> pt_y;
std::vector<double> pt_z;
std::vector<double> jep(3, 0);
std::vector<double> k_p(3, 0);
std::vector<double> k_d(3, 0);
std::vector<double> torques(3, 0);

std::vector<double> mobile_base_k_p(3, 0);
std::vector<double> mobile_base_k_d(3, 0);
std::vector<double> mobile_base_ep(3, 0); // x, y, theta
std::vector<double> mobile_base_generalized_forces(3, 0); // Fx, Fy, Tz

//Temporary variables that should be cleaned up with TF at some point///////
std::vector<double> x_c;
std::vector<double> y_c;
std::vector<double> z_c;
std::vector<double> x_n;
std::vector<double> y_n;
std::vector<double> z_n;
float torso_half_width = 0.196; // ctorsoy
float torso_half_width_side = 0;
float upper_arm_length = 0.334; // ruparmz
float upper_arm_width = 0;
float forearm_length = 0;//0.288; //+ 0.115; // rlowarmz + rhandz
float forearm_width = 0;

///////////////////////////////////////////////////////////////////////////

//std::vector<dBodyID> link_ids(3);
dBodyID link_ids[3];
dBody obstacles[NUM_OBST];
dBody compliant_obstacles[NUM_OBST];
double obst_home[NUM_OBST][3];
double obst_stiffness[NUM_OBST];
double obst_damping[NUM_OBST];
dBodyID fixed_obst_ids[NUM_OBST];
dBody fixed_obstacles[NUM_OBST];
dJointID plane2d_joint_ids[NUM_OBST];
dJointID compliant_plane2d_joint_ids[NUM_OBST];
dJointID fixed_joint_ids[NUM_OBST];

dJointGroup joints;
dBody *env;
dWorld *world;
dSpace *space;


// body, geometry, joint
// (body is for dynamics, geometry is for collision detection) -- This
// is Advait, so the documentation might be incorrect.

dBody *link1;
dBox *g_link1;
dHingeJoint *hinge1;

dBody *link2;
dBox *g_link2;
dHingeJoint *hinge2;

dBody *link3;
dBox *g_link3;
dHingeJoint *hinge3;

int include_mobile_base;

dBody *body_mobile_base;
dBox *geom_mobile_base;
dJointID mobile_base_plane2d_jt_id;


hrl_haptic_manipulation_in_clutter_msgs::SkinContact skin;
hrl_haptic_manipulation_in_clutter_msgs::BodyDraw draw;
hrl_haptic_manipulation_in_clutter_msgs::TaxelArray taxel;
hrl_haptic_manipulation_in_clutter_msgs::MechanicalImpedanceParams impedance_params;

void inner_torque_loop();


void base_ep_cb(const hrl_msgs::FloatArrayBare msg)
{
    mobile_base_ep = msg.data;
}

void jep_cb(const hrl_msgs::FloatArrayBare msg)
{
    jep = msg.data;
    //jep[0] = msg.data[0];	
    //jep[1] = msg.data[1];	
    //jep[2] = msg.data[2];      
}

void ROSCallback_impedance(const hrl_haptic_manipulation_in_clutter_msgs::MechanicalImpedanceParams msg)
{
    k_p = msg.k_p.data;
    k_d = msg.k_d.data;
}



static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
    int i;
    // if (o1->body && o2->body) return;                                                                                 
    //    // if contact with ground, then ignore.

    // exit without doing anything if the two bodies are connected by a joint                                            
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

    bool b1_is_link = false;
    bool b2_is_link = false;
    for (int i=0; i<3; i++)
    {
        if (link_ids[i] == b1)
            b1_is_link = true;
        if (link_ids[i] == b2)
            b2_is_link = true;
    }

    if (include_mobile_base != 0)
    {
        if (body_mobile_base->id() == b1 || body_mobile_base->id() == b2)
        {
            // ignoring all contact with mobile base.
            return;
        }
    }

    if (b1_is_link && b2_is_link)
        // ignoring link-link contact.
        return;

    bool arm_contact = false;
    stringstream ss;
    bool is_b1 = false;
    for (int i=0; i<3; i++)
        if (link_ids[i] == b1 || link_ids[i] == b2)
        {
            arm_contact = true;
            ss <<"link"<<i+1;
            if (link_ids[i] == b1)
                is_b1 = true;
            else
                is_b1 = false;
        }

    dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box                                         
    int numc = dCollide (o1, o2, MAX_CONTACTS, &(contact[0].geom), sizeof(dContact));
    if (numc > 0)
    {
        if (arm_contact == false)
        {
            for (int i=0; i<numc; i++)
            {
                contact[i].surface.mode = dContactSoftCFM | dContactApprox1 | dContactSoftERP;
                contact[i].surface.mu = 0.2;
                contact[i].surface.mu2 = 0.2;
                //contact[i].surface.bounce = 0.01;
                contact[i].surface.soft_cfm = 0.3;
                contact[i].surface.soft_erp = 0.6;

                dJointID c = dJointCreateContact (*world,joints.id(), &contact[i]);
                dJointAttach (c, dGeomGetBody(contact[i].geom.g1),
                        dGeomGetBody(contact[i].geom.g2));
            }
        }
        else
        {
            dVector3 contact_loc = {0, 0, 0};
            skin.link_names.push_back(ss.str());

            for (i=0; i<numc; i++) 
            {
                contact[i].surface.mode = dContactSoftCFM | dContactApprox1 | dContactSoftERP;
                contact[i].surface.mu = 0.4;
                contact[i].surface.mu2 = 0.4;
                //contact[i].surface.bounce = 0.01;
                contact[i].surface.soft_cfm = 0.3;
                contact[i].surface.soft_erp = 0.6;

                contact_loc[0] = contact_loc[0]+contact[i].geom.pos[0];
                contact_loc[1] = contact_loc[1]+contact[i].geom.pos[1];
                contact_loc[2] = contact_loc[2]+contact[i].geom.pos[2];

                pt_x.push_back(contact[i].geom.pos[0]);
                pt_y.push_back(contact[i].geom.pos[1]);
                pt_z.push_back(contact[i].geom.pos[2]);

                dJointID c = dJointCreateContact (*world,joints.id(),&contact[i]);
                dJointAttach (c,b1,b2);

                if (fbnum<MAX_FEEDBACKNUM && (ss.str()=="link1"||ss.str()=="link2"||ss.str()=="link3") )
                {
                    dJointSetFeedback (c,&feedbacks[fbnum++].fb);
                    force_grouping.push_back(force_group);
                    if (is_b1 == true)
                        force_sign.push_back(1);
                    else
                        force_sign.push_back(-1);
                }

            }

            contact_loc[0] = contact_loc[0]/numc;
            contact_loc[1] = contact_loc[1]/numc;
            contact_loc[2] = contact_loc[2]/numc;
            geometry_msgs::Point con_pt;
            con_pt.x = contact_loc[0];
            con_pt.y = contact_loc[1];
            con_pt.z = contact_loc[2];

            skin.locations.push_back(con_pt);
            hrl_msgs::FloatArrayBare pts_x_ar;
            pts_x_ar.data = pt_x;
            skin.pts_x.push_back(pts_x_ar);
            hrl_msgs::FloatArrayBare pts_y_ar;
            pts_y_ar.data = pt_y;
            skin.pts_y.push_back(pts_y_ar);
            hrl_msgs::FloatArrayBare pts_z_ar;
            pts_z_ar.data = pt_z;
            skin.pts_z.push_back(pts_z_ar);
            pt_x.clear();
            pt_y.clear();
            pt_z.clear();
            force_group += 1;
        }
    }
}


void go_initial_position(ros::NodeHandle n)
{
    XmlRpc::XmlRpcValue init_angle;
    while (n.getParam("/m3/software_testbed/joints/init_angle", init_angle) == false)
        sleep(0.1);

    int num_links;
    while (n.getParam("/m3/software_testbed/linkage/num_links", num_links) == false)
        sleep(0.1);

    // moving mobile base to 0, 0, 0
    mobile_base_ep[0] = 0;
    mobile_base_ep[1] = 0;
    mobile_base_ep[2] = 0;

    //implicit assumption that number of links == number of jts
    for (int ii = 0; ii < num_links ; ii++)
    {
        jep[ii] = (double)init_angle[ii];
    }

    float error = 1.;
    float error_thresh = 0.005;
    const dReal timestep = 0.0005;
    int torque_step(0);

    while (error > error_thresh)
    {
        world->step(timestep);
        torque_step++;
        inner_torque_loop();
		hinge1->addTorque(torques[0]);
		hinge2->addTorque(torques[1]);	
		hinge3->addTorque(torques[2]);		
        float e1 = q[0]-jep[0];
        float e2 = q[1]-jep[1];
        float e3 = q[2]-jep[2];
        error = e1*e1 + e2*e2 + e3*e3;
    }
}


void create_robot(ros::NodeHandle n)
{
    //setting intial impedance parameters
    k_p[0] = 30;
    k_p[1] = 20;
    k_p[2] = 15;
    k_d[0] = 15;
    k_d[1] = 10;
    k_d[2] = 8;

    mobile_base_k_p[0] = 5000;
    mobile_base_k_p[1] = 5000;
    mobile_base_k_d[0] = 2500;
    mobile_base_k_d[1] = 2500;

    XmlRpc::XmlRpcValue link_dimensions;
    while (n.getParam("/m3/software_testbed/linkage/dimensions", link_dimensions) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue link_pos;
    while (n.getParam("/m3/software_testbed/linkage/positions", link_pos) == false)
        sleep(0.1);
    int num_links;
    while (n.getParam("/m3/software_testbed/linkage/num_links", num_links) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue link_masses;
    while (n.getParam("/m3/software_testbed/linkage/mass", link_masses) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue jt_min;
    while (n.getParam("/m3/software_testbed/joints/min", jt_min) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue jt_max;
    while (n.getParam("/m3/software_testbed/joints/max", jt_max) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue jt_axes;
    while (n.getParam("/m3/software_testbed/joints/axes", jt_axes) == false)
        sleep(0.1);
    XmlRpc::XmlRpcValue jt_anchor;
    while (n.getParam("/m3/software_testbed/joints/anchor", jt_anchor) == false)
        sleep(0.1);


    double torso_ulim = (double)jt_max[0];
    double torso_llim = (double)jt_min[0];

    double shoulder_ulim = (double)jt_max[1];
    double shoulder_llim = (double)jt_min[1];

    double elbow_ulim = (double)jt_max[2];
    double elbow_llim = (double)jt_min[2];
    //float elbow_llim = 0 * rad_per_deg;

    //double torso_mass = (double)link_masses[0]; // ctorsomass
    //double upper_arm_mass = (double)link_masses[1];// ruparmmass
    //double forearm_mass = (double)link_masses[2];//+0.49; // rlowarmmass + rhandmass

    //link1
    link1 = new dBody(*world);
    dMass mass_l1;
    //dMassSetBoxTotal(&mass_l1, torso_mass, 0.03, torso_half_width*2, 0.03);
    dMassSetBoxTotal(&mass_l1, (double)link_masses[0], (double)link_dimensions[0][0], (double)link_dimensions[0][1], (double)link_dimensions[0][2]);
    link1->setMass(mass_l1);
    //g_link1 = new dBox(*space, 0.03, torso_half_width*2, 0.03);
    g_link1 = new dBox(*space, (double)link_dimensions[0][0], (double)link_dimensions[0][1], (double)link_dimensions[0][2]);
    g_link1->setBody(*link1);
    link1->setPosition((double)link_pos[0][0], (double)link_pos[0][1], (double)link_pos[0][2]); 
    link_ids[0] = link1->id();
    torso_half_width = (double)link_dimensions[0][1];
    torso_half_width_side = (double)link_dimensions[0][0];


    //link2
    link2 = new dBody(*world);
    dMass mass_l2;
    dMassSetBoxTotal(&mass_l2, (double)link_masses[1], (double)link_dimensions[1][0], (double)link_dimensions[1][1], (double)link_dimensions[1][2]);
    link2->setMass(mass_l2);
    g_link2 = new dBox(*space, (double)link_dimensions[1][0], (double)link_dimensions[1][1], (double)link_dimensions[1][2]);
    g_link2->setBody(*link2);
    link2->setPosition((double)link_pos[1][0], (double)link_pos[1][1], (double)link_pos[1][2]); 
    link_ids[1] = link2->id();
    upper_arm_length = (double)link_dimensions[1][1];
    upper_arm_width = (double)link_dimensions[1][0];

    //link3
    link3 = new dBody(*world);
    dMass mass_l3;
    dMassSetBoxTotal(&mass_l3, (double)link_masses[2], (double)link_dimensions[2][0], (double)link_dimensions[2][1], (double)link_dimensions[2][2]);
    link3->setMass(mass_l3);
    g_link3 = new dBox(*space, (double)link_dimensions[2][0], (double)link_dimensions[2][1], (double)link_dimensions[2][2]);
    g_link3->setBody(*link3);
    link3->setPosition((double)link_pos[2][0], (double)link_pos[2][1], (double)link_pos[2][2]); 
    link_ids[2] = link3->id();
    forearm_length = (double)link_dimensions[2][1];//0.288; //+ 0.115; // rlowarmz + rhandz
    forearm_width = (double)link_dimensions[2][0];


    if (include_mobile_base != 0)
    {
        dMass mass_base;
        double base_x = 0.2;
        double base_y = 0.2;
        double base_z = 0.05;
        dMassSetBoxTotal(&mass_base, 10., base_x, base_y, base_z);

        body_mobile_base = new dBody(*world);
        body_mobile_base->setMass(mass_l3);

        geom_mobile_base = new dBox(*space, base_x, base_y, base_z);
        geom_mobile_base->setBody(*body_mobile_base);
    }

    //------- now define the joints ----------
    if (include_mobile_base != 0)
    {
        mobile_base_plane2d_jt_id = dJointCreatePlane2D(world->id(), 0);
        // attach to mobile base body and the world.
        dJointAttach(mobile_base_plane2d_jt_id, body_mobile_base->id(), 0);

        dJointSetPlane2DXParam(mobile_base_plane2d_jt_id, dParamFMax, 0);
        dJointSetPlane2DYParam(mobile_base_plane2d_jt_id, dParamFMax, 0);

        dJointSetPlane2DXParam(mobile_base_plane2d_jt_id, dParamVel, 0.0);
        dJointSetPlane2DYParam(mobile_base_plane2d_jt_id, dParamVel, 0.0);

        dJointSetPlane2DAngleParam(mobile_base_plane2d_jt_id, dParamFMax, 100);
        dJointSetPlane2DAngleParam(mobile_base_plane2d_jt_id, dParamVel, 0.0);

        hinge1 = new dHingeJoint(*world);
        hinge1->attach(*link1, *body_mobile_base);
    }
    else
    {
        hinge1 = new dHingeJoint(*world);
        hinge1->attach(*link1, 0);
    }

    hinge1->setAnchor((double)jt_anchor[0][0], (double)jt_anchor[0][1], (double)jt_anchor[0][2]);
    hinge1->setAxis((double)jt_axes[0][0], (double)jt_axes[0][1], (double)jt_axes[0][2]);
    dJointSetHingeParam(hinge1->id(),dParamLoStop, torso_llim);
    dJointSetHingeParam(hinge1->id(),dParamHiStop, torso_ulim);

    hinge2 = new dHingeJoint(*world);
    hinge2->attach(*link2, *link1);
    hinge2->setAnchor((double)jt_anchor[1][0], (double)jt_anchor[1][1], (double)jt_anchor[1][2]);
    hinge2->setAxis((double)jt_axes[1][0], (double)jt_axes[1][1], (double)jt_axes[1][2]);
    dJointSetHingeParam(hinge2->id(),dParamLoStop, shoulder_llim);
    dJointSetHingeParam(hinge2->id(),dParamHiStop, shoulder_ulim);

    hinge3 = new dHingeJoint(*world);
    hinge3->attach(*link3, *link2);
    hinge3->setAnchor((double)jt_anchor[2][0], (double)jt_anchor[2][1], (double)jt_anchor[2][2]);
    hinge3->setAxis((double)jt_axes[2][0], (double)jt_axes[2][1], (double)jt_axes[2][2]);
    dJointSetHingeParam(hinge3->id(),dParamLoStop, elbow_llim);
    dJointSetHingeParam(hinge3->id(),dParamHiStop, elbow_ulim);


    // this is for contact joints (Advait)
    joints.create();
}

void create_movable_obstacles(ros::NodeHandle n)
{
    float obstacle_mass = 1.;

    // this needs to be the first param that is read for stupid
    // synchronization with obstacles.py
    XmlRpc::XmlRpcValue cylinders_dim;
    while (n.getParam("/m3/software_testbed/movable_dimen", cylinders_dim) == false)
        sleep(0.1);

    XmlRpc::XmlRpcValue cylinders_pos;
    n.getParam("/m3/software_testbed/movable_position", cylinders_pos);

    n.getParam("/m3/software_testbed/num_movable", num_used_movable);

    XmlRpc::XmlRpcValue cylinders_max_force;     
    bool got_max_force;
    got_max_force = n.getParam("/m3/software_testbed/movable_max_force", cylinders_max_force);

    for (int i = 0; i < num_used_movable; i++)
    {
        dMass m_obst;
        dCapsule *geom_cyl;

        dMassSetCapsuleTotal(&m_obst, obstacle_mass, 3,
                (double)cylinders_dim[i][0], (double)cylinders_dim[i][2]);
        obstacles[i].create(*world);
        obstacles[i].setPosition((double)cylinders_pos[i][0], (double)cylinders_pos[i][1], (double)cylinders_pos[i][2]);
        obstacles[i].setMass(&m_obst);
        plane2d_joint_ids[i] = dJointCreatePlane2D(world->id(), 0);
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

        geom_cyl = new dCapsule(*space, (double)cylinders_dim[i][0], (double)cylinders_dim[i][2]);
        geom_cyl->setBody(obstacles[i]);
        dJointSetFeedback(plane2d_joint_ids[i], &frict_feedbacks[i].fb);
    }
}


void create_compliant_obstacles(ros::NodeHandle n)
{
    float obstacle_mass = 1.;

    // this needs to be the first param that is read for stupid
    // synchronization with obstacles.py
    XmlRpc::XmlRpcValue cylinders_dim;
    while (n.getParam("/m3/software_testbed/compliant_dimen", cylinders_dim) == false)
        sleep(0.1);

    XmlRpc::XmlRpcValue cylinders_pos;
    n.getParam("/m3/software_testbed/compliant_position", cylinders_pos);

    XmlRpc::XmlRpcValue cylinders_stiffness;
    bool got_stiffness;
    got_stiffness = n.getParam("/m3/software_testbed/compliant_stiffness_value", cylinders_stiffness);

    n.getParam("/m3/software_testbed/num_compliant", num_used_compliant);

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

        dMassSetCapsuleTotal(&m_obst, obstacle_mass, 3,
                (double)cylinders_dim[i][0], (double)cylinders_dim[i][2]);
        compliant_obstacles[i].create(*world);
        compliant_obstacles[i].setPosition((double)cylinders_pos[i][0], (double)cylinders_pos[i][1], (double)cylinders_pos[i][2]);
        obst_home[i][0] = (double)cylinders_pos[i][0];
        obst_home[i][1] = (double)cylinders_pos[i][1];
        obst_home[i][2] = (double)cylinders_pos[i][2];
        compliant_obstacles[i].setMass(&m_obst);
        compliant_plane2d_joint_ids[i] = dJointCreatePlane2D(world->id(), 0);
        dJointAttach(compliant_plane2d_joint_ids[i], compliant_obstacles[i].id(), 0);
        // if (got_max_force == false)
        //   {
        //     dJointSetPlane2DXParam(compliant_plane2d_joint_ids[i], dParamFMax, max_friction*0.707);
        //     dJointSetPlane2DYParam(compliant_plane2d_joint_ids[i], dParamFMax, max_friction*0.707);
        //     dJointSetPlane2DAngleParam(compliant_plane2d_joint_ids[i], dParamFMax, max_tor_friction);
        //   }
        // else
        //   {
        //     dJointSetPlane2DXParam(compliant_plane2d_joint_ids[i], dParamFMax, (double)cylinders_max_force[i]*0.707);
        //     dJointSetPlane2DYParam(compliant_plane2d_joint_ids[i], dParamFMax, (double)cylinders_max_force[i]*0.707);
        //     dJointSetPlane2DAngleParam(compliant_plane2d_joint_ids[i], dParamFMax, max_tor_friction);
        //   }

        dJointSetPlane2DXParam(compliant_plane2d_joint_ids[i], dParamVel, 0.0);
        dJointSetPlane2DYParam(compliant_plane2d_joint_ids[i], dParamVel, 0.0);
        dJointSetPlane2DAngleParam(compliant_plane2d_joint_ids[i], dParamVel, 0.0);

        geom_cyl = new dCapsule(*space, (double)cylinders_dim[i][0], (double)cylinders_dim[i][2]);
        geom_cyl->setBody(compliant_obstacles[i]);
        //dJointSetFeedback(compliant_plane2d_joint_ids[i], &compliant_frict_feedbacks[i].fb);
    }
}


void create_fixed_obstacles(ros::NodeHandle n)
{
    float obstacle_mass = 1.;

    while(n.getParam("/m3/software_testbed/num_fixed", num_used_fixed) == false)
        sleep(0.1);

    XmlRpc::XmlRpcValue fixed_pos;
    n.getParam("/m3/software_testbed/fixed_position", fixed_pos);
    XmlRpc::XmlRpcValue fixed_dim;
    n.getParam("/m3/software_testbed/fixed_dimen", fixed_dim);

    for (int i = 0; i < num_used_fixed; i++)
    {
        dMass m_obst;
        dCapsule *geom_fixed;
        dMassSetCapsuleTotal(&m_obst, obstacle_mass, 3,
                (double)fixed_dim[i][0], (double)fixed_dim[i][2]);

        fixed_obstacles[i].create(*world);
        fixed_obstacles[i].setPosition((double)fixed_pos[i][0], (double)fixed_pos[i][1], (double)fixed_pos[i][2]);
        fixed_obstacles[i].setMass(&m_obst);
        fixed_joint_ids[i] = dJointCreateFixed(world->id(), 0);
        dJointAttach(fixed_joint_ids[i], fixed_obstacles[i].id(), 0);
        dJointSetFixed(fixed_joint_ids[i]);
        geom_fixed = new dCapsule(*space, (double)fixed_dim[i][0], (double)fixed_dim[i][2]);
        geom_fixed->setBody(fixed_obstacles[i]);
    }
}

double get_wall_clock_time()
{
    timeval tim;
    gettimeofday(&tim, NULL);
    double t1=tim.tv_sec+(tim.tv_usec/1000000.0);
    return t1;
}



void taxel_simulation_code(double resolution)
{
    /**********************************************************************************/
    /*  THE TAXEL CODE BELOW IS A MESS, IT WORKS ONLY FOR 2D ALONG A LINE AND NEEDS TO BE
        CLEANED UP AND CONDENSED INTO A SINGLE FUNCTION CALL FOR EACH LINK *******************/
    // float torso_half_width = 0.196; // ctorsoy
    /*************************************************************************************/
    taxel.header.frame_id = "/world";
    taxel.header.stamp = ros::Time::now();

    std::vector < string > taxel_links;

    //////////////////////////////////////link1 taxels//////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////


    int k = 2;

    assert(torso_half_width>0);
    assert(torso_half_width_side>0);

    int num = floor(resolution*torso_half_width);
    float step =  1.0/resolution;
    float link1_width = torso_half_width_side/2.0;

    dVector3 global_pt0;
    dBodyGetRelPointPos(link1->id(), link1_width, 0.0,  0.0, global_pt0);
    taxel.centers_x.push_back(global_pt0[0]);
    taxel.centers_y.push_back(global_pt0[1]);
    taxel.centers_z.push_back(global_pt0[2]);
    taxel.link_names.push_back("link1");

    taxel_links.push_back("link1");

    dBodyGetRelPointPos(link1->id(), -link1_width, 0.0,  0.0, global_pt0);
    taxel.centers_x.push_back(global_pt0[0]);
    taxel.centers_y.push_back(global_pt0[1]);
    taxel.centers_z.push_back(global_pt0[2]);
    taxel.link_names.push_back("link1");

    taxel_links.push_back("link1");

    dVector3 global_n0;
    dBodyVectorToWorld (link1->id(), 1.0, 0.0, 0.0, global_n0);
    taxel.normals_x.push_back(global_n0[0]);
    taxel.normals_y.push_back(global_n0[1]);
    taxel.normals_z.push_back(global_n0[2]);

    dBodyVectorToWorld (link1->id(), -1.0, 0.0, 0.0, global_n0);
    taxel.normals_x.push_back(global_n0[0]);
    taxel.normals_y.push_back(global_n0[1]);
    taxel.normals_z.push_back(global_n0[2]);

    while ( k < num )
    {
        dVector3 global_pt;
        dVector3 global_pt2;

        if (num >1 )
        {
            dBodyGetRelPointPos(link1->id(), link1_width, (k/2)*step,  0.0, global_pt);
            taxel.centers_x.push_back(global_pt[0]);
            taxel.centers_y.push_back(global_pt[1]);
            taxel.centers_z.push_back(global_pt[2]);
            taxel.link_names.push_back("link1");

            taxel_links.push_back("link1");

            dBodyGetRelPointPos(link1->id(), -link1_width, (k/2)*step,  0.0, global_pt2);
            taxel.centers_x.push_back(global_pt2[0]);
            taxel.centers_y.push_back(global_pt2[1]);
            taxel.centers_z.push_back(global_pt2[2]);
            taxel.link_names.push_back("link1");

            taxel_links.push_back("link1");

            dBodyGetRelPointPos(link1->id(), link1_width, -(k/2)*step,  0.0, global_pt);
            taxel.centers_x.push_back(global_pt[0]);
            taxel.centers_y.push_back(global_pt[1]);
            taxel.centers_z.push_back(global_pt[2]);
            taxel.link_names.push_back("link1");

            taxel_links.push_back("link1");

            dBodyGetRelPointPos(link1->id(), -link1_width, -(k/2)*step,  0.0, global_pt2);
            taxel.centers_x.push_back(global_pt2[0]);
            taxel.centers_y.push_back(global_pt2[1]);
            taxel.centers_z.push_back(global_pt2[2]);
            taxel.link_names.push_back("link1");

            taxel_links.push_back("link1");

            dVector3 global_n;
            dVector3 global_n2;
            dBodyVectorToWorld (link1->id(), 1.0, 0.0, 0.0, global_n);
            taxel.normals_x.push_back(global_n[0]);
            taxel.normals_y.push_back(global_n[1]);
            taxel.normals_z.push_back(global_n[2]);

            dBodyVectorToWorld (link1->id(), -1.0, 0.0, 0.0, global_n2);
            taxel.normals_x.push_back(global_n2[0]);
            taxel.normals_y.push_back(global_n2[1]);
            taxel.normals_z.push_back(global_n2[2]);

            dBodyVectorToWorld (link1->id(), 1.0, 0.0, 0.0, global_n);
            taxel.normals_x.push_back(global_n[0]);
            taxel.normals_y.push_back(global_n[1]);
            taxel.normals_z.push_back(global_n[2]);

            dBodyVectorToWorld (link1->id(), -1.0, 0.0, 0.0, global_n2);
            taxel.normals_x.push_back(global_n2[0]);
            taxel.normals_y.push_back(global_n2[1]);
            taxel.normals_z.push_back(global_n2[2]);
        }
        k = k+2;
    }

    k = 2;
    num = floor(resolution*(torso_half_width_side+0.0001));

    dBodyGetRelPointPos(link1->id(), 0.0, -torso_half_width/2.0,  0.0, global_pt0);
    taxel.centers_x.push_back(global_pt0[0]);
    taxel.centers_y.push_back(global_pt0[1]);
    taxel.centers_z.push_back(global_pt0[2]);
    taxel.link_names.push_back("link1");

    taxel_links.push_back("link1");

    dBodyVectorToWorld (link1->id(), 0.0, -1.0, 0.0, global_n0);
    taxel.normals_x.push_back(global_n0[0]);
    taxel.normals_y.push_back(global_n0[1]);
    taxel.normals_z.push_back(global_n0[2]);

    ///////////////////////////////////////////////////test for taxel////////////
    dBodyGetRelPointPos(link1->id(), 0.0, torso_half_width/2.0,  0.0, global_pt0);
    taxel.centers_x.push_back(global_pt0[0]);
    taxel.centers_y.push_back(global_pt0[1]);
    taxel.centers_z.push_back(global_pt0[2]);
    taxel.link_names.push_back("link1");

    taxel_links.push_back("link1");

    dBodyVectorToWorld (link1->id(), 0.0, 1.0, 0.0, global_n0);
    taxel.normals_x.push_back(global_n0[0]);
    taxel.normals_y.push_back(global_n0[1]);
    taxel.normals_z.push_back(global_n0[2]);
    ///////////////////////////////////////////////////////////////////////////////


    while ( k < num )
    {
        dVector3 global_pt;
        dVector3 global_pt2;
        dVector3 global_n;
        dVector3 global_n2;

        if (num >  1.0)
        {
            dBodyGetRelPointPos(link1->id(), (k/2)*step, -torso_half_width/2.0,  0.0, global_pt);		
            taxel.centers_x.push_back(global_pt[0]);
            taxel.centers_y.push_back(global_pt[1]);
            taxel.centers_z.push_back(global_pt[2]);
            taxel.link_names.push_back("link1");

            taxel_links.push_back("link1");

            dBodyGetRelPointPos(link1->id(), -(k/2)*step, -torso_half_width/2.0,  0.0, global_pt2);		
            taxel.centers_x.push_back(global_pt2[0]);
            taxel.centers_y.push_back(global_pt2[1]);
            taxel.centers_z.push_back(global_pt2[2]);
            taxel.link_names.push_back("link1");

            taxel_links.push_back("link1");

            dBodyVectorToWorld (link1->id(), 0.0, -1.0, 0.0, global_n);
            taxel.normals_x.push_back(global_n[0]);
            taxel.normals_y.push_back(global_n[1]);
            taxel.normals_z.push_back(global_n[2]);

            dBodyVectorToWorld (link1->id(), 0.0, -1.0, 0.0, global_n2);
            taxel.normals_x.push_back(global_n2[0]);
            taxel.normals_y.push_back(global_n2[1]);
            taxel.normals_z.push_back(global_n2[2]);

            dBodyGetRelPointPos(link1->id(), (k/2)*step, torso_half_width/2.0,  0.0, global_pt);		
            taxel.centers_x.push_back(global_pt[0]);
            taxel.centers_y.push_back(global_pt[1]);
            taxel.centers_z.push_back(global_pt[2]);
            taxel.link_names.push_back("link1");

            taxel_links.push_back("link1");

            dBodyGetRelPointPos(link1->id(), -(k/2)*step, torso_half_width/2.0,  0.0, global_pt2);		
            taxel.centers_x.push_back(global_pt2[0]);
            taxel.centers_y.push_back(global_pt2[1]);
            taxel.centers_z.push_back(global_pt2[2]);
            taxel.link_names.push_back("link1");

            taxel_links.push_back("link1");

            dBodyVectorToWorld (link1->id(), 0.0, 1.0, 0.0, global_n);
            taxel.normals_x.push_back(global_n[0]);
            taxel.normals_y.push_back(global_n[1]);
            taxel.normals_z.push_back(global_n[2]);

            dBodyVectorToWorld (link1->id(), 0.0, 1.0, 0.0, global_n2);
            taxel.normals_x.push_back(global_n2[0]);
            taxel.normals_y.push_back(global_n2[1]);
            taxel.normals_z.push_back(global_n2[2]);

        }

        k = k+2;
    }

    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////



    //////////////////////////////////////link2 taxels//////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////


    // float upper_arm_length = 0.334; // ruparmz
    assert(upper_arm_length>0);
    assert(upper_arm_width>0);
    float link2_width = upper_arm_width/2.0;
    k = 2;
    num = floor(resolution*upper_arm_length);
    dBodyGetRelPointPos(link2->id(), link2_width, 0.0,  0.0, global_pt0);
    taxel.centers_x.push_back(global_pt0[0]);
    taxel.centers_y.push_back(global_pt0[1]);
    taxel.centers_z.push_back(global_pt0[2]);
    taxel.link_names.push_back("link2");

    taxel_links.push_back("link2");

    dBodyGetRelPointPos(link2->id(), -link2_width, 0.0,  0.0, global_pt0);
    taxel.centers_x.push_back(global_pt0[0]);
    taxel.centers_y.push_back(global_pt0[1]);
    taxel.centers_z.push_back(global_pt0[2]);
    taxel.link_names.push_back("link2");

    taxel_links.push_back("link2");

    dBodyVectorToWorld (link2->id(), 1.0, 0.0, 0.0, global_n0);
    taxel.normals_x.push_back(global_n0[0]);
    taxel.normals_y.push_back(global_n0[1]);
    taxel.normals_z.push_back(global_n0[2]);

    dBodyVectorToWorld (link2->id(), -1.0, 0.0, 0.0, global_n0);
    taxel.normals_x.push_back(global_n0[0]);
    taxel.normals_y.push_back(global_n0[1]);
    taxel.normals_z.push_back(global_n0[2]);

    while ( k < num )
    {
        dVector3 global_pt;
        dVector3 global_pt2;

        if ( num > 1)
        {
            dBodyGetRelPointPos(link2->id(), link2_width, (k/2)*step,  0.0, global_pt);
            taxel.centers_x.push_back(global_pt[0]);
            taxel.centers_y.push_back(global_pt[1]);
            taxel.centers_z.push_back(global_pt[2]);
            taxel.link_names.push_back("link2");

            taxel_links.push_back("link2");

            dBodyGetRelPointPos(link2->id(), -link2_width, (k/2)*step,  0.0, global_pt2);
            taxel.centers_x.push_back(global_pt2[0]);
            taxel.centers_y.push_back(global_pt2[1]);
            taxel.centers_z.push_back(global_pt2[2]);
            taxel.link_names.push_back("link2");

            taxel_links.push_back("link2");

            dBodyGetRelPointPos(link2->id(), link2_width, -(k/2)*step,  0.0, global_pt);
            taxel.centers_x.push_back(global_pt[0]);
            taxel.centers_y.push_back(global_pt[1]);
            taxel.centers_z.push_back(global_pt[2]);
            taxel.link_names.push_back("link2");

            taxel_links.push_back("link2");

            dBodyGetRelPointPos(link2->id(), -link2_width, -(k/2)*step,  0.0, global_pt2);
            taxel.centers_x.push_back(global_pt2[0]);
            taxel.centers_y.push_back(global_pt2[1]);
            taxel.centers_z.push_back(global_pt2[2]);
            taxel.link_names.push_back("link2");

            taxel_links.push_back("link2");

            dVector3 global_n;
            dVector3 global_n2;
            dBodyVectorToWorld (link2->id(), 1.0, 0.0, 0.0, global_n);
            taxel.normals_x.push_back(global_n[0]);
            taxel.normals_y.push_back(global_n[1]);
            taxel.normals_z.push_back(global_n[2]);

            dBodyVectorToWorld (link2->id(), -1.0, 0.0, 0.0, global_n2);
            taxel.normals_x.push_back(global_n2[0]);
            taxel.normals_y.push_back(global_n2[1]);
            taxel.normals_z.push_back(global_n2[2]);

            dBodyVectorToWorld (link2->id(), 1.0, 0.0, 0.0, global_n);
            taxel.normals_x.push_back(global_n[0]);
            taxel.normals_y.push_back(global_n[1]);
            taxel.normals_z.push_back(global_n[2]);

            dBodyVectorToWorld (link2->id(), -1.0, 0.0, 0.0, global_n2);
            taxel.normals_x.push_back(global_n2[0]);
            taxel.normals_y.push_back(global_n2[1]);
            taxel.normals_z.push_back(global_n2[2]);
        }
        k = k+2;
    }

    ////////////////taxels for front and back of link//////////////////////

    k = 2;
    num = floor(resolution*(upper_arm_width+0.0001));

    dBodyGetRelPointPos(link2->id(), 0.0, -upper_arm_length/2.0,  0.0, global_pt0);
    taxel.centers_x.push_back(global_pt0[0]);
    taxel.centers_y.push_back(global_pt0[1]);
    taxel.centers_z.push_back(global_pt0[2]);
    taxel.link_names.push_back("link2");

    taxel_links.push_back("link2");

    dBodyVectorToWorld (link2->id(), 0.0, -1.0, 0.0, global_n0);
    taxel.normals_x.push_back(global_n0[0]);
    taxel.normals_y.push_back(global_n0[1]);
    taxel.normals_z.push_back(global_n0[2]);

    ///////////////////////////////////////////////////test for taxel////////////
    dBodyGetRelPointPos(link2->id(), 0.0, upper_arm_length/2.0,  0.0, global_pt0);
    taxel.centers_x.push_back(global_pt0[0]);
    taxel.centers_y.push_back(global_pt0[1]);
    taxel.centers_z.push_back(global_pt0[2]);
    taxel.link_names.push_back("link2");

    taxel_links.push_back("link2");

    dBodyVectorToWorld (link2->id(), 0.0, 1.0, 0.0, global_n0);
    taxel.normals_x.push_back(global_n0[0]);
    taxel.normals_y.push_back(global_n0[1]);
    taxel.normals_z.push_back(global_n0[2]);
    ///////////////////////////////////////////////////////////////////////////////


    while ( k < num )
    {
        dVector3 global_pt;
        dVector3 global_pt2;
        dVector3 global_n;
        dVector3 global_n2;

        if (num >  1.0)
        {
            dBodyGetRelPointPos(link2->id(), (k/2)*step, -upper_arm_length/2.0,  0.0, global_pt);		
            taxel.centers_x.push_back(global_pt[0]);
            taxel.centers_y.push_back(global_pt[1]);
            taxel.centers_z.push_back(global_pt[2]);
            taxel.link_names.push_back("link2");

            taxel_links.push_back("link2");

            dBodyGetRelPointPos(link2->id(), -(k/2)*step, -upper_arm_length/2.0,  0.0, global_pt2);		
            taxel.centers_x.push_back(global_pt2[0]);
            taxel.centers_y.push_back(global_pt2[1]);
            taxel.centers_z.push_back(global_pt2[2]);
            taxel.link_names.push_back("link2");

            taxel_links.push_back("link2");

            dBodyVectorToWorld (link2->id(), 0.0, -1.0, 0.0, global_n);
            taxel.normals_x.push_back(global_n[0]);
            taxel.normals_y.push_back(global_n[1]);
            taxel.normals_z.push_back(global_n[2]);

            dBodyVectorToWorld (link2->id(), 0.0, -1.0, 0.0, global_n2);
            taxel.normals_x.push_back(global_n2[0]);
            taxel.normals_y.push_back(global_n2[1]);
            taxel.normals_z.push_back(global_n2[2]);

            dBodyGetRelPointPos(link2->id(), (k/2)*step, upper_arm_length/2.0,  0.0, global_pt);		
            taxel.centers_x.push_back(global_pt[0]);
            taxel.centers_y.push_back(global_pt[1]);
            taxel.centers_z.push_back(global_pt[2]);
            taxel.link_names.push_back("link2");

            taxel_links.push_back("link2");

            dBodyGetRelPointPos(link2->id(), -(k/2)*step, upper_arm_length/2.0,  0.0, global_pt2);		
            taxel.centers_x.push_back(global_pt2[0]);
            taxel.centers_y.push_back(global_pt2[1]);
            taxel.centers_z.push_back(global_pt2[2]);
            taxel.link_names.push_back("link2");

            taxel_links.push_back("link2");

            dBodyVectorToWorld (link2->id(), 0.0, 1.0, 0.0, global_n);
            taxel.normals_x.push_back(global_n[0]);
            taxel.normals_y.push_back(global_n[1]);
            taxel.normals_z.push_back(global_n[2]);

            dBodyVectorToWorld (link2->id(), 0.0, 1.0, 0.0, global_n2);
            taxel.normals_x.push_back(global_n2[0]);
            taxel.normals_y.push_back(global_n2[1]);
            taxel.normals_z.push_back(global_n2[2]);

        }

        k = k+2;
    }


    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////




    //////////////////////////////////////link3 taxels//////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////


    // float forearm_length = 0.288; //+ 0.115; // rlowarmz + rhandz
    k = 2;
    assert(forearm_length>0);
    assert(forearm_width>0);
    float link3_width = forearm_width/2.0;
    num = floor(resolution*forearm_length);

    dBodyGetRelPointPos(link3->id(), link3_width, 0.0,  0.0, global_pt0);
    taxel.centers_x.push_back(global_pt0[0]);
    taxel.centers_y.push_back(global_pt0[1]);
    taxel.centers_z.push_back(global_pt0[2]);
    taxel.link_names.push_back("link3");

    taxel_links.push_back("link3");

    dBodyGetRelPointPos(link3->id(), -link3_width, 0.0,  0.0, global_pt0);
    taxel.centers_x.push_back(global_pt0[0]);
    taxel.centers_y.push_back(global_pt0[1]);
    taxel.centers_z.push_back(global_pt0[2]);
    taxel.link_names.push_back("link3");

    taxel_links.push_back("link3");

    dBodyVectorToWorld (link3->id(), 1.0, 0.0, 0.0, global_n0);
    taxel.normals_x.push_back(global_n0[0]);
    taxel.normals_y.push_back(global_n0[1]);
    taxel.normals_z.push_back(global_n0[2]);

    dBodyVectorToWorld (link3->id(), -1.0, 0.0, 0.0, global_n0);
    taxel.normals_x.push_back(global_n0[0]);
    taxel.normals_y.push_back(global_n0[1]);
    taxel.normals_z.push_back(global_n0[2]);

    while ( k < num )
    {
        dVector3 global_pt;
        dVector3 global_pt2;

        if (num > 1)
        {
            dBodyGetRelPointPos(link3->id(), link3_width, (k/2)*step,  0.0, global_pt);
            taxel.centers_x.push_back(global_pt[0]);
            taxel.centers_y.push_back(global_pt[1]);
            taxel.centers_z.push_back(global_pt[2]);
            taxel.link_names.push_back("link3");

            taxel_links.push_back("link3");

            dBodyGetRelPointPos(link3->id(), -link3_width, (k/2)*step,  0.0, global_pt2);
            taxel.centers_x.push_back(global_pt2[0]);
            taxel.centers_y.push_back(global_pt2[1]);
            taxel.centers_z.push_back(global_pt2[2]);
            taxel.link_names.push_back("link3");

            taxel_links.push_back("link3");

            dBodyGetRelPointPos(link3->id(), link3_width, -(k/2)*step,  0.0, global_pt);
            taxel.centers_x.push_back(global_pt[0]);
            taxel.centers_y.push_back(global_pt[1]);
            taxel.centers_z.push_back(global_pt[2]);
            taxel.link_names.push_back("link3");

            taxel_links.push_back("link3");

            dBodyGetRelPointPos(link3->id(), -link3_width, -(k/2)*step,  0.0, global_pt2);
            taxel.centers_x.push_back(global_pt2[0]);
            taxel.centers_y.push_back(global_pt2[1]);
            taxel.centers_z.push_back(global_pt2[2]);
            taxel.link_names.push_back("link3");

            taxel_links.push_back("link3");

            dVector3 global_n;
            dVector3 global_n2;
            dBodyVectorToWorld (link3->id(), 1.0, 0.0, 0.0, global_n);
            taxel.normals_x.push_back(global_n[0]);
            taxel.normals_y.push_back(global_n[1]);
            taxel.normals_z.push_back(global_n[2]);

            dBodyVectorToWorld (link3->id(), -1.0, 0.0, 0.0, global_n2);
            taxel.normals_x.push_back(global_n2[0]);
            taxel.normals_y.push_back(global_n2[1]);
            taxel.normals_z.push_back(global_n2[2]);

            dBodyVectorToWorld (link3->id(), 1.0, 0.0, 0.0, global_n);
            taxel.normals_x.push_back(global_n[0]);
            taxel.normals_y.push_back(global_n[1]);
            taxel.normals_z.push_back(global_n[2]);

            dBodyVectorToWorld (link3->id(), -1.0, 0.0, 0.0, global_n2);
            taxel.normals_x.push_back(global_n2[0]);
            taxel.normals_y.push_back(global_n2[1]);
            taxel.normals_z.push_back(global_n2[2]);
        }
        k = k+2;
    }

    // float forearm_length = 0.288; //+ 0.115; // rlowarmz + rhandz
    k = 2;
    num = floor(resolution*(forearm_width+0.0001));

    dBodyGetRelPointPos(link3->id(), 0.0, -forearm_length/2.0,  0.0, global_pt0);
    taxel.centers_x.push_back(global_pt0[0]);
    taxel.centers_y.push_back(global_pt0[1]);
    taxel.centers_z.push_back(global_pt0[2]);
    taxel.link_names.push_back("link3");

    taxel_links.push_back("link3");

    dBodyVectorToWorld (link3->id(), 0.0, -1.0, 0.0, global_n0);
    taxel.normals_x.push_back(global_n0[0]);
    taxel.normals_y.push_back(global_n0[1]);
    taxel.normals_z.push_back(global_n0[2]);

    ///////////////////////////////////////////////////test for taxel////////////
    dBodyGetRelPointPos(link3->id(), 0.0, forearm_length/2.0,  0.0, global_pt0);
    taxel.centers_x.push_back(global_pt0[0]);
    taxel.centers_y.push_back(global_pt0[1]);
    taxel.centers_z.push_back(global_pt0[2]);
    taxel.link_names.push_back("link3");

    taxel_links.push_back("link3");

    dBodyVectorToWorld (link3->id(), 0.0, 1.0, 0.0, global_n0);
    taxel.normals_x.push_back(global_n0[0]);
    taxel.normals_y.push_back(global_n0[1]);
    taxel.normals_z.push_back(global_n0[2]);
    ///////////////////////////////////////////////////////////////////////////////


    while ( k < num )
    {
        dVector3 global_pt;
        dVector3 global_pt2;
        dVector3 global_n;
        dVector3 global_n2;

        if (num >  1.0)
        {
            dBodyGetRelPointPos(link3->id(), (k/2)*step, -forearm_length/2.0,  0.0, global_pt);		
            taxel.centers_x.push_back(global_pt[0]);
            taxel.centers_y.push_back(global_pt[1]);
            taxel.centers_z.push_back(global_pt[2]);
            taxel.link_names.push_back("link3");

            taxel_links.push_back("link3");

            dBodyGetRelPointPos(link3->id(), -(k/2)*step, -forearm_length/2.0,  0.0, global_pt2);		
            taxel.centers_x.push_back(global_pt2[0]);
            taxel.centers_y.push_back(global_pt2[1]);
            taxel.centers_z.push_back(global_pt2[2]);
            taxel.link_names.push_back("link3");

            taxel_links.push_back("link3");

            dBodyVectorToWorld (link3->id(), 0.0, -1.0, 0.0, global_n);
            taxel.normals_x.push_back(global_n[0]);
            taxel.normals_y.push_back(global_n[1]);
            taxel.normals_z.push_back(global_n[2]);

            dBodyVectorToWorld (link3->id(), 0.0, -1.0, 0.0, global_n2);
            taxel.normals_x.push_back(global_n2[0]);
            taxel.normals_y.push_back(global_n2[1]);
            taxel.normals_z.push_back(global_n2[2]);

            dBodyGetRelPointPos(link3->id(), (k/2)*step, forearm_length/2.0,  0.0, global_pt);		
            taxel.centers_x.push_back(global_pt[0]);
            taxel.centers_y.push_back(global_pt[1]);
            taxel.centers_z.push_back(global_pt[2]);
            taxel.link_names.push_back("link3");

            taxel_links.push_back("link3");

            dBodyGetRelPointPos(link3->id(), -(k/2)*step, forearm_length/2.0,  0.0, global_pt2);		
            taxel.centers_x.push_back(global_pt2[0]);
            taxel.centers_y.push_back(global_pt2[1]);
            taxel.centers_z.push_back(global_pt2[2]);
            taxel.link_names.push_back("link3");

            taxel_links.push_back("link3");

            dBodyVectorToWorld (link3->id(), 0.0, 1.0, 0.0, global_n);
            taxel.normals_x.push_back(global_n[0]);
            taxel.normals_y.push_back(global_n[1]);
            taxel.normals_z.push_back(global_n[2]);

            dBodyVectorToWorld (link3->id(), 0.0, 1.0, 0.0, global_n2);
            taxel.normals_x.push_back(global_n2[0]);
            taxel.normals_y.push_back(global_n2[1]);
            taxel.normals_z.push_back(global_n2[2]);

        }

        k = k+2;
    }

    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////


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

        for (unsigned int k = 0; k <  taxel.centers_x.size(); k++)
        {
            float distance = sqrt(pow((taxel.centers_x[k]-skin.locations[j].x),2)+pow((taxel.centers_y[k]-skin.locations[j].y),2)+pow((taxel.centers_z[k]-skin.locations[j].z),2));
            if (distance < min_distance && taxel_links[k] == skin.link_names[j])
            {
                dVector3 body_pt;
                dVector3 body_taxel_pt;
                float length(0);
                float width(0);

                if (skin.link_names[j] == "link1")
                {
                    dBodyGetPosRelPoint (link1->id(), skin.locations[j].x, skin.locations[j].y, skin.locations[j].z, body_pt);
                    dBodyGetPosRelPoint (link1->id(), taxel.centers_x[k], taxel.centers_y[k], taxel.centers_z[k], body_taxel_pt);
                    length = torso_half_width/2.0;
                    width = torso_half_width_side/2.0;
                }
                else if (skin.link_names[j] == "link2")
                {
                    dBodyGetPosRelPoint (link2->id(), skin.locations[j].x, skin.locations[j].y, skin.locations[j].z, body_pt);
                    dBodyGetPosRelPoint (link2->id(), taxel.centers_x[k], taxel.centers_y[k], taxel.centers_z[k], body_taxel_pt);
                    length = upper_arm_length/2.0;
                    width = upper_arm_width/2.0;
                }
                else if (skin.link_names[j] == "link3")
                {
                    dBodyGetPosRelPoint (link3->id(), skin.locations[j].x, skin.locations[j].y, skin.locations[j].z, body_pt);
                    dBodyGetPosRelPoint (link3->id(), taxel.centers_x[k], taxel.centers_y[k], taxel.centers_z[k], body_taxel_pt);
                    length = forearm_length/2.0;
                    width = forearm_width/2.0;
                }

                double mag_norm = sqrt(taxel.normals_x[k]*taxel.normals_x[k]+taxel.normals_y[k]*taxel.normals_y[k]);
                double mag_force = sqrt(skin.forces[j].x*skin.forces[j].x + skin.forces[j].y*skin.forces[j].y);

                double dot_prod = (taxel.normals_x[k]*skin.forces[j].x + taxel.normals_y[k]*skin.forces[j].y);
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

    for (unsigned int k = 0; k < taxel.centers_x.size(); k++)
    {
        taxel.forces_x.push_back(0.0);
        taxel.forces_y.push_back(0.0);
        taxel.forces_z.push_back(0.0);
    }

    for (unsigned int j = 0; j <  f_ind.size(); j++)
    {
        taxel.forces_x[f_ind[j]] = taxel.forces_x[f_ind[j]] + skin.forces[j].x;
        taxel.forces_y[f_ind[j]] = taxel.forces_y[f_ind[j]] + skin.forces[j].y;
        taxel.forces_z[f_ind[j]] = taxel.forces_z[f_ind[j]] + skin.forces[j].z;
    }

    f_ind.clear();
    /*************************************************************************************/
    /*end of really bad code at least, all code above for taxel stuff needs drastic rework*/
    /****************************************************************************************/
}


void inner_torque_loop()
{
    q[0] = hinge1->getAngle();
    q[1] = hinge2->getAngle();
    q[2] = hinge3->getAngle();

    q_dot[0] = hinge1->getAngleRate();
    q_dot[1] = hinge2->getAngleRate();
    q_dot[2] = hinge3->getAngleRate();

    torques[0]=(-k_p[0]*(q[0]-jep[0]) - k_d[0]*q_dot[0]);	
    torques[1]=(-k_p[1]*(q[1]-jep[1]) - k_d[1]*q_dot[1]);	
    torques[2]=(-k_p[2]*(q[2]-jep[2]) - k_d[2]*q_dot[2]);      

    if (include_mobile_base != 0)
    {
        const dReal *base_pos, *base_vel;
        base_pos = body_mobile_base->getPosition();
        base_vel = body_mobile_base->getLinearVel();

        for(int i=0; i<3; i++)
            mobile_base_generalized_forces[i]=(-mobile_base_k_p[i]*(base_pos[i]-mobile_base_ep[i])
                    -mobile_base_k_d[i]*base_vel[i]);

        body_mobile_base->addRelForce(mobile_base_generalized_forces[0],
                mobile_base_generalized_forces[1],
                0);
        body_mobile_base->addRelTorque(0, 0, mobile_base_generalized_forces[2]);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_arm");

    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    ros::Publisher skin_pub = n.advertise<hrl_haptic_manipulation_in_clutter_msgs::SkinContact>("/skin/contacts", 100);
    ros::Publisher jep_pub = n.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/jep", 100);
    ros::Publisher angles_pub = n.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/joint_angles", 100);
    ros::Publisher angle_rates_pub = n.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/joint_angle_rates", 100);
    ros::Publisher bodies_draw = n.advertise<hrl_haptic_manipulation_in_clutter_msgs::BodyDraw>("/sim_arm/bodies_visualization", 100);
    ros::Publisher taxel_pub = n.advertise<hrl_haptic_manipulation_in_clutter_msgs::TaxelArray>("/skin/taxel_array", 100);
    ros::Publisher imped_pub = n.advertise<hrl_haptic_manipulation_in_clutter_msgs::MechanicalImpedanceParams>("sim_arm/joint_impedance", 100);

    // x, y, angle of mobile base.
    tf::TransformBroadcaster br;
    tf::Transform tf_transform;
    ros::Publisher odom_pub = n.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/odometry", 100);
    ros::Publisher base_ep_pub = n.advertise<hrl_msgs::FloatArrayBare>("/sim_arm/base_ep", 100);
    ROS_WARN("Not publishing angle in odometry\n");

    ros::Subscriber s1 = n.subscribe("/sim_arm/command/jep", 100, jep_cb);
    ros::Subscriber s2 = n.subscribe("/sim_arm/command/base_ep", 100, base_ep_cb);
    ros::Subscriber s3 = n.subscribe("/sim_arm/command/joint_impedance", 100, ROSCallback_impedance);

    //const dReal timestep = 0.0005;
    const dReal timestep = 0.0001;
    ros::Publisher clock_pub = n.advertise<rosgraph_msgs::Clock>("/clock", 500);

    double cur_time = 0;
    int torque_step = 0;
    int q_pub_step = 0;
    int clock_pub_step = 0;
    int skin_step = 0;

    private_nh.param<int>("include_mobile_base", include_mobile_base, 0);

    int resol;
    //    float resolution;
    printf("waiting for /m3/software_testbed/resolution\n");
    while (n.getParam("/m3/software_testbed/resolution", resol) == false)
        sleep(0.1);
    printf("Done waiting\n");

    double resolution = (double) resol;

    dInitODE();
    // setup pointers to drawstuff callback functions

    space = new dSimpleSpace();

    world = new dWorld;
    world->setGravity(0, 0, -9.8);

    // make robot and go to starting configuration.
    create_robot(n);
    ROS_INFO("Before going to initial position");
    go_initial_position(n); // initial jep defined inside this function.
    ROS_INFO("After going to initial position");

    // add obstacles.
    create_movable_obstacles(n); //call this first for stupid ROS param server sync.
    create_compliant_obstacles(n);
    create_fixed_obstacles(n);

    printf("Starting Simulation now \n");
    double t_now = get_wall_clock_time() - timestep;
    double t_expected;

    while (ros::ok())    
    {
        space->collide(0, nearCallback);

        // simulation will not run faster than real-time.
        t_expected = t_now + timestep;
        t_now = get_wall_clock_time();
        if (t_now < t_expected)
            usleep((t_expected - t_now)*1000000. + 0.5);

        world->step(timestep);
        cur_time += timestep;

        geometry_msgs::Vector3 force;	
        geometry_msgs::Vector3 normal;	

        assert(fbnum <= MAX_FEEDBACKNUM);

        dVector3 sum = {0, 0, 0};

        for (int i=0; i<fbnum; i++) 
        {
            dReal *f = feedbacks[i].fb.f2;
            sum[0] += f[0] * force_sign[i];
            sum[1] += f[1] * force_sign[i];
            sum[2] += f[2] * force_sign[i];

            double f_mag = sqrt(sum[0]*sum[0]+sum[1]*sum[1]+sum[2]*sum[2]);
            if (i < fbnum-1)
            {
                if (force_grouping[i] != force_grouping[i+1])
                {
                    force.x = sum[0];
                    force.y = sum[1];
                    force.z = sum[2];
                    skin.forces.push_back(force);

                    // hacky code by Advait to add normals to the
                    // SkinContact message
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
                normal.x = force.x / f_mag;
                normal.y = force.y / f_mag;
                normal.z = force.z / f_mag;
                skin.normals.push_back(normal);

                sum[0] = 0;
                sum[1] = 0;
                sum[2] = 0;
            }
        }


        torque_step++;
        q_pub_step++;
        clock_pub_step++;
        skin_step++;

        if (clock_pub_step >= 0.002/timestep)
        {
            rosgraph_msgs::Clock c;
            c.clock.sec = int(cur_time);
            c.clock.nsec = int(1000000000*(cur_time-int(cur_time)));
            clock_pub.publish(c);
            clock_pub_step = 0;
        }

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
        }

        for (int l = 0; l<num_used_compliant; l++)
        {
            dJointSetPlane2DXParam(compliant_plane2d_joint_ids[l], dParamFMax, 0);
            dJointSetPlane2DYParam(compliant_plane2d_joint_ids[l], dParamFMax, 0);
            const dReal *cur_pos;
            cur_pos = dBodyGetPosition(compliant_obstacles[l].id());
            const dReal *cur_vel;
            cur_vel = dBodyGetLinearVel(compliant_obstacles[l].id());

            //this is the control law used to simulate compliant objects with critical damping
            dReal Fx = (obst_home[l][0]-cur_pos[0])*obst_stiffness[l] - cur_vel[0]*obst_damping[l];
            dReal Fy = (obst_home[l][1]-cur_pos[1])*obst_stiffness[l] - cur_vel[1]*obst_damping[l];

            dBodyAddForce(compliant_obstacles[l].id(), Fx, Fy, 0);
            dBodyAddForce(compliant_obstacles[l].id(), 0, 0, 0);
        }


	//computing torques at 1 kHz but applying at every simulation time step
	//because ode zeros the "accumulators" after every time step
        if (torque_step >= 0.001/timestep)
        {
            inner_torque_loop();
            torque_step = 0;
        }
	hinge1->addTorque(torques[0]);
	hinge2->addTorque(torques[1]);	
	hinge3->addTorque(torques[2]);		
	//-----------------------------------------------------------------------


        if (q_pub_step >= 0.01/timestep)
        {
            hrl_msgs::FloatArrayBare angles;
            hrl_msgs::FloatArrayBare angle_rates;

            angles.data = q;
            angle_rates.data = q_dot;

            angles_pub.publish(angles);
            angle_rates_pub.publish(angle_rates);

            hrl_msgs::FloatArrayBare jep_ros;
            jep_ros.data = jep;
            jep_pub.publish(jep_ros);

            q_pub_step = 0;

            if (include_mobile_base != 0)
            {
                const dReal *base_pos, *base_vel;
                base_pos = body_mobile_base->getPosition();
                base_vel = body_mobile_base->getLinearVel();

                hrl_msgs::FloatArrayBare bp;
                bp.data.resize(3);
                bp.data[0] = base_pos[0]; // x
                bp.data[1] = base_pos[1]; // y
                bp.data[2] = 0.; // ang
                odom_pub.publish(bp);

                tf_transform.setOrigin(tf::Vector3(base_pos[0],
                            base_pos[1], 0.0));
                tf_transform.setRotation(tf::Quaternion(base_pos[2], 0,
                            0));
                br.sendTransform(tf::StampedTransform(tf_transform,
                            ros::Time::now(), "/world",
                            "/torso_lift_link"));

                hrl_msgs::FloatArrayBare bep;
                bep.data.resize(3);
                bep.data[0] = mobile_base_ep[0];
                bep.data[1] = mobile_base_ep[1];
                bep.data[2] = mobile_base_ep[2];
                base_ep_pub.publish(bep);

            }
            else
            {
                tf_transform.setOrigin(tf::Vector3(0.0,
                            0.0, 
                            0.0));
                tf_transform.setRotation(tf::Quaternion(0.0, 
                            0.0,
                            0.0));
                br.sendTransform(tf::StampedTransform(tf_transform,
                            ros::Time::now(), "/world",
                            "/torso_lift_link"));
            }
        }

        if (skin_step >= 0.01/timestep)
        { 
            // this block of code gets executed every 10ms (100Hz)
            // simulator time.

            // unsure what this is.
            impedance_params.header.frame_id = "/world";
            impedance_params.header.stamp = ros::Time::now();
            impedance_params.k_p.data = k_p;
            impedance_params.k_d.data = k_d;
            imped_pub.publish(impedance_params);

            // publishing skin msg. (maybe perform the computation,
            // taxel resolution etc at a 100Hz as well?)
            skin.header.frame_id = "/world";
            skin.header.stamp = ros::Time::now();
            skin_pub.publish(skin);

            taxel_simulation_code(resolution);
            taxel_pub.publish(taxel);

            taxel.centers_x.clear();
            taxel.centers_y.clear();
            taxel.centers_z.clear();
            taxel.normals_x.clear();
            taxel.normals_y.clear();
            taxel.normals_z.clear();
            taxel.forces_x.clear();
            taxel.forces_y.clear();
            taxel.forces_z.clear();
            taxel.link_names.clear();

            //--------------------------------------------------------- 
            // visualize bodies in rviz.
            // will publish data that a python script will use to the
            // publish rviz markers.
            //---------------------------------------------------------

            hrl_msgs::FloatArrayBare link_pos_ar;
            hrl_msgs::FloatArrayBare link_rot_ar;
            hrl_msgs::FloatArrayBare obst_pos_ar;
            hrl_msgs::FloatArrayBare obst_rot_ar;

            const dReal *position = link1->getPosition();
            const dReal *rotation = link1->getRotation();
            std::vector<double> pos_vec(3);
            std::vector<double> rot_vec(12);
            for (int l = 0; l<3; l++)
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

            // movable obstacles.
            for (int l = 0; l<num_used_movable; l++)
            {
                position = dBodyGetPosition(obstacles[l].id());
                rotation = dBodyGetRotation(obstacles[l].id());
                pos_vec[0] = position[0];
                pos_vec[1] = position[1];
                pos_vec[2] = position[2];
                obst_pos_ar.data = pos_vec;
                draw.obst_loc.push_back(obst_pos_ar);

                for (int k = 0; k<12; k++)
                    rot_vec[k] = rotation[k];

                obst_rot_ar.data = rot_vec;
                draw.obst_rot.push_back(obst_rot_ar);
            }

            //compliant obstacles
            for (int l = 0; l<num_used_compliant; l++)
            {
                position = dBodyGetPosition(compliant_obstacles[l].id());
                rotation = dBodyGetRotation(compliant_obstacles[l].id());
                pos_vec[0] = position[0];
                pos_vec[1] = position[1];
                pos_vec[2] = position[2];
                obst_pos_ar.data = pos_vec;
                draw.obst_loc.push_back(obst_pos_ar);

                for (int k = 0; k<12; k++)
                    rot_vec[k] = rotation[k];

                obst_rot_ar.data = rot_vec;
                draw.obst_rot.push_back(obst_rot_ar);
            }


            // fixed obstacles.
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
                    rot_vec[k] = rotation[k];

                obst_rot_ar.data = rot_vec;
                draw.obst_rot.push_back(obst_rot_ar);
            }

            draw.header.frame_id = "/world";
            draw.header.stamp = ros::Time::now();
            bodies_draw.publish(draw);
            skin_step = 0;

            // clear for the next time.
            draw.link_loc.clear();
            draw.link_rot.clear();
            draw.obst_loc.clear();
            draw.obst_rot.clear();
        }

        skin.pts_x.clear();
        skin.pts_y.clear();
        skin.pts_z.clear();
        skin.link_names.clear();
        skin.locations.clear();
        skin.forces.clear();
        skin.normals.clear();
        force_grouping.clear();
        force_sign.clear();
        joints.clear();	

        fbnum = 0;
        force_group = 0;

        ros::spinOnce();
    }

    dCloseODE();
}


