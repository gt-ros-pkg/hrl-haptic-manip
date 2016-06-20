/*********************************************************************
 *
 * Software License Agreement (New BSD License)
 *
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of Georgia Tech nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE GEORGIA TECH RESEARCH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Authors: Kevin Chow, Daehyung Park
 *  Healthcare Robotics Laboratory
 *********************************************************************/


#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/Events.hh>
#include <common/common.hh>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "sttr_msgs/RagdollObjectArray.h"
#include "sttr_msgs/RagdollWrenchArray.h"
#include "sttr_srvs/GazeboObjectState.h"
#include "sttr_msgs/WrenchArray.h"

namespace gazebo
{   
  class ROSWorldPlugin : public WorldPlugin
  {

    public: ROSWorldPlugin()
    {
      this->world_created_ = false;
    }
    public: ~ROSWorldPlugin()
    {
      // disconnect slots
      event::Events::DisconnectWorldUpdateBegin(this->time_update_event_);
      event::Events::DisconnectWorldUpdateBegin(this->object_update_event_);

      // shutdown ros
      this->rosnode_->shutdown();
      delete this->rosnode_;

    }


    
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      // setup ros related
      if (!ros::isInitialized()){
        std::string name = "ros_world_plugin_node";
        int argc = 0;
        ros::init(argc,NULL,name,ros::init_options::NoSigintHandler);
      }
      else
        ROS_ERROR("Something other than this ros_world plugin started ros::init(...), clock may not be published properly.");

      this->rosnode_ = new ros::NodeHandle("~");

      this->lock_.lock();
      if (this->world_created_)
      {
        this->lock_.unlock();
        return;
      }

      // set flag to true and load this plugin
      this->world_created_ = true;
      this->lock_.unlock();

      this->world = physics::get_world(_parent->GetName());
      if (!this->world)
      {
        ROS_FATAL("cannot load gazebo ros world server plugin, physics::get_world() fails to return world");
        return;
      }

      /// \brief advertise all services
      this->AdvertiseServices();

      // hooks for applying forces, publishing simtime on /clock
      this->time_update_event_   = event::Events::ConnectWorldUpdateBegin(boost::bind(&ROSWorldPlugin::publishSimTime,this));
      this->object_update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ROSWorldPlugin::publishObjectInfo,this));

      crona_wrench_msg_available = 0;
      object_wrench_msg_available = 0;
////////////////////////////////////////////////////////////////////////////////
      /*
      physics::Model_V models_;
      unsigned int nModel = 0;

      models_ = this->world->GetModels();
      nModel = this->world->GetModelCount();
      
      for (unsigned int i = 0; i < nModel; i++){
        std::string modelName = models_[i]->GetName();
        if (modelName.find("ragdoll") != std::string::npos){
	  physics::Joint_V joints_;
          unsigned int njoints = 0;
	  joints_ = this->model->GetJoints();
      	  njoints= this->model->GetJointCount();
	  for (unsigned int k = 0; k < njoints; k++){
	    std::string jointName = joints_[k]->GetName();
	    if (jointName.find("upper_body_joint") != std::string::npos){
	      gzdbg << "HERE upper_body_joint" << "\n";
	    }
	  }
        }
      }
      */
///////////////////////////////////////////////////////////////////////////////////
    }
    
    /// \brief advertise services
    void AdvertiseServices()
    {
      
      // publish clock for simulated ros time
      this->pub_clock_         = this->rosnode_->advertise<rosgraph_msgs::Clock>("/clock",1/0.0005);

      // publish object state 
      this->pub_object_        = this->rosnode_->advertise<sttr_msgs::RagdollObjectArray>("/gazebo/object", 100);

      this->pub_objectcog_     = this->rosnode_->advertise<sttr_msgs::RagdollObjectArray>("/gazebo/objectcog", 100);
      
      this->pub_objectft_     = this->rosnode_->advertise<sttr_msgs::WrenchArray>("/gazebo/objectft", 100);

      this->pub_cronaft_       = this->rosnode_->advertise<sttr_msgs::WrenchArray>("/gazebo/cronaft", 100);
      
      this->pub_cronajt_       = this->rosnode_->advertise<sttr_msgs::WrenchArray>("/gazebo/cronajt", 100);
      
      this->pub_liftboxft_     = this->rosnode_->advertise<sttr_msgs::WrenchArray>("/gazebo/liftboxft", 100);

      this->pub_ragdollcog_     = this->rosnode_->advertise<sttr_msgs::RagdollObjectArray>("/gazebo/ragdollcog", 100);

      // subscribe object switch command
      this->sub_move_ragdoll_ = this->rosnode_->subscribe<std_msgs::Bool>("/gazebo/move_ragdoll", 1000, &ROSWorldPlugin::MoveRagdollCallback, this );

      this->sub_rigid_ = this->rosnode_->subscribe<std_msgs::Bool>("/gazebo/object_rigid", 1000, &ROSWorldPlugin::SetRigidCallback, this);
	
      this->sub_move_boxes_ = this->rosnode_->subscribe<std_msgs::Bool>("/gazebo/move_boxes", 1000, &ROSWorldPlugin::MoveBoxesCallback, this );
	
      // subscribe object wrench command
      this->sub_object_wrench_ = this->rosnode_->subscribe<sttr_msgs::WrenchArray>("/gazebo/object_wrench", 1000, &ROSWorldPlugin::ApplyObjectWrenchCallback, this );
      this->sub_crona_wrench_ = this->rosnode_->subscribe<sttr_msgs::WrenchArray>("/gazebo/crona_wrench", 1000, &ROSWorldPlugin::ApplycRoNAWrenchCallback, this );
      this->sub_ragdoll_wrench_ = this->rosnode_->subscribe<sttr_msgs::WrenchArray>("/gazebo/ragdoll_wrench", 1000, &ROSWorldPlugin::ApplyRagdollWrenchCallback, this );

      this->sub_forearm_roller_accel_ = this->rosnode_->subscribe<sttr_msgs::RagdollWrenchArray>("/gazebo/forearm_roller_accel", 1000, &ROSWorldPlugin::ApplyForearmRollerCallback, this );

      // set param for use_sim_time if not set by user alread
      this->rosnode_->setParam("/use_sim_time", true);
    }

    void publishSimTime()
    {
      common::Time currentTime = this->world->GetSimTime();
      rosgraph_msgs::Clock ros_time_;
      ros_time_.clock.fromSec(currentTime.Double());
      //  publish time to ros
      this->pub_clock_.publish(ros_time_);
    }

    // Publish Object Properties and Positions
    void publishObjectInfo()
    {
      this->lock_.lock();

      physics::Model_V models_;
      unsigned int nModel = 0;
      physics::Link_V links_;
      physics::LinkPtr link_;
      physics::LinkPtr child_link_;
      physics::Joint_V joints_;
      std::string linkName;
      std::string jointName;
      math::Pose obs_pose;
      math::Pose obj_pose;
      math::Vector3 object_forces;
      math::Vector3 object_torques;
      math::Vector3 crona_forces_at_forearm;
      math::Vector3 crona_torques_at_forearm;
      physics::JointWrench crona_jt;
      math::Vector3 crona_forces;
      math::Vector3 crona_torques;
      math::Vector3 crona_jointtorque;
      sdf::ElementPtr bodyElem_;
      physics::InertialPtr inertial_;
      math::Pose link_cog;
      math::Pose link_pose;
      double ragdoll_cog_num_x = 0;
      double ragdoll_cog_num_y = 0;
      double ragdoll_cog_num_z = 0;
      double ragdoll_cog_den = 0;
      double ragdoll_m1_num_x = 0;
      double ragdoll_m1_num_y = 0;
      double ragdoll_m1_num_z = 0;
      double ragdoll_m1_den = 0;
      double ragdoll_m2_num_x = 0;
      double ragdoll_m2_num_y = 0;
      double ragdoll_m2_num_z = 0;
      double ragdoll_m2_den = 0;
      double ragdoll_m3_num_x = 0;
      double ragdoll_m3_num_y = 0;
      double ragdoll_m3_num_z = 0;
      double ragdoll_m3_den = 0;
      double lower_body_mass = 10.14;
      double l_thigh_mass = 9.912;
      double l_shin_mass = 3.031;
      double l_foot_mass = 0.959;
      double r_thigh_mass = 9.912;
      double r_shin_mass = 3.031;
      double r_foot_mass = 0.959;
      double middle_body_mass = 10.14;
      double upper_body_mass = 10.14;
      double l_arm_mass = 1.897;
      double l_wrist_mass = 1.135;
      double l_hand_mass = 0.427;
      double neck_mass = 1.1;
      double head_mass = 2.429;
      double r_arm_mass = 1.897;
      double r_wrist_mass = 1.135;
      double r_hand_mass = 0.427;
      math::Pose ragdoll_m1;
      math::Pose ragdoll_m2;
      math::Pose ragdoll_m3;
      math::Pose ragdoll_cog;
      math::Pose ragdoll_lower_body_cog;
      math::Pose ragdoll_l_thigh_cog;
      //math::Pose ragdoll_l_shin_cog;
      //math::Pose ragdoll_l_foot_cog;
      math::Pose ragdoll_r_thigh_cog;
      //math::Pose ragdoll_r_shin_cog;
      //math::Pose ragdoll_r_foot_cog;
      //math::Pose ragdoll_middle_body_cog;
      math::Pose ragdoll_upper_body_cog;
      //math::Pose ragdoll_l_arm_cog;
      //math::Pose ragdoll_l_wrist_cog;
      //math::Pose ragdoll_l_hand_cog;
      //math::Pose ragdoll_neck_cog;
      //math::Pose ragdoll_head_cog;
      //math::Pose ragdoll_r_arm_cog;
      //math::Pose ragdoll_r_wrist_cog;
      //math::Pose ragdoll_r_hand_cog;
      math::Pose ragdoll_lower_body_pose;
      math::Pose ragdoll_l_thigh_pose;
      math::Pose ragdoll_l_shin_pose;
      math::Pose ragdoll_l_foot_pose;
      math::Pose ragdoll_r_thigh_pose;
      math::Pose ragdoll_r_shin_pose;
      math::Pose ragdoll_r_foot_pose;
      math::Pose ragdoll_middle_body_pose;
      math::Pose ragdoll_upper_body_pose;
      math::Pose ragdoll_l_arm_pose;
      math::Pose ragdoll_l_wrist_pose;
      math::Pose ragdoll_l_hand_pose;
      math::Pose ragdoll_neck_pose;
      math::Pose ragdoll_head_pose;
      math::Pose ragdoll_r_arm_pose;
      math::Pose ragdoll_r_wrist_pose;
      math::Pose ragdoll_r_hand_pose;

      models_ = this->world->GetModels();
      nModel = this->world->GetModelCount();
      

      // Set header
      this->pub_object_array_.header.frame_id = "world";
      this->pub_object_array_.header.stamp = ros::Time(this->world->GetSimTime().Double());

      for (unsigned int i = 0; i < nModel; i++){
        std::string modelName = models_[i]->GetName();

        // Get only obstacles
        if (modelName.find("ragdoll") != std::string::npos){
          obs_pose = models_[i]->GetWorldPose();

  	  obj_pose = links_[0]->GetWorldCoGPose();
          //this->pub_objectcog_array_.frame_names.push_back("small_lift_box"); 
          this->pub_object_array_.frame_names.push_back(modelName); // not correct
          this->pub_object_array_.centers_x.push_back(obs_pose.pos.x);
          this->pub_object_array_.centers_y.push_back(obs_pose.pos.y);
          this->pub_object_array_.centers_z.push_back(obs_pose.pos.z);
	  this->pub_object_array_.rotation_x.push_back(obs_pose.rot.x);
	  this->pub_object_array_.rotation_y.push_back(obs_pose.rot.y);
	  this->pub_object_array_.rotation_z.push_back(obs_pose.rot.z);
	  this->pub_object_array_.rotation_w.push_back(obs_pose.rot.w);
	
 	  links_ = models_[i]->GetLinks();
	  for (unsigned int j = 0; j < links_.size(); j++){
            std::string linkName = links_[j]->GetName();
	    //inertial_ = links_[j]->GetInertial();
	    //double mass = inertial_->GetMass();
	    link_cog = links_[j]->GetWorldCoGPose();
	    link_pose = links_[j]->GetCollision(0)->GetWorldPose();
	    if (ragdoll_wrench_msg_available == 1){
		if (linkName.find("body") != std::string::npos){
                    links_[j]->AddRelativeForce(ragdoll_body_force_vector);
		}
		else {
                    links_[j]->AddRelativeForce(ragdoll_force_vector);
		}
            }

	    if (linkName.find("lower_body") != std::string::npos){
		ragdoll_lower_body_cog.pos.x = link_cog.pos.x;
		ragdoll_lower_body_cog.pos.y = link_cog.pos.y;
		ragdoll_lower_body_cog.pos.z = link_cog.pos.z;
	        ragdoll_lower_body_cog.rot.x = link_cog.rot.x;
                ragdoll_lower_body_cog.rot.y = link_cog.rot.y;
                ragdoll_lower_body_cog.rot.z = link_cog.rot.z;
                ragdoll_lower_body_cog.rot.w = link_cog.rot.w;
		ragdoll_lower_body_pose.pos.x = link_pose.pos.x;
                ragdoll_lower_body_pose.pos.y = link_pose.pos.y;
                ragdoll_lower_body_pose.pos.z = link_pose.pos.z;
                ragdoll_lower_body_pose.rot.x = link_pose.rot.x;
                ragdoll_lower_body_pose.rot.y = link_pose.rot.y;
                ragdoll_lower_body_pose.rot.z = link_pose.rot.z;
                ragdoll_lower_body_pose.rot.w = link_pose.rot.w;
	  	ragdoll_m2_num_x = ragdoll_m2_num_x + lower_body_mass*link_cog.pos.x;
                ragdoll_m2_num_y = ragdoll_m2_num_y + lower_body_mass*link_cog.pos.y;
                ragdoll_m2_num_z = ragdoll_m2_num_z + lower_body_mass*link_cog.pos.z;
                ragdoll_m2_den = ragdoll_m2_den + lower_body_mass;
		ragdoll_cog_num_x = ragdoll_cog_num_x + lower_body_mass*link_cog.pos.x;
            	ragdoll_cog_num_y = ragdoll_cog_num_y + lower_body_mass*link_cog.pos.y;
            	ragdoll_cog_num_z = ragdoll_cog_num_z + lower_body_mass*link_cog.pos.z;
            	ragdoll_cog_den = ragdoll_cog_den + lower_body_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("lower_body") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
		}
		
	    }
	    if (linkName.find("l_thigh") != std::string::npos){
                ragdoll_l_thigh_cog.pos.x = link_cog.pos.x;
                ragdoll_l_thigh_cog.pos.y = link_cog.pos.y;
                ragdoll_l_thigh_cog.pos.z = link_cog.pos.z;
		ragdoll_l_thigh_cog.rot.x = link_cog.rot.x;
                ragdoll_l_thigh_cog.rot.y = link_cog.rot.y;
                ragdoll_l_thigh_cog.rot.z = link_cog.rot.z;
                ragdoll_l_thigh_cog.rot.w = link_cog.rot.w;
		ragdoll_l_thigh_pose.pos.x = link_pose.pos.x;
                ragdoll_l_thigh_pose.pos.y = link_pose.pos.y;
                ragdoll_l_thigh_pose.pos.z = link_pose.pos.z;
                ragdoll_l_thigh_pose.rot.x = link_pose.rot.x;
                ragdoll_l_thigh_pose.rot.y = link_pose.rot.y;
                ragdoll_l_thigh_pose.rot.z = link_pose.rot.z;
                ragdoll_l_thigh_pose.rot.w = link_pose.rot.w;
		ragdoll_m2_num_x = ragdoll_m2_num_x + l_thigh_mass*link_cog.pos.x;
                ragdoll_m2_num_y = ragdoll_m2_num_y + l_thigh_mass*link_cog.pos.y;
                ragdoll_m2_num_z = ragdoll_m2_num_z + l_thigh_mass*link_cog.pos.z;
                ragdoll_m2_den = ragdoll_m2_den + l_thigh_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + l_thigh_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + l_thigh_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + l_thigh_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + l_thigh_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("l_thigh") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
	    if (linkName.find("l_shin") != std::string::npos){
		//ragdoll_l_shin_cog.pos.x = link_cog.pos.x;
                //ragdoll_l_shin_cog.pos.y = link_cog.pos.y;
                //ragdoll_l_shin_cog.pos.z = link_cog.pos.z;
                //ragdoll_l_shin_cog.rot.x = link_cog.rot.x;
                //ragdoll_l_shin_cog.rot.y = link_cog.rot.y;
                //ragdoll_l_shin_cog.rot.z = link_cog.rot.z;
                //ragdoll_l_shin_cog.rot.w = link_cog.rot.w;
		ragdoll_l_shin_pose.pos.x = link_pose.pos.x;
                ragdoll_l_shin_pose.pos.y = link_pose.pos.y;
                ragdoll_l_shin_pose.pos.z = link_pose.pos.z;
                ragdoll_l_shin_pose.rot.x = link_pose.rot.x;
                ragdoll_l_shin_pose.rot.y = link_pose.rot.y;
                ragdoll_l_shin_pose.rot.z = link_pose.rot.z;
                ragdoll_l_shin_pose.rot.w = link_pose.rot.w;
		ragdoll_m1_num_x = ragdoll_m1_num_x + l_shin_mass*link_cog.pos.x;
                ragdoll_m1_num_y = ragdoll_m1_num_y + l_shin_mass*link_cog.pos.y;
                ragdoll_m1_num_z = ragdoll_m1_num_z + l_shin_mass*link_cog.pos.z;
                ragdoll_m1_den = ragdoll_m1_den + l_shin_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + l_shin_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + l_shin_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + l_shin_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + l_shin_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("l_shin") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
	    if (linkName.find("l_foot") != std::string::npos){
		//ragdoll_l_foot_cog.pos.x = link_cog.pos.x;
                //ragdoll_l_foot_cog.pos.y = link_cog.pos.y;
                //ragdoll_l_foot_cog.pos.z = link_cog.pos.z;
                //ragdoll_l_foot_cog.rot.x = link_cog.rot.x;
                //ragdoll_l_foot_cog.rot.y = link_cog.rot.y;
                //ragdoll_l_foot_cog.rot.z = link_cog.rot.z;
                //ragdoll_l_foot_cog.rot.w = link_cog.rot.w;
		ragdoll_l_foot_pose.pos.x = link_pose.pos.x;
                ragdoll_l_foot_pose.pos.y = link_pose.pos.y;
                ragdoll_l_foot_pose.pos.z = link_pose.pos.z;
                ragdoll_l_foot_pose.rot.x = link_pose.rot.x;
                ragdoll_l_foot_pose.rot.y = link_pose.rot.y;
                ragdoll_l_foot_pose.rot.z = link_pose.rot.z;
                ragdoll_l_foot_pose.rot.w = link_pose.rot.w;
		ragdoll_m1_num_x = ragdoll_m1_num_x + l_foot_mass*link_cog.pos.x;
                ragdoll_m1_num_y = ragdoll_m1_num_y + l_foot_mass*link_cog.pos.y;
                ragdoll_m1_num_z = ragdoll_m1_num_z + l_foot_mass*link_cog.pos.z;
                ragdoll_m1_den = ragdoll_m1_den + l_foot_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + l_foot_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + l_foot_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + l_foot_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + l_foot_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("l_foot") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
	    if (linkName.find("r_thigh") != std::string::npos){
                ragdoll_r_thigh_cog.pos.x = link_cog.pos.x;
                ragdoll_r_thigh_cog.pos.y = link_cog.pos.y;
                ragdoll_r_thigh_cog.pos.z = link_cog.pos.z;
		ragdoll_r_thigh_cog.rot.x = link_cog.rot.x;
                ragdoll_r_thigh_cog.rot.y = link_cog.rot.y;
                ragdoll_r_thigh_cog.rot.z = link_cog.rot.z;
                ragdoll_r_thigh_cog.rot.w = link_cog.rot.w;
		ragdoll_r_thigh_pose.pos.x = link_pose.pos.x;
                ragdoll_r_thigh_pose.pos.y = link_pose.pos.y;
                ragdoll_r_thigh_pose.pos.z = link_pose.pos.z;
                ragdoll_r_thigh_pose.rot.x = link_pose.rot.x;
                ragdoll_r_thigh_pose.rot.y = link_pose.rot.y;
                ragdoll_r_thigh_pose.rot.z = link_pose.rot.z;
                ragdoll_r_thigh_pose.rot.w = link_pose.rot.w;
		ragdoll_m2_num_x = ragdoll_m2_num_x + r_thigh_mass*link_cog.pos.x;
                ragdoll_m2_num_y = ragdoll_m2_num_y + r_thigh_mass*link_cog.pos.y;
                ragdoll_m2_num_z = ragdoll_m2_num_z + r_thigh_mass*link_cog.pos.z;
                ragdoll_m2_den = ragdoll_m2_den + r_thigh_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + r_thigh_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + r_thigh_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + r_thigh_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + r_thigh_mass;
		ragdoll_m2.rot.x = link_cog.rot.x;
		ragdoll_m2.rot.y = link_cog.rot.y;
		ragdoll_m2.rot.z = link_cog.rot.z;
		ragdoll_m2.rot.w = link_cog.rot.w;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("r_thigh") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
	    if (linkName.find("r_shin") != std::string::npos){
		//ragdoll_r_shin_cog.pos.x = link_cog.pos.x;
                //ragdoll_r_shin_cog.pos.y = link_cog.pos.y;
                //ragdoll_r_shin_cog.pos.z = link_cog.pos.z;
                //ragdoll_r_shin_cog.rot.x = link_cog.rot.x;
                //ragdoll_r_shin_cog.rot.y = link_cog.rot.y;
                //ragdoll_r_shin_cog.rot.z = link_cog.rot.z;
                //ragdoll_r_shin_cog.rot.w = link_cog.rot.w;
		ragdoll_r_shin_pose.pos.x = link_pose.pos.x;
                ragdoll_r_shin_pose.pos.y = link_pose.pos.y;
                ragdoll_r_shin_pose.pos.z = link_pose.pos.z;
                ragdoll_r_shin_pose.rot.x = link_pose.rot.x;
                ragdoll_r_shin_pose.rot.y = link_pose.rot.y;
                ragdoll_r_shin_pose.rot.z = link_pose.rot.z;
                ragdoll_r_shin_pose.rot.w = link_pose.rot.w;
		ragdoll_m1_num_x = ragdoll_m1_num_x + r_shin_mass*link_cog.pos.x;
                ragdoll_m1_num_y = ragdoll_m1_num_y + r_shin_mass*link_cog.pos.y;
                ragdoll_m1_num_z = ragdoll_m1_num_z + r_shin_mass*link_cog.pos.z;
                ragdoll_m1_den = ragdoll_m1_den + r_shin_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + r_shin_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + r_shin_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + r_shin_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + r_shin_mass;
		ragdoll_m1.rot.x = link_cog.rot.x;
                ragdoll_m1.rot.y = link_cog.rot.y;
                ragdoll_m1.rot.z = link_cog.rot.z;
                ragdoll_m1.rot.w = link_cog.rot.w;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("r_shin") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
	    }
	    if (linkName.find("r_foot") != std::string::npos){
		//ragdoll_r_foot_cog.pos.x = link_cog.pos.x;
                //ragdoll_r_foot_cog.pos.y = link_cog.pos.y;
                //ragdoll_r_foot_cog.pos.z = link_cog.pos.z;
                //ragdoll_r_foot_cog.rot.x = link_cog.rot.x;
                //ragdoll_r_foot_cog.rot.y = link_cog.rot.y;
                //ragdoll_r_foot_cog.rot.z = link_cog.rot.z;
                //ragdoll_r_foot_cog.rot.w = link_cog.rot.w;
		ragdoll_r_foot_pose.pos.x = link_pose.pos.x;
                ragdoll_r_foot_pose.pos.y = link_pose.pos.y;
                ragdoll_r_foot_pose.pos.z = link_pose.pos.z;
                ragdoll_r_foot_pose.rot.x = link_pose.rot.x;
                ragdoll_r_foot_pose.rot.y = link_pose.rot.y;
                ragdoll_r_foot_pose.rot.z = link_pose.rot.z;
                ragdoll_r_foot_pose.rot.w = link_pose.rot.w;
		ragdoll_m1_num_x = ragdoll_m1_num_x + r_foot_mass*link_cog.pos.x;
                ragdoll_m1_num_y = ragdoll_m1_num_y + r_foot_mass*link_cog.pos.y;
                ragdoll_m1_num_z = ragdoll_m1_num_z + r_foot_mass*link_cog.pos.z;
                ragdoll_m1_den = ragdoll_m1_den + r_foot_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + r_foot_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + r_foot_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + r_foot_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + r_foot_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("r_foot") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
	    if (linkName.find("middle_body") != std::string::npos){
		//ragdoll_middle_body_cog.pos.x = link_cog.pos.x;
                //ragdoll_middle_body_cog.pos.y = link_cog.pos.y;
                //ragdoll_middle_body_cog.pos.z = link_cog.pos.z;
                //ragdoll_middle_body_cog.rot.x = link_cog.rot.x;
                //ragdoll_middle_body_cog.rot.y = link_cog.rot.y;
                //ragdoll_middle_body_cog.rot.z = link_cog.rot.z;
                //ragdoll_middle_body_cog.rot.w = link_cog.rot.w;
		ragdoll_middle_body_pose.pos.x = link_pose.pos.x;
                ragdoll_middle_body_pose.pos.y = link_pose.pos.y;
                ragdoll_middle_body_pose.pos.z = link_pose.pos.z;
                ragdoll_middle_body_pose.rot.x = link_pose.rot.x;
                ragdoll_middle_body_pose.rot.y = link_pose.rot.y;
                ragdoll_middle_body_pose.rot.z = link_pose.rot.z;
                ragdoll_middle_body_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + middle_body_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + middle_body_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + middle_body_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + middle_body_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + middle_body_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + middle_body_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + middle_body_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + middle_body_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("middle_body") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
	    if (linkName.find("upper_body") != std::string::npos){
                ragdoll_upper_body_cog.pos.x = link_cog.pos.x;
                ragdoll_upper_body_cog.pos.y = link_cog.pos.y;
                ragdoll_upper_body_cog.pos.z = link_cog.pos.z;
		ragdoll_upper_body_cog.rot.x = link_cog.rot.x;
                ragdoll_upper_body_cog.rot.y = link_cog.rot.y;
                ragdoll_upper_body_cog.rot.z = link_cog.rot.z;
                ragdoll_upper_body_cog.rot.w = link_cog.rot.w;
		ragdoll_upper_body_pose.pos.x = link_pose.pos.x;
                ragdoll_upper_body_pose.pos.y = link_pose.pos.y;
                ragdoll_upper_body_pose.pos.z = link_pose.pos.z;
                ragdoll_upper_body_pose.rot.x = link_pose.rot.x;
                ragdoll_upper_body_pose.rot.y = link_pose.rot.y;
                ragdoll_upper_body_pose.rot.z = link_pose.rot.z;
                ragdoll_upper_body_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + upper_body_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + upper_body_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + upper_body_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + upper_body_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + upper_body_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + upper_body_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + upper_body_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + upper_body_mass;
		ragdoll_m3.rot.x = link_cog.rot.x;
                ragdoll_m3.rot.y = link_cog.rot.y;
                ragdoll_m3.rot.z = link_cog.rot.z;
                ragdoll_m3.rot.w = link_cog.rot.w;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("upper_body") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
	    if (linkName.find("l_arm") != std::string::npos){
		//ragdoll_l_arm_cog.pos.x = link_cog.pos.x;
                //ragdoll_l_arm_cog.pos.y = link_cog.pos.y;
                //ragdoll_l_arm_cog.pos.z = link_cog.pos.z;
                //ragdoll_l_arm_cog.rot.x = link_cog.rot.x;
                //ragdoll_l_arm_cog.rot.y = link_cog.rot.y;
                //ragdoll_l_arm_cog.rot.z = link_cog.rot.z;
                //ragdoll_l_arm_cog.rot.w = link_cog.rot.w;
		ragdoll_l_arm_pose.pos.x = link_pose.pos.x;
                ragdoll_l_arm_pose.pos.y = link_pose.pos.y;
                ragdoll_l_arm_pose.pos.z = link_pose.pos.z;
                ragdoll_l_arm_pose.rot.x = link_pose.rot.x;
                ragdoll_l_arm_pose.rot.y = link_pose.rot.y;
                ragdoll_l_arm_pose.rot.z = link_pose.rot.z;
                ragdoll_l_arm_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + l_arm_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + l_arm_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + l_arm_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + l_arm_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + l_arm_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + l_arm_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + l_arm_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + l_arm_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("l_arm") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
	    if (linkName.find("l_wrist") != std::string::npos){
		//ragdoll_l_wrist_cog.pos.x = link_cog.pos.x;
                //ragdoll_l_wrist_cog.pos.y = link_cog.pos.y;
                //ragdoll_l_wrist_cog.pos.z = link_cog.pos.z;
                //ragdoll_l_wrist_cog.rot.x = link_cog.rot.x;
                //ragdoll_l_wrist_cog.rot.y = link_cog.rot.y;
                //ragdoll_l_wrist_cog.rot.z = link_cog.rot.z;
                //ragdoll_l_wrist_cog.rot.w = link_cog.rot.w;
		ragdoll_l_wrist_pose.pos.x = link_pose.pos.x;
                ragdoll_l_wrist_pose.pos.y = link_pose.pos.y;
                ragdoll_l_wrist_pose.pos.z = link_pose.pos.z;
                ragdoll_l_wrist_pose.rot.x = link_pose.rot.x;
                ragdoll_l_wrist_pose.rot.y = link_pose.rot.y;
                ragdoll_l_wrist_pose.rot.z = link_pose.rot.z;
                ragdoll_l_wrist_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + l_wrist_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + l_wrist_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + l_wrist_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + l_wrist_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + l_wrist_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + l_wrist_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + l_wrist_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + l_wrist_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("l_wrist") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
	    if (linkName.find("l_hand") != std::string::npos){
		//ragdoll_l_hand_cog.pos.x = link_cog.pos.x;
                //ragdoll_l_hand_cog.pos.y = link_cog.pos.y;
                //ragdoll_l_hand_cog.pos.z = link_cog.pos.z;
                //ragdoll_l_hand_cog.rot.x = link_cog.rot.x;
                //ragdoll_l_hand_cog.rot.y = link_cog.rot.y;
                //ragdoll_l_hand_cog.rot.z = link_cog.rot.z;
                //ragdoll_l_hand_cog.rot.w = link_cog.rot.w;
		ragdoll_l_hand_pose.pos.x = link_pose.pos.x;
                ragdoll_l_hand_pose.pos.y = link_pose.pos.y;
                ragdoll_l_hand_pose.pos.z = link_pose.pos.z;
                ragdoll_l_hand_pose.rot.x = link_pose.rot.x;
                ragdoll_l_hand_pose.rot.y = link_pose.rot.y;
                ragdoll_l_hand_pose.rot.z = link_pose.rot.z;
                ragdoll_l_hand_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + l_hand_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + l_hand_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + l_hand_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + l_hand_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + l_hand_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + l_hand_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + l_hand_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + l_hand_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("l_hand") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
	    if (linkName.find("neck") != std::string::npos){
		//ragdoll_neck_cog.pos.x = link_cog.pos.x;
                //ragdoll_neck_cog.pos.y = link_cog.pos.y;
                //ragdoll_neck_cog.pos.z = link_cog.pos.z;
                //ragdoll_neck_cog.rot.x = link_cog.rot.x;
                //ragdoll_neck_cog.rot.y = link_cog.rot.y;
                //ragdoll_neck_cog.rot.z = link_cog.rot.z;
                //ragdoll_neck_cog.rot.w = link_cog.rot.w;
		ragdoll_neck_pose.pos.x = link_pose.pos.x;
                ragdoll_neck_pose.pos.y = link_pose.pos.y;
                ragdoll_neck_pose.pos.z = link_pose.pos.z;
                ragdoll_neck_pose.rot.x = link_pose.rot.x;
                ragdoll_neck_pose.rot.y = link_pose.rot.y;
                ragdoll_neck_pose.rot.z = link_pose.rot.z;
                ragdoll_neck_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + neck_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + neck_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + neck_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + neck_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + neck_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + neck_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + neck_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + neck_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("neck") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
            if (linkName.find("head") != std::string::npos){
		//ragdoll_head_cog.pos.x = link_cog.pos.x;
                //ragdoll_head_cog.pos.y = link_cog.pos.y;
                //ragdoll_head_cog.pos.z = link_cog.pos.z;
                //ragdoll_head_cog.rot.x = link_cog.rot.x;
                //ragdoll_head_cog.rot.y = link_cog.rot.y;
                //ragdoll_head_cog.rot.z = link_cog.rot.z;
                //ragdoll_head_cog.rot.w = link_cog.rot.w;
		ragdoll_head_pose.pos.x = link_pose.pos.x;
                ragdoll_head_pose.pos.y = link_pose.pos.y;
                ragdoll_head_pose.pos.z = link_pose.pos.z;
                ragdoll_head_pose.rot.x = link_pose.rot.x;
                ragdoll_head_pose.rot.y = link_pose.rot.y;
                ragdoll_head_pose.rot.z = link_pose.rot.z;
                ragdoll_head_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + head_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + head_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + head_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + head_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + head_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + head_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + head_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + head_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("head") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
            if (linkName.find("r_arm") != std::string::npos){
		//ragdoll_r_arm_cog.pos.x = link_cog.pos.x;
                //ragdoll_r_arm_cog.pos.y = link_cog.pos.y;
                //ragdoll_r_arm_cog.pos.z = link_cog.pos.z;
                //ragdoll_r_arm_cog.rot.x = link_cog.rot.x;
                //ragdoll_r_arm_cog.rot.y = link_cog.rot.y;
                //ragdoll_r_arm_cog.rot.z = link_cog.rot.z;
                //ragdoll_r_arm_cog.rot.w = link_cog.rot.w;
		ragdoll_r_arm_pose.pos.x = link_pose.pos.x;
                ragdoll_r_arm_pose.pos.y = link_pose.pos.y;
                ragdoll_r_arm_pose.pos.z = link_pose.pos.z;
                ragdoll_r_arm_pose.rot.x = link_pose.rot.x;
                ragdoll_r_arm_pose.rot.y = link_pose.rot.y;
                ragdoll_r_arm_pose.rot.z = link_pose.rot.z;
                ragdoll_r_arm_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + r_arm_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + r_arm_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + r_arm_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + r_arm_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + r_arm_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + r_arm_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + r_arm_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + r_arm_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("r_arm") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
            if (linkName.find("r_wrist") != std::string::npos){
		//ragdoll_r_wrist_cog.pos.x = link_cog.pos.x;
                //ragdoll_r_wrist_cog.pos.y = link_cog.pos.y;
                //ragdoll_r_wrist_cog.pos.z = link_cog.pos.z;
                //ragdoll_r_wrist_cog.rot.x = link_cog.rot.x;
                //ragdoll_r_wrist_cog.rot.y = link_cog.rot.y;
                //ragdoll_r_wrist_cog.rot.z = link_cog.rot.z;
                //ragdoll_r_wrist_cog.rot.w = link_cog.rot.w;
		ragdoll_r_wrist_pose.pos.x = link_pose.pos.x;
                ragdoll_r_wrist_pose.pos.y = link_pose.pos.y;
                ragdoll_r_wrist_pose.pos.z = link_pose.pos.z;
                ragdoll_r_wrist_pose.rot.x = link_pose.rot.x;
                ragdoll_r_wrist_pose.rot.y = link_pose.rot.y;
                ragdoll_r_wrist_pose.rot.z = link_pose.rot.z;
                ragdoll_r_wrist_pose.rot.w = link_pose.rot.w;
		ragdoll_m3_num_x = ragdoll_m3_num_x + r_wrist_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + r_wrist_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + r_wrist_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + r_wrist_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + r_wrist_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + r_wrist_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + r_wrist_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + r_wrist_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("r_wrist") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
	    if (linkName.find("r_hand") != std::string::npos){
		//ragdoll_r_hand_cog.pos.x = link_cog.pos.x;
                //ragdoll_r_hand_cog.pos.y = link_cog.pos.y;
                //ragdoll_r_hand_cog.pos.z = link_cog.pos.z;
                //ragdoll_r_hand_cog.rot.x = link_cog.rot.x;
                //ragdoll_r_hand_cog.rot.y = link_cog.rot.y;
                //ragdoll_r_hand_cog.rot.z = link_cog.rot.z;
                //ragdoll_r_hand_cog.rot.w = link_cog.rot.w;
		ragdoll_r_hand_pose.pos.x = link_pose.pos.x;
                ragdoll_r_hand_pose.pos.y = link_pose.pos.y;
                ragdoll_r_hand_pose.pos.z = link_pose.pos.z;
                ragdoll_r_hand_pose.rot.x = link_pose.rot.x;
                ragdoll_r_hand_pose.rot.y = link_pose.rot.y;
                ragdoll_r_hand_pose.rot.z = link_pose.rot.z;
                ragdoll_r_hand_pose.rot.w = link_pose.rot.w;
                ragdoll_m3_num_x = ragdoll_m3_num_x + r_hand_mass*link_cog.pos.x;
                ragdoll_m3_num_y = ragdoll_m3_num_y + r_hand_mass*link_cog.pos.y;
                ragdoll_m3_num_z = ragdoll_m3_num_z + r_hand_mass*link_cog.pos.z;
                ragdoll_m3_den = ragdoll_m3_den + r_hand_mass;
                ragdoll_cog_num_x = ragdoll_cog_num_x + r_hand_mass*link_cog.pos.x;
                ragdoll_cog_num_y = ragdoll_cog_num_y + r_hand_mass*link_cog.pos.y;
                ragdoll_cog_num_z = ragdoll_cog_num_z + r_hand_mass*link_cog.pos.z;
                ragdoll_cog_den = ragdoll_cog_den + r_hand_mass;
		if (ragdoll_random_wrench_msg_available == 1){
                    if (ragdoll_force_location.find("r_hand") != std::string::npos){
                        links_[j]->AddForce(ragdoll_force_vector);
                    }
                }
            }
	
	  }

	  ragdoll_m1.pos.x = ragdoll_m1_num_x/ragdoll_m1_den;
          ragdoll_m1.pos.y = ragdoll_m1_num_y/ragdoll_m1_den;
          ragdoll_m1.pos.z = ragdoll_m1_num_z/ragdoll_m1_den;

          ragdoll_m2.pos.x = ragdoll_m2_num_x/ragdoll_m2_den;
          ragdoll_m2.pos.y = ragdoll_m2_num_y/ragdoll_m2_den;
          ragdoll_m2.pos.z = ragdoll_m2_num_z/ragdoll_m2_den;

          ragdoll_m3.pos.x = ragdoll_m3_num_x/ragdoll_m3_den;
          ragdoll_m3.pos.y = ragdoll_m3_num_y/ragdoll_m3_den;
          ragdoll_m3.pos.z = ragdoll_m3_num_z/ragdoll_m3_den;

          this->pub_objectcog_array_.frame_names.push_back("ragdoll_m1");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_m1.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_m1.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_m1.pos.z);
          this->pub_objectcog_array_.rotation_x.push_back(ragdoll_m1.rot.x);
          this->pub_objectcog_array_.rotation_y.push_back(ragdoll_m1.rot.y);
          this->pub_objectcog_array_.rotation_z.push_back(ragdoll_m1.rot.z);
          this->pub_objectcog_array_.rotation_w.push_back(ragdoll_m1.rot.w);
          this->pub_objectcog_array_.mass.push_back(ragdoll_m1_den);

          this->pub_objectcog_array_.frame_names.push_back("ragdoll_m2");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_m2.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_m2.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_m2.pos.z);
	  this->pub_objectcog_array_.rotation_x.push_back(ragdoll_m2.rot.x);
          this->pub_objectcog_array_.rotation_y.push_back(ragdoll_m2.rot.y);
          this->pub_objectcog_array_.rotation_z.push_back(ragdoll_m2.rot.z);
          this->pub_objectcog_array_.rotation_w.push_back(ragdoll_m2.rot.w);
          this->pub_objectcog_array_.mass.push_back(ragdoll_m2_den);

          this->pub_objectcog_array_.frame_names.push_back("ragdoll_m3");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_m3.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_m3.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_m3.pos.z);
	  this->pub_objectcog_array_.rotation_x.push_back(ragdoll_m3.rot.x);
          this->pub_objectcog_array_.rotation_y.push_back(ragdoll_m3.rot.y);
          this->pub_objectcog_array_.rotation_z.push_back(ragdoll_m3.rot.z);
          this->pub_objectcog_array_.rotation_w.push_back(ragdoll_m3.rot.w);
          this->pub_objectcog_array_.mass.push_back(ragdoll_m3_den);

	  this->pub_objectcog_array_.frame_names.push_back("ragdoll_lower_body_cog");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_lower_body_cog.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_lower_body_cog.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_lower_body_cog.pos.z);

	  this->pub_objectcog_array_.frame_names.push_back("ragdoll_l_thigh_cog");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_l_thigh_cog.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_l_thigh_cog.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_l_thigh_cog.pos.z);
	
	  this->pub_objectcog_array_.frame_names.push_back("ragdoll_r_thigh_cog");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_r_thigh_cog.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_r_thigh_cog.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_r_thigh_cog.pos.z);

	  this->pub_objectcog_array_.frame_names.push_back("ragdoll_upper_body_cog");
          this->pub_objectcog_array_.centers_x.push_back(ragdoll_upper_body_cog.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_upper_body_cog.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_upper_body_cog.pos.z);

	  ragdoll_cog.pos.x = ragdoll_cog_num_x/ragdoll_cog_den;
	  ragdoll_cog.pos.y = ragdoll_cog_num_y/ragdoll_cog_den;
	  ragdoll_cog.pos.z = ragdoll_cog_num_z/ragdoll_cog_den;
	  //gzdbg << "Ragdoll Center of Gravity: " << ragdoll_cog << "\n";

	  this->pub_objectcog_array_.frame_names.push_back("ragdoll_cog");
	  this->pub_objectcog_array_.centers_x.push_back(ragdoll_cog.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(ragdoll_cog.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(ragdoll_cog.pos.z);
	  this->pub_objectcog_array_.mass.push_back(ragdoll_cog_den);
	
	  // publish the cog of each ragdoll link
	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_lower_body_pose");
	  this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_lower_body_pose.pos.x);
	  this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_lower_body_pose.pos.y);
	  this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_lower_body_pose.pos.z);
	  this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_lower_body_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_lower_body_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_lower_body_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_lower_body_pose.rot.w);
	   
	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_l_thigh_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_l_thigh_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_l_thigh_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_l_thigh_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_l_thigh_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_l_thigh_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_l_thigh_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_l_thigh_pose.rot.w);

 	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_l_shin_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_l_shin_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_l_shin_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_l_shin_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_l_shin_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_l_shin_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_l_shin_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_l_shin_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_l_foot_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_l_foot_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_l_foot_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_l_foot_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_l_foot_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_l_foot_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_l_foot_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_l_foot_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_r_thigh_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_r_thigh_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_r_thigh_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_r_thigh_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_r_thigh_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_r_thigh_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_r_thigh_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_r_thigh_pose.rot.w);

          this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_r_shin_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_r_shin_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_r_shin_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_r_shin_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_r_shin_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_r_shin_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_r_shin_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_r_shin_pose.rot.w);

          this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_r_foot_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_r_foot_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_r_foot_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_r_foot_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_r_foot_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_r_foot_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_r_foot_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_r_foot_pose.rot.w);	

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_middle_body_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_middle_body_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_middle_body_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_middle_body_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_middle_body_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_middle_body_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_middle_body_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_middle_body_pose.rot.w);
	
	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_upper_body_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_upper_body_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_upper_body_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_upper_body_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_upper_body_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_upper_body_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_upper_body_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_upper_body_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_l_arm_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_l_arm_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_l_arm_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_l_arm_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_l_arm_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_l_arm_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_l_arm_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_l_arm_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_l_wrist_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_l_wrist_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_l_wrist_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_l_wrist_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_l_wrist_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_l_wrist_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_l_wrist_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_l_wrist_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_l_hand_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_l_hand_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_l_hand_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_l_hand_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_l_hand_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_l_hand_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_l_hand_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_l_hand_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_neck_cog");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_neck_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_neck_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_neck_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_neck_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_neck_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_neck_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_neck_pose.rot.w);

          this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_head_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_head_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_head_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_head_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_head_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_head_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_head_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_head_pose.rot.w);

	  this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_r_arm_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_r_arm_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_r_arm_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_r_arm_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_r_arm_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_r_arm_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_r_arm_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_r_arm_pose.rot.w);

          this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_r_wrist_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_r_wrist_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_r_wrist_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_r_wrist_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_r_wrist_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_r_wrist_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_r_wrist_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_r_wrist_pose.rot.w);

          this->pub_ragdollcog_array_.frame_names.push_back("ragdoll_r_hand_pose");
          this->pub_ragdollcog_array_.centers_x.push_back(ragdoll_r_hand_pose.pos.x);
          this->pub_ragdollcog_array_.centers_y.push_back(ragdoll_r_hand_pose.pos.y);
          this->pub_ragdollcog_array_.centers_z.push_back(ragdoll_r_hand_pose.pos.z);
          this->pub_ragdollcog_array_.rotation_x.push_back(ragdoll_r_hand_pose.rot.x);
          this->pub_ragdollcog_array_.rotation_y.push_back(ragdoll_r_hand_pose.rot.y);
          this->pub_ragdollcog_array_.rotation_z.push_back(ragdoll_r_hand_pose.rot.z);
          this->pub_ragdollcog_array_.rotation_w.push_back(ragdoll_r_hand_pose.rot.w);

	}

	else if (modelName.find("crona") != std::string::npos){

	  links_ = models_[i]->GetLinks();
	  for (unsigned int j = 0; j < links_.size(); j++){
	    std::string linkName = links_[j]->GetName();
	    if (linkName.find("forearm") != std::string::npos){
	      
	      obs_pose = links_[j]->GetWorldPose();
	      this->pub_object_array_.frame_names.push_back(linkName); // not correct
              this->pub_object_array_.centers_x.push_back(obs_pose.pos.x);
              this->pub_object_array_.centers_y.push_back(obs_pose.pos.y);
              this->pub_object_array_.centers_z.push_back(obs_pose.pos.z);
              this->pub_object_array_.rotation_x.push_back(obs_pose.rot.x);
              this->pub_object_array_.rotation_y.push_back(obs_pose.rot.y);
              this->pub_object_array_.rotation_z.push_back(obs_pose.rot.z);
              this->pub_object_array_.rotation_w.push_back(obs_pose.rot.w);
		
	      obs_pose = links_[j]->GetWorldCoGPose();
              this->pub_objectcog_array_.frame_names.push_back(linkName); // not correct
              this->pub_objectcog_array_.centers_x.push_back(obs_pose.pos.x);
              this->pub_objectcog_array_.centers_y.push_back(obs_pose.pos.y);
              this->pub_objectcog_array_.centers_z.push_back(obs_pose.pos.z);
              this->pub_objectcog_array_.rotation_x.push_back(obs_pose.rot.x);
              this->pub_objectcog_array_.rotation_y.push_back(obs_pose.rot.y);
              this->pub_objectcog_array_.rotation_z.push_back(obs_pose.rot.z);
              this->pub_objectcog_array_.rotation_w.push_back(obs_pose.rot.w);
	    
	      crona_forces_at_forearm = links_[j]->GetWorldForce();
	      crona_torques_at_forearm = links_[j]->GetWorldTorque();
	      this->pub_cronaft_array_.frame_names.push_back(linkName);
	      this->pub_cronaft_array_.force_x.push_back(crona_forces_at_forearm.x);
	      this->pub_cronaft_array_.force_y.push_back(crona_forces_at_forearm.y);
	      this->pub_cronaft_array_.force_z.push_back(crona_forces_at_forearm.z);
	      this->pub_cronaft_array_.torque_x.push_back(crona_torques_at_forearm.x);
	      this->pub_cronaft_array_.torque_y.push_back(crona_torques_at_forearm.y);
	      this->pub_cronaft_array_.torque_z.push_back(crona_torques_at_forearm.z);
	    }
	  }
	  
	  joints_ = models_[i]->GetJoints(); 
	  for (unsigned int j = 0; j < joints_.size(); j++){
	    std::string jointName = joints_[j]->GetName();
            if (jointName.find("l_shoulder") != std::string::npos || jointName.find("l_forearm") != std::string::npos || jointName.find("l_elbow")!= std::string::npos || jointName.find("l_hand") != std::string::npos){
	      crona_jt = joints_[j]->GetForceTorque(0);
	      crona_forces = crona_jt.body2Force;
	      crona_torques = crona_jt.body2Torque;
	      this->pub_cronajt_array_.frame_names.push_back(jointName);
              this->pub_cronajt_array_.force_x.push_back(crona_forces.x);
              this->pub_cronajt_array_.force_y.push_back(crona_forces.y);
              this->pub_cronajt_array_.force_z.push_back(crona_forces.z);
              this->pub_cronajt_array_.torque_x.push_back(crona_torques.x);
              this->pub_cronajt_array_.torque_y.push_back(crona_torques.y);
              this->pub_cronajt_array_.torque_z.push_back(crona_torques.z);
	    }
	    //if (jointName.find("torso") != std::string::npos){
	    //  obs_pose = joints_[j]->anchorPose
	    //  gzdbg << "torso joint pose: " << obs_pose << "\n";
	    //}
          } 
	  
	  if (crona_wrench_msg_available == 1){
            link_ = models_[i]->GetLink("l_forearm_roll_link");
            math::Vector3 rel_pos = math::Vector3(0.,0.,0.209);
	    //force_vector = math::Vector3(0,0,1);
            link_->AddForceAtRelativePosition(crona_larm_force_vector,rel_pos);
	    link_ = models_[i]->GetLink("r_forearm_roll_link");
            rel_pos = math::Vector3(0.,0.,0.209);
            //force_vector = math::Vector3(0,0,1);
            link_->AddForceAtRelativePosition(crona_rarm_force_vector,rel_pos);
	  }

	}

   	else if (modelName.find("link1") != std::string::npos){

          links_ = models_[i]->GetLinks();
          obs_pose = links_[0]->GetWorldCoGPose();
          //this->pub_objectcog_array_.frame_names.push_back("small_lift_box"); 
          this->pub_objectcog_array_.frame_names.push_back("link1");
          this->pub_objectcog_array_.centers_x.push_back(obs_pose.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(obs_pose.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(obs_pose.pos.z);
          this->pub_objectcog_array_.rotation_x.push_back(obs_pose.rot.x);
          this->pub_objectcog_array_.rotation_y.push_back(obs_pose.rot.y);
          this->pub_objectcog_array_.rotation_z.push_back(obs_pose.rot.z);
          this->pub_objectcog_array_.rotation_w.push_back(obs_pose.rot.w);

          obs_pose = links_[0]->GetWorldPose();
          //this->pub_object_array_.frame_names.push_back("small_lift_box"); 
          this->pub_object_array_.frame_names.push_back("link1");
          this->pub_object_array_.centers_x.push_back(obs_pose.pos.x);
          this->pub_object_array_.centers_y.push_back(obs_pose.pos.y);
          this->pub_object_array_.centers_z.push_back(obs_pose.pos.z);
          this->pub_object_array_.rotation_x.push_back(obs_pose.rot.x);
          this->pub_object_array_.rotation_y.push_back(obs_pose.rot.y);
          this->pub_object_array_.rotation_z.push_back(obs_pose.rot.z);
          this->pub_object_array_.rotation_w.push_back(obs_pose.rot.w);

      }

 	else if (modelName.find("lift_box") != std::string::npos){

          links_ = models_[i]->GetLinks();
	  obs_pose = links_[0]->GetWorldCoGPose();
          this->pub_objectcog_array_.frame_names.push_back("small_lift_box"); 
          this->pub_objectcog_array_.centers_x.push_back(obs_pose.pos.x);
          this->pub_objectcog_array_.centers_y.push_back(obs_pose.pos.y);
          this->pub_objectcog_array_.centers_z.push_back(obs_pose.pos.z);
          this->pub_objectcog_array_.rotation_x.push_back(obs_pose.rot.x);
          this->pub_objectcog_array_.rotation_y.push_back(obs_pose.rot.y);
          this->pub_objectcog_array_.rotation_z.push_back(obs_pose.rot.z);
          this->pub_objectcog_array_.rotation_w.push_back(obs_pose.rot.w);
	  
	  obs_pose = links_[0]->GetWorldPose();
          this->pub_object_array_.frame_names.push_back("small_lift_box"); 
          this->pub_object_array_.centers_x.push_back(obs_pose.pos.x);
          this->pub_object_array_.centers_y.push_back(obs_pose.pos.y);
          this->pub_object_array_.centers_z.push_back(obs_pose.pos.z);
          this->pub_object_array_.rotation_x.push_back(obs_pose.rot.x);
          this->pub_object_array_.rotation_y.push_back(obs_pose.rot.y);
          this->pub_object_array_.rotation_z.push_back(obs_pose.rot.z);
          this->pub_object_array_.rotation_w.push_back(obs_pose.rot.w);

	  object_forces = links_[0]->GetWorldCoGLinearVel();
          object_torques = links_[0]->GetWorldAngularAccel();
	  //gzdbg << object_forces << "\n";
          this->pub_liftboxft_array_.frame_names.push_back("small_lift_box");
          this->pub_liftboxft_array_.force_x.push_back(object_forces.x);
          this->pub_liftboxft_array_.force_y.push_back(object_forces.y);
          this->pub_liftboxft_array_.force_z.push_back(object_forces.z);
          this->pub_liftboxft_array_.torque_x.push_back(object_torques.x);
          this->pub_liftboxft_array_.torque_y.push_back(object_torques.y);
          this->pub_liftboxft_array_.torque_z.push_back(object_torques.z);
	
          if (object_wrench_msg_available == 1){
            link_ = models_[i]->GetLink("small_lift_box");
            math::Vector3 rel_pos = math::Vector3(0.,0.,0.);
            link_->AddForceAtRelativePosition(object_force_vector,rel_pos);
          }

      }
      }
	
      //  publish object to ros
      this->pub_object_.publish(this->pub_object_array_);
      this->pub_objectcog_.publish(this->pub_objectcog_array_);
      this->pub_objectft_.publish(this->pub_objectft_array_);
      this->pub_cronaft_.publish(this->pub_cronaft_array_);
      this->pub_cronajt_.publish(this->pub_cronajt_array_);
      this->pub_liftboxft_.publish(this->pub_liftboxft_array_);
      this->pub_ragdollcog_.publish(this->pub_ragdollcog_array_);

      this->pub_ragdollcog_array_.frame_names.clear();
      this->pub_ragdollcog_array_.centers_x.clear();
      this->pub_ragdollcog_array_.centers_y.clear();
      this->pub_ragdollcog_array_.centers_z.clear();
      this->pub_ragdollcog_array_.rotation_x.clear();
      this->pub_ragdollcog_array_.rotation_y.clear();
      this->pub_ragdollcog_array_.rotation_z.clear();
      this->pub_ragdollcog_array_.rotation_w.clear();
      
      this->pub_object_array_.frame_names.clear();
      this->pub_object_array_.centers_x.clear();
      this->pub_object_array_.centers_y.clear();
      this->pub_object_array_.centers_z.clear();
      this->pub_object_array_.rotation_x.clear();
      this->pub_object_array_.rotation_y.clear();
      this->pub_object_array_.rotation_z.clear();
      this->pub_object_array_.rotation_w.clear();

      this->pub_objectcog_array_.frame_names.clear();
      this->pub_objectcog_array_.mass.clear();
      this->pub_objectcog_array_.centers_x.clear();
      this->pub_objectcog_array_.centers_y.clear();
      this->pub_objectcog_array_.centers_z.clear();
      this->pub_objectcog_array_.rotation_x.clear();
      this->pub_objectcog_array_.rotation_y.clear();
      this->pub_objectcog_array_.rotation_z.clear();
      this->pub_objectcog_array_.rotation_w.clear();

      this->pub_objectft_array_.frame_names.clear();
      this->pub_objectft_array_.force_x.clear();
      this->pub_objectft_array_.force_y.clear();
      this->pub_objectft_array_.force_z.clear();
      this->pub_objectft_array_.torque_x.clear();
      this->pub_objectft_array_.torque_y.clear();
      this->pub_objectft_array_.torque_z.clear();
    
      this->pub_cronaft_array_.frame_names.clear();
      this->pub_cronaft_array_.force_x.clear();
      this->pub_cronaft_array_.force_y.clear();
      this->pub_cronaft_array_.force_z.clear();
      this->pub_cronaft_array_.torque_x.clear();
      this->pub_cronaft_array_.torque_y.clear();
      this->pub_cronaft_array_.torque_z.clear();	

      this->pub_cronajt_array_.frame_names.clear();
      this->pub_cronajt_array_.force_x.clear();
      this->pub_cronajt_array_.force_y.clear();
      this->pub_cronajt_array_.force_z.clear();
      this->pub_cronajt_array_.torque_x.clear();
      this->pub_cronajt_array_.torque_y.clear();
      this->pub_cronajt_array_.torque_z.clear();
 
      this->pub_liftboxft_array_.frame_names.clear();
      this->pub_liftboxft_array_.force_x.clear();
      this->pub_liftboxft_array_.force_y.clear();
      this->pub_liftboxft_array_.force_z.clear();
      this->pub_liftboxft_array_.torque_x.clear();
      this->pub_liftboxft_array_.torque_y.clear();
      this->pub_liftboxft_array_.torque_z.clear();

      this->lock_.unlock();
    }

    void MoveBoxesCallback(const std_msgs::Bool::ConstPtr& msg)
    {

      physics::Model_V models_;
      unsigned int nModel = 0;
      models_ = this->world->GetModels();
      nModel = this->world->GetModelCount();
      math::Pose obs_pose;     
 
      if (msg->data == true){
        for (unsigned int i = 0; i < nModel; i++){
          std::string modelName = models_[i]->GetName();
          if (modelName.find("box_1") != std::string::npos){
              if ((this->pub_objectcog_array_.centers_y[0]-this->pub_objectcog_array_.centers_y[1])>0.7){
                  //obs_pose.pos.x = srv.response.centers_x[0];
                  obs_pose.pos.x = 4;
                  obs_pose.pos.y = (this->pub_objectcog_array_.centers_y[0]+this->pub_objectcog_array_.centers_y[1])/2;
              }
              else {
                  obs_pose.pos.x = 10;
                  obs_pose.pos.y = 3;
              }
	      obs_pose.pos.z = 0.25;
              obs_pose.rot.x = 0;
              obs_pose.rot.y = 0;
              obs_pose.rot.z = 0;
              obs_pose.rot.w = 0;
              models_[i]->SetWorldPose(obs_pose);
          }
	  else if (modelName.find("small_box") != std::string::npos){
              if ((this->pub_objectcog_array_.centers_y[0]-this->pub_objectcog_array_.centers_y[1])>0.7){
                  obs_pose.pos.x = 10;
                  obs_pose.pos.y = 3;
              }
              else {
                  //obs_pose.pos.x = srv.response.centers_x[j];
                  obs_pose.pos.x = 4;
                  obs_pose.pos.y = (this->pub_objectcog_array_.centers_y[0]+this->pub_objectcog_array_.centers_y[1])/2;
              }
              obs_pose.pos.z = 0.25;
              obs_pose.rot.x = 0;
              obs_pose.rot.y = 0;
              obs_pose.rot.z = 0;
              obs_pose.rot.w = 0;
              models_[i]->SetWorldPose(obs_pose);
          }
	  else if (modelName.find("box_2")!= std::string::npos){
              obs_pose.pos.x = 4;
              obs_pose.pos.y = this->pub_objectcog_array_.centers_y[0]+1.15;
              obs_pose.pos.z = 0.25;
              obs_pose.rot.x = 0;
              obs_pose.rot.y = 0;
              obs_pose.rot.z = 0;
              obs_pose.rot.w = 0;
              models_[i]->SetWorldPose(obs_pose);
          }
	  else if (modelName.find("box_3")!= std::string::npos){
              obs_pose.pos.x = 4;
              obs_pose.pos.y = this->pub_objectcog_array_.centers_y[1]-1.15;
              obs_pose.pos.z = 0.25;
              obs_pose.rot.x = 0;
              obs_pose.rot.y = 0;
              obs_pose.rot.z = 0;
              obs_pose.rot.w = 0;
              models_[i]->SetWorldPose(obs_pose);
          }
      }
      }	
      else if (msg->data == false){
        for (unsigned int i = 0; i < nModel; i++){
          std::string modelName = models_[i]->GetName();
          if (modelName.find("box_1") != std::string::npos){
              obs_pose.pos.x = 10;
              obs_pose.pos.y = 0;
              obs_pose.pos.z = 0.25;
              obs_pose.rot.x = 0;
              obs_pose.rot.y = 0;
              obs_pose.rot.z = 0;
              obs_pose.rot.w = 0;
	      models_[i]->SetWorldPose(obs_pose);
          }
          else if (modelName.find("small_box") != std::string::npos){
              obs_pose.pos.x = 10;
              obs_pose.pos.y = 0;
              obs_pose.pos.z = 0.25;
              obs_pose.rot.x = 0;
              obs_pose.rot.y = 0;
              obs_pose.rot.z = 0;
              obs_pose.rot.w = 0;
              models_[i]->SetWorldPose(obs_pose);
          } 
	  else if (modelName.find("box_2")!= std::string::npos){
              obs_pose.pos.x = 10;
              obs_pose.pos.y = 1.4;
              obs_pose.pos.z = 0.25;
              obs_pose.rot.x = 0;
              obs_pose.rot.y = 0;
              obs_pose.rot.z = 0;
              obs_pose.rot.w = 0;
              models_[i]->SetWorldPose(obs_pose);
          }
	  else if (modelName.find("box_3")!= std::string::npos){
              obs_pose.pos.x = 10;
              obs_pose.pos.y = -1.4;
              obs_pose.pos.z = 0.25;
              obs_pose.rot.x = 0;
              obs_pose.rot.y = 0;
              obs_pose.rot.z = 0;
              obs_pose.rot.w = 0;
              models_[i]->SetWorldPose(obs_pose);
          }
	}
    }
    }

    void SetRigidCallback(const std_msgs::Bool::ConstPtr& msg)
    {
      if (msg->data == true){
	ROS_INFO("Set ragdoll joints to zero");
	physics::ModelPtr ragdoll_;
        ragdoll_ = this->world->GetModel("ragdoll");
        physics::Joint_V joints_;
        math::Vector3 zero_vector(0,0,0);
        unsigned int njoints = 0;
        double _angle = 0;
        joints_ = ragdoll_->GetJoints();
        njoints= ragdoll_->GetJointCount();
	for (unsigned int i=0; i < njoints; i++){
	  gzdbg << "Angle: " << joints_[i]->GetAngle(0) << "\n"; 
	  joints_[i]->SetAngle(0,_angle);
	  joints_[i]->Update();
          gzdbg << "Angle: " << joints_[i]->GetAngle(0) << "\n";
	}
      }
      else {
	ROS_INFO("Setting ragdoll joints to normal limits");
      }
    }


    void MoveRagdollCallback(const std_msgs::Bool::ConstPtr& msg)
    {
      if (msg->data == true){
        ROS_INFO("Set Objects");
        
        // Get initial object position
        sttr_srvs::GazeboObjectState srv;

        this->init_object_client_ = this->rosnode_->serviceClient<sttr_srvs::GazeboObjectState>("/gazebo/object_init_srv");
        this->init_object_client_.waitForExistence();
        this->init_object_client_.call(srv);

        // Set initial object position
        physics::Model_V models_;
        unsigned int nModel = 0;
        models_ = this->world->GetModels();
        nModel = this->world->GetModelCount();
	math::Pose obs_pose;
	
	// set object pose
        for (unsigned int i = 0; i < nModel; i++){
          std::string modelName = models_[i]->GetName();
          if (modelName.find("ragdoll") != std::string::npos){
            // Get current pose
            obs_pose = models_[i]->GetWorldPose();
            if (modelName.find(srv.response.frame_names[0]) != std::string::npos){
                obs_pose.pos.x = srv.response.centers_x[0];
                obs_pose.pos.y = srv.response.centers_y[0];
		obs_pose.pos.z = srv.response.centers_z[0];
		obs_pose.rot.x = srv.response.rotation_x[0];
		obs_pose.rot.y = srv.response.rotation_y[0];
		obs_pose.rot.z = srv.response.rotation_z[0];
		obs_pose.rot.w = srv.response.rotation_w[0];
            }

	    physics::Joint_V joints_;
	    physics::Link_V links_;
	    math::Vector3 zero_vector(0,0,0);
	    unsigned int njoints = 0;
	    double _angle = 0;
	    joints_ = models_[i]->GetJoints();
	    njoints= models_[i]->GetJointCount();
	    links_ = models_[i]->GetLinks();
	    models_[i]->SetWorldPose(obs_pose);
	    for (unsigned int k = 0; k < njoints; k++){
		joints_[k]->SetAngle(0,_angle);

	    }
	    for (unsigned int k = 0; k < 3; k++){
		links_[k]->SetForce(zero_vector);
		links_[k]->SetTorque(zero_vector);

            }

  	  } 
        }
	
      }
      else if (msg->data == false){
        ROS_INFO("Remove objects");
        physics::Model_V models_;
        unsigned int nModel = 0;
        models_ = this->world->GetModels();
        nModel = this->world->GetModelCount();
        math::Pose obs_pose;
        for (unsigned int i = 0; i < nModel; i++){
          std::string modelName = models_[i]->GetName();
          if (modelName.find("ragdoll") != std::string::npos){
            obs_pose = models_[i]->GetWorldPose();
            //obs_pose.pos.x += 2.5;
            obs_pose.pos.x -= 10;
	    obs_pose.pos.z = 0.5;

	    physics::Joint_V joints_;
	    physics::Link_V links_;
	    math::Vector3 zero_vector(0,0,0);
	    
            unsigned int njoints = 0;
            math::Angle _angle = 0;
	    double joint_vel = 0;
            joints_ = models_[i]->GetJoints();
            njoints= models_[i]->GetJointCount();
	    models_[i]->SetWorldPose(obs_pose);
            for (unsigned int k = 0; k < njoints; k++){
                joints_[k]->SetAngle(0,_angle);
	    }
	    models_[i]->Reset();
	  }
        }

      }
    } 
    
   /* 
    void ApplyObjectWrenchCallback(const sttr_msgs::WrenchArray::ConstPtr& msg)
    {
      physics::Model_V models_;
      physics::Link_V links_;
      physics::LinkPtr link_;	
      unsigned int nModel = 0;
      models_ = this->world->GetModels();
      nModel = this->world->GetModelCount();
      srand (time(NULL));
      int duration = rand() % 10 + 1;
      int k = 5;
	
      math::Pose obs_pose;

      for (unsigned int i = 0; i < nModel; i++){
      std::string modelName = models_[i]->GetName();
      if (modelName.find("ragdoll") != std::string::npos){
	links_ = models_[i]->GetLinks();
	for (unsigned int j = 0; j < msg->frame_names.size(); j++){
          link_ = models_[i]->GetLink(msg->frame_names[j]);
	  math::Vector3 torque_vector = math::Vector3(msg->torque_x[j],msg->torque_y[j],msg->torque_z[j]);
	  link_->AddRelativeTorque(torque_vector);
	  }
	  }

    }
    }
   */

    void ApplyObjectWrenchCallback(const sttr_msgs::WrenchArray::ConstPtr& msg)
    {
      if (msg->frame_names[0].find("none") == std::string::npos){
        object_force_location = msg->frame_names[0];
        object_force_vector = math::Vector3(msg->force_x[0],msg->force_y[0],msg->force_z[0]);
        object_wrench_msg_available = 1;
      }
      else {
        object_wrench_msg_available = 0;
      }
     
    }

    void ApplycRoNAWrenchCallback(const sttr_msgs::WrenchArray::ConstPtr& msg)
    {
      if (msg->frame_names[0].find("none") == std::string::npos){
        crona_force_location = msg->frame_names[0];
        crona_larm_force_vector = math::Vector3(msg->force_x[0],msg->force_y[0],msg->force_z[0]);
        crona_rarm_force_vector = math::Vector3(msg->force_x[1],msg->force_y[1],msg->force_z[1]);
        crona_wrench_msg_available = 1;
      }
      else {
	crona_wrench_msg_available = 0;
      }
     
    }

    void ApplyRagdollWrenchCallback(const sttr_msgs::WrenchArray::ConstPtr& msg)
    {
      //ragdoll_force_vector = math::Vector3(msg->force_x[0],msg->force_y[0],msg->force_z[0]);
      if (msg->frame_names[0].find("move_and_scoop") != std::string::npos){
      	ragdoll_force_vector = math::Vector3(0,0,0);
     	ragdoll_body_force_vector = math::Vector3(msg->force_x[0],4*msg->force_y[0],3*msg->force_z[0]);
      	ragdoll_wrench_msg_available = 1;
      }
      else if (msg->frame_names[0].find("") != std::string::npos){
	ragdoll_force_location = msg->frame_names[0];
	ragdoll_force_vector = math::Vector3(msg->force_x[0],msg->force_y[0],msg->force_z[0]);
	ragdoll_random_wrench_msg_available = 1;
      }
      else {
	ragdoll_wrench_msg_available = 0;
	ragdoll_random_wrench_msg_available = 0;
      }
    }
     
    
    void ApplyForearmRollerCallback(const sttr_msgs::RagdollWrenchArray::ConstPtr& msg)
    {
      physics::Model_V models_;
      unsigned int nModel = 0;
      physics::Link_V links_;

      models_ = this->world->GetModels();
      nModel = this->world->GetModelCount();
      for (unsigned int i = 0; i < nModel; i++){
        std::string modelName = models_[i]->GetName();
	if (modelName.find("ragdoll") != std::string::npos){
	  links_ = models_[i]->GetLinks();
	  for (unsigned int j = 0; j < msg->frame_names.size(); j++){ 
            for (unsigned int k = 0; k < links_.size(); k++){
	      if (msg->frame_names[j].find(links_[k]->GetName()) != std::string::npos){
	        forearm_ragdoll_roller_force_location = math::Vector3(msg->centers_x[j], msg->centers_y[j], msg->centers_z[j]);
		// For no-slip forearm roller
		//forearm_ragdoll_roller_force_vector = math::Vector3(links_[k]->GetInertial()->GetMass()*msg->force_x[j], links_[k]->GetInertial()->GetMass()*msg->force_y[j], links_[k]->GetInertial()->GetMass()*msg->force_z[j]);
		// For Charlie's idea
		ragdoll_roller_force_vector = math::Vector3(msg->force_x[j],msg->force_y[j],msg->force_z[j]);
		forearm_roller_force_vector = math::Vector3(-1*msg->force_x[j],-1*msg->force_y[j],-1*msg->force_z[j]);
	        links_[k]->AddForceAtWorldPosition(ragdoll_roller_force_vector, forearm_ragdoll_roller_force_location);
	      }
	      if (msg->frame_names[j][0] == 'l'){
                this->world->GetModel("crona2")->GetLink("l_forearm_roll_link")->AddForceAtWorldPosition(forearm_roller_force_vector, forearm_ragdoll_roller_force_location);
	      }
	      if (msg->frame_names[j][0] == 'r'){
		this->world->GetModel("crona2")->GetLink("r_forearm_roll_link")->AddForceAtWorldPosition(forearm_roller_force_vector, forearm_ragdoll_roller_force_location);
	      }
	    }
	  }
	}
      }
    }
    
    // 
    private: physics::WorldPtr world;
    private: event::ConnectionPtr time_update_event_;
    private: event::ConnectionPtr object_update_event_;

    /// \brief A mutex to lock access to fields that are used in ROS message callbacks
    private: boost::mutex lock_;

    /// \brief utilites for checking incoming string URDF/XML/Param
    bool world_created_;

    ros::NodeHandle* rosnode_;

    // ROS Publisher & Subscriber
    ros::Publisher pub_clock_;
    ros::Publisher pub_object_;
    ros::Publisher pub_objectcog_;
    ros::Publisher pub_objectft_;
    ros::Publisher pub_cronaft_;
    ros::Publisher pub_cronajt_;
    ros::Publisher pub_liftboxft_;
    ros::Publisher pub_ragdollcog_;
    //ros::Subscriber sub_object_;
    ros::Subscriber sub_move_ragdoll_;
    ros::Subscriber sub_rigid_;
    ros::Subscriber sub_move_boxes_;
    ros::Subscriber sub_object_wrench_;
    ros::Subscriber sub_crona_wrench_;
    ros::Subscriber sub_ragdoll_wrench_;
    ros::Subscriber sub_forearm_roller_accel_;
    ros::ServiceClient init_object_client_;

    // Object related variables
    math::Vector3 prev_torque_vector;
    //bool object_reset;

    // Message
    sttr_msgs::RagdollObjectArray pub_object_array_;
    sttr_msgs::RagdollObjectArray pub_objectcog_array_;
    sttr_msgs::RagdollObjectArray pub_ragdollcog_array_;
    sttr_msgs::WrenchArray pub_objectft_array_;
    sttr_msgs::WrenchArray pub_cronaft_array_;
    sttr_msgs::WrenchArray pub_cronajt_array_;
    sttr_msgs::WrenchArray pub_liftboxft_array_;

    std::string crona_force_location;
    math::Vector3 crona_larm_force_vector;
    math::Vector3 crona_rarm_force_vector;
    bool crona_wrench_msg_available;
    std::string object_force_location;
    math::Vector3 object_force_vector;
    bool object_wrench_msg_available;
    std::string ragdoll_force_location;
    math::Vector3 ragdoll_force_vector;
    math::Vector3 ragdoll_body_force_vector;
    bool ragdoll_wrench_msg_available;
    bool ragdoll_random_wrench_msg_available;
    math::Vector3 forearm_ragdoll_roller_force_location;
    math::Vector3 ragdoll_roller_force_vector;
    math::Vector3 forearm_roller_force_vector;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ROSWorldPlugin);
}



