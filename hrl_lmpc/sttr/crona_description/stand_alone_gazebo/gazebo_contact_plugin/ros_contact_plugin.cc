#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/World.hh>
#include <physics/physics.hh>
#include <sensors/sensors.hh>
#include <sensors/ContactSensor.hh>
#include <math/Pose.hh>
#include <math/Quaternion.hh>
#include <math/Vector3.hh>
#include <common/common.hh>
#include <stdio.h>
#include <boost/thread/mutex.hpp>
#include "sttr_msgs/RagdollObjectArray.h"

#include "ros/ros.h"
#include "hrl_haptic_manipulation_in_clutter_msgs/TaxelArray.h"


namespace gazebo
{   
  class ROSContactPlugin : public SensorPlugin
  {

    public: ROSContactPlugin()
    {
      // Start up ROS
      std::string name = "ros_contact_plugin_node";
      int argc = 0;
      //ros::init(argc, NULL, name);

    }
    public: ~ROSContactPlugin()
    {
    }

    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {   
      // Start up ROS
      if (!ros::isInitialized()){
        std::string name = "gazebo_ros_plugin_node";
        int argc = 0;
        ros::init(argc, NULL, name);
      }
      else{
        ROS_ERROR("ROS Contact Plugin>> Something other than this ros_world plugin started ros::init(...), clock may not be published properly.");
      }
  
      // Get the parent sensor.
      this->parentSensor =
        boost::shared_dynamic_cast<sensors::ContactSensor>(_sensor);

      // Make sure the parent sensor is valid.
      if (!this->parentSensor)
        { 
          gzerr << "ROSContactPlugin requires a ContactSensor.\n";
          return;
        } 

      // Get the parent world.
      std::string worldName = this->parentSensor->GetWorldName();
      this->parentWorld = physics::get_world(worldName);

      // ROS Nodehandle
      this->rosnode_ = new ros::NodeHandle("ROSContactNode");

      // ROS Topic Base Name
      std::string topicName = "/";
      topicName.append(this->parentSensor->GetName());
      topicName.append("/contact");

      // ROS Publisher

      this->pub_ragdoll_contact_ = this->rosnode_->advertise<sttr_msgs::RagdollObjectArray>(topicName, 100);

      // Frame Initialize
      // get the contact pose reference frame name
      this->frame_name_ = "world";

      // Connect to the sensor update event.
      this->updateConnection = this->parentSensor->ConnectUpdated(
      	            boost::bind(&ROSContactPlugin::OnUpdate, this));

      // preset myFrame to NULL, will search for the body with matching name in UpdateChild()
      // since most bodies are constructed on the fly
      //this->refFrame = NULL;

      // Make sure the parent sensor is active.
      this->parentSensor->SetActive(true);      
    }


    // Called by the world update start event
    public: void OnUpdate()
    {
      // Get all the contacts.
      msgs::Contacts contacts;
      math::Pose contact_pose;
      std::string prev_contact;
      unsigned int count = 0;
      contact_pose.pos.x = 0;
      contact_pose.pos.y = 0;
      contact_pose.pos.z = 0;
      contacts = this->parentSensor->GetContacts();

      for (unsigned int i = 0; i < contacts.contact_size(); ++i){
          for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j){
	      if ((contacts.contact(i).collision1().find("ground") == std::string::npos) && (contacts.contact(i).collision2().find("ground") == std::string::npos)){
	          this->pub_ragdoll_contact_array_.centers_x.push_back(contacts.contact(i).position(j).x());
	          this->pub_ragdoll_contact_array_.centers_y.push_back(contacts.contact(i).position(j).y());
	      	  this->pub_ragdoll_contact_array_.centers_z.push_back(contacts.contact(i).position(j).z());
	          if ((contacts.contact(i).collision1().find("forearm") != std::string::npos) || (contacts.contact(i).collision1().find("hand") != std::string::npos)){
                      this->pub_ragdoll_contact_array_.frame_names.push_back(contacts.contact(i).collision2());
                  }
                  else {
                      this->pub_ragdoll_contact_array_.frame_names.push_back(contacts.contact(i).collision1());
                  };
	      }
	  }
      }
      this->pub_ragdoll_contact_.publish(this->pub_ragdoll_contact_array_);
      this->pub_ragdoll_contact_array_.frame_names.clear();
      this->pub_ragdoll_contact_array_.centers_x.clear();
      this->pub_ragdoll_contact_array_.centers_y.clear();
      this->pub_ragdoll_contact_array_.centers_z.clear();

      /*
      if (contacts.contact_size() > 0){
          if (contacts.contact(0).collision1().find("forearm") != std::string::npos){
              prev_contact = contacts.contact(0).collision2();
          }
      	  else {
              prev_contact = contacts.contact(0).collision1();
          };
      }
      for (unsigned int i = 0; i < contacts.contact_size(); ++i)
      {
    	  gzdbg << i << "  Collision between[" << contacts.contact(i).collision1()
                    << "] and [" << contacts.contact(i).collision2() << "]\n";
	  if (contacts.contact(i).collision1().find(prev_contact) != std::string::npos || contacts.contact(i).collision2().find(prev_contact) != std::string::npos || this->pub_ragdoll_contact_array_.frame_names.find(contacts.contact(i).collision1()) != std::string::npos || this->pub_ragdoll_contact_array_.frame_names.find(contacts.contact(i).collision2()) != std::string::npos){
	  }
	  else {
	      contact_pose.pos.x = contact_pose.pos.x/(count);
              contact_pose.pos.y = contact_pose.pos.y/(count);
              contact_pose.pos.z = contact_pose.pos.z/(count);
              if (contacts.contact(i).collision1().find("forearm") != std::string::npos){
                  this->pub_ragdoll_contact_array_.frame_names.push_back(contacts.contact(i).collision2());
              }
              else {
                  this->pub_ragdoll_contact_array_.frame_names.push_back(contacts.contact(i).collision1());
              }
              this->pub_ragdoll_contact_array_.centers_x.push_back(contact_pose.pos.x);
              this->pub_ragdoll_contact_array_.centers_y.push_back(contact_pose.pos.y);
              this->pub_ragdoll_contact_array_.centers_z.push_back(contact_pose.pos.z);
	      contact_pose.pos.x = 0;
	      contact_pose.pos.y = 0;
	      contact_pose.pos.z = 0;
	      count = 0;
	  }
	  for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
   	  {
      	      gzdbg << j << "  Position:"
                        << contacts.contact(i).position(j).x() << " "
                	<< contacts.contact(i).position(j).y() << " "
                	<< contacts.contact(i).position(j).z() << "\n";
      	      gzdbg << "   Normal:"
                	<< contacts.contact(i).normal(j).x() << " "
                	<< contacts.contact(i).normal(j).y() << " "
                	<< contacts.contact(i).normal(j).z() << "\n";
      	      gzdbg << "   Depth:" << contacts.contact(i).depth(j) << "\n";
	      contact_pose.pos.x = contact_pose.pos.x+contacts.contact(i).position(j).x();
	      contact_pose.pos.y = contact_pose.pos.y+contacts.contact(i).position(j).y();
	      contact_pose.pos.z = contact_pose.pos.z+contacts.contact(i).position(j).z();
	      ++count;
	      gzdbg << "Feedback: " << contact_pose.pos.x << count << "\n";
    	  }
          if (contacts.contact(i).collision1().find("forearm") != std::string::npos){
              prev_contact = contacts.contact(i).collision2();
          }
          else {
              prev_contact = contacts.contact(i).collision1();
          };
      }
      if (contacts.contact_size() > 0){
          contact_pose.pos.x = contact_pose.pos.x/(count);
          contact_pose.pos.y = contact_pose.pos.y/(count);
          contact_pose.pos.z = contact_pose.pos.z/(count);
          if (contacts.contact(contacts.contact_size()-1).collision1().find("forearm") != std::string::npos){
              this->pub_ragdoll_contact_array_.frame_names.push_back(contacts.contact(contacts.contact_size()-1).collision2());
          }
          else {
              this->pub_ragdoll_contact_array_.frame_names.push_back(contacts.contact(contacts.contact_size()-1).collision1());
          }
          this->pub_ragdoll_contact_array_.centers_x.push_back(contact_pose.pos.x);
          this->pub_ragdoll_contact_array_.centers_y.push_back(contact_pose.pos.y);
          this->pub_ragdoll_contact_array_.centers_z.push_back(contact_pose.pos.z);
          this->pub_ragdoll_contact_.publish(this->pub_ragdoll_contact_array_);
          this->pub_ragdoll_contact_array_.frame_names.clear();
          this->pub_ragdoll_contact_array_.centers_x.clear();
          this->pub_ragdoll_contact_array_.centers_y.clear();
          this->pub_ragdoll_contact_array_.centers_z.clear();
      }
      */
    }

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* rosnode_;

    // ROS Publisher
    ros::Publisher pub;
    ros::Publisher pub_ragdoll_contact_;

    // Frame conversion
    private: std::string frame_id;
    private: std::string frame_name_;

    // World
    private: physics::WorldPtr parentWorld;

    /// Mutex to protect updates.
    private: boost::mutex mutex;

    sttr_msgs::RagdollObjectArray pub_ragdoll_contact_array_;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(ROSContactPlugin)
}

