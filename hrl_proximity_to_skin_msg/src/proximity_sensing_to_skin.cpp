/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

//\Author Marc Killpack, Healthcare Robotics Lab 
//(derived from patches_tf.cpp by Adrian Funk at Bosch LLC)

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>
#include "hrl_haptic_manipulation_in_clutter_msgs/TaxelArray.h"
//#include <m3skin_ros/TaxelArray.h>
#include <proximity_sensor_driver/prox_sensor_measurement.h>

//#include "proximity_to_skin.h"
using namespace std;


boost::mutex m;
int patches = 0;
vector<float> sensor_measurements;

// Struct for keeping position vector and orientation quaternion internally
typedef struct
{
  tf::Vector3 position;
  tf::Vector3 nrml;
  string parent_frame;
} patch_pos_and_nrml;

template <class T>
inline void readParam( const ros::NodeHandle &nh, const std::string &str_name, T &rhs )
{
  bool check = false;
  int counter = 0;
  while ( check == false and counter < 10 )
    {
      if( nh.getParam(str_name, rhs) )
	{
	  check = true;
	}
      else
	{
	  counter++;
	  sleep(0.1);
	}
    }

  if (check == false)
    {
      ROS_ERROR("Could not find parameter %s", str_name.c_str());
      exit(1);
    }
  return;
}


void callback(const boost::shared_ptr<proximity_sensor_driver::prox_sensor_measurement const> &msg )
{
  m.lock();
  for( int int_current_patch = 0; int_current_patch < patches; int_current_patch++ )
  {
    if (abs((float)(msg->offset[int_current_patch]) - (float)(msg->value[int_current_patch])) < 3)
      {
	sensor_measurements[int_current_patch] = .33;
      }
    else
      {
	float diff = (float)(msg->offset[int_current_patch]) - (float)(msg->value[int_current_patch]);
	sensor_measurements[int_current_patch] = 1.0/diff ;
      }
    //sensor_measurements[int_current_patch] = (float)msg->offset[int_current_patch] - (float)msg->value[int_current_patch];
  }
  m.unlock();
  return;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "proximity_sensing_to_skin");
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  ros::Publisher forearm_taxel_pub = n.advertise<hrl_haptic_manipulation_in_clutter_msgs::TaxelArray>("/skin/bosch/forearm_taxel_array", 100);
  ros::Publisher upperarm_taxel_pub = n.advertise<hrl_haptic_manipulation_in_clutter_msgs::TaxelArray>("/skin/bosch/upperarm_taxel_array", 100);

  ROS_INFO("Before getting patch normals and positions ... \n");

  vector<patch_pos_and_nrml> patch_array;
  readParam( nh_private, "/prox_sensor_driver/sensor_count", patches );
  sensor_measurements.resize( patches );

  // Fill up data structure for each patch
  patch_pos_and_nrml tmp;
  for( int current_patch = 0; current_patch < patches; current_patch++ )
  {
    // Read the parameters
    double x_pos, y_pos, z_pos;
    double x_ori, y_ori, z_ori;

    string patch_domain( "patch_" + boost::lexical_cast<string>( current_patch ) );

    readParam( nh_private, patch_domain + "/x_pos", x_pos );
    readParam( nh_private, patch_domain + "/y_pos", y_pos );
    readParam( nh_private, patch_domain + "/z_pos", z_pos );

    readParam( nh_private, patch_domain + "/x_ori", x_ori );
    readParam( nh_private, patch_domain + "/y_ori", y_ori );
    readParam( nh_private, patch_domain + "/z_ori", z_ori );

    readParam( nh_private, patch_domain + "/string_parent_frame", tmp.parent_frame );


    tf::Vector3 patch_center( x_pos, y_pos, z_pos);
    tf::Vector3 patch_normal( x_ori, y_ori, z_ori);

    if( patch_normal.isZero() )
    {
      ROS_ERROR("Normal of patch %d is zero vector!", current_patch);
      exit(1);
    }

    patch_normal.normalize();

    // Save data results for that we don't have to compute them before every transform
    tmp.position = patch_center;
    tmp.nrml = patch_normal;
    patch_array.push_back( tmp );
  }

  ROS_INFO("Got all patch normals and positions ... now publishing the skin data \n");
  

  // name of callback and object -> &class::callback, &object
  ros::Subscriber sub1 = n.subscribe("/prox_sensor_data", 1000, callback);  
  hrl_haptic_manipulation_in_clutter_msgs::TaxelArray taxel_forearm;      
  hrl_haptic_manipulation_in_clutter_msgs::TaxelArray taxel_upperarm;      
  ros::Rate r(5);
  while (ros::ok())
    {
      for( int current_patch = 0; current_patch < patches; current_patch++ )
	{
	  

	  taxel_forearm.header.frame_id = "/l_forearm_link";
	  taxel_forearm.header.stamp = ros::Time::now();
	  taxel_forearm.sensor_type = "distance";

	  taxel_upperarm.header.frame_id = "/l_upper_arm_link";
	  taxel_upperarm.header.stamp = ros::Time::now();
	  taxel_upperarm.sensor_type = "distance";

	  patch_pos_and_nrml patch = patch_array.at( current_patch );

	  //cerr<<"parent frame is :\t" << patch.parent_frame << endl;

	  if (patch.parent_frame == "l_forearm_link")
	    {
	      taxel_forearm.centers_x.push_back(patch.position[0]);
	      taxel_forearm.centers_y.push_back(patch.position[1]);
	      taxel_forearm.centers_z.push_back(patch.position[2]);

	      taxel_forearm.normals_x.push_back(patch.nrml[0]);
	      taxel_forearm.normals_y.push_back(patch.nrml[1]);
	      taxel_forearm.normals_z.push_back(patch.nrml[2]);

	      taxel_forearm.values_x.push_back(patch.nrml[0]*sensor_measurements[current_patch]);
	      taxel_forearm.values_y.push_back(patch.nrml[1]*sensor_measurements[current_patch]);
	      taxel_forearm.values_z.push_back(patch.nrml[2]*sensor_measurements[current_patch]);

	      taxel_forearm.link_names.push_back(patch.parent_frame);
	    }
	  else if (patch.parent_frame == "l_upper_arm_link")
	    {
	      taxel_upperarm.centers_x.push_back(patch.position[0]);
	      taxel_upperarm.centers_y.push_back(patch.position[1]);
	      taxel_upperarm.centers_z.push_back(patch.position[2]);

	      taxel_upperarm.normals_x.push_back(patch.nrml[0]);
	      taxel_upperarm.normals_y.push_back(patch.nrml[1]);
	      taxel_upperarm.normals_z.push_back(patch.nrml[2]);

	      taxel_upperarm.values_x.push_back(patch.nrml[0]*sensor_measurements[current_patch]);
	      taxel_upperarm.values_y.push_back(patch.nrml[1]*sensor_measurements[current_patch]);
	      taxel_upperarm.values_z.push_back(patch.nrml[2]*sensor_measurements[current_patch]);

	      taxel_upperarm.link_names.push_back(patch.parent_frame);
	    }
	  else
	    {
	      ROS_INFO("patch parent frame doesn't match taxels that we are aware of ... \n");
	      exit(1);
	    }
   
	}

      forearm_taxel_pub.publish(taxel_forearm);
      upperarm_taxel_pub.publish(taxel_upperarm);
      //publish

      taxel_forearm.centers_x.clear();
      taxel_forearm.centers_y.clear();
      taxel_forearm.centers_z.clear();

      taxel_forearm.normals_x.clear();
      taxel_forearm.normals_y.clear();
      taxel_forearm.normals_z.clear();

      taxel_forearm.values_x.clear();
      taxel_forearm.values_y.clear();
      taxel_forearm.values_z.clear();

      taxel_forearm.link_names.clear();

      taxel_upperarm.centers_x.clear();
      taxel_upperarm.centers_y.clear();
      taxel_upperarm.centers_z.clear();

      taxel_upperarm.normals_x.clear();
      taxel_upperarm.normals_y.clear();
      taxel_upperarm.normals_z.clear();

      taxel_upperarm.values_x.clear();
      taxel_upperarm.values_y.clear();
      taxel_upperarm.values_z.clear();

      taxel_upperarm.link_names.clear();

      ros::spinOnce();

      r.sleep();
    }

  return(1);
}

