/*
 * Sends markers to RViz to visualize the pressure
 * being applied on a taxel array sensor.
 *
 * Copyright (C) 2011  Meka Robotics
 * Author: Pierre-Luc Bacon <pierrelucbacon@mekabot.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "RosPressureVisualizer.h"
#include <m3skin_ros/None_TransformArray.h>
#include <m3skin_ros/None_String.h>

#include <visualization_msgs/MarkerArray.h>

namespace m3 {

visualization_msgs::Marker RosPressureVisualizer::GetArrowMarker(tf::Vector3 position, tf::Quaternion orientation, double scale)
{
	visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;

	m.pose.position.x = position.x();
	m.pose.position.y = position.y();
	m.pose.position.z = position.z();

	m.pose.orientation.x = orientation.x();
	m.pose.orientation.y = orientation.y();
	m.pose.orientation.z = orientation.z();
	m.pose.orientation.w = orientation.w();

	m.scale.x = scale;
	m.scale.y = SCALE_ARROW_Y;
	m.scale.z = SCALE_ARROW_Z;

	m.color.r = 1.0;
	m.color.g = 0;
	m.color.b = 0;
	m.color.a = 0.5;

	m.lifetime = ros::Duration(DURATION_ARROW);

	return m;
}

void RosPressureVisualizer::GetArrowTextMarkers(tf::Vector3 position,
                            tf::Quaternion orientation, double scale,
                            visualization_msgs::Marker *arrow,
                            visualization_msgs::Marker *text,
                            double nx, double ny, double nz)
{
  arrow->type = visualization_msgs::Marker::ARROW;
  arrow->action = visualization_msgs::Marker::ADD;
  
  arrow->pose.position.x = position.x();
  arrow->pose.position.y = position.y();
  arrow->pose.position.z = position.z();
  
  arrow->pose.orientation.x = orientation.x();
  arrow->pose.orientation.y = orientation.y();
  arrow->pose.orientation.z = orientation.z();
  arrow->pose.orientation.w = orientation.w();
  
  /*
  //   Diamondback
  arrow->scale.x = scale;
  arrow->scale.y = SCALE_ARROW_Y;
  arrow->scale.z = SCALE_ARROW_Z;
    */
  
  // Electric
  arrow->scale.x = SCALE_ARROW_Y;
  arrow->scale.y = SCALE_ARROW_Z;
  arrow->scale.z = scale;
  
  arrow->color.r = 1.0;
  arrow->color.g = 0;
  arrow->color.b = 0;
  arrow->color.a = 1.0;
  
  arrow->lifetime = ros::Duration(DURATION_ARROW);
  
  text->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text->action = visualization_msgs::Marker::ADD;
  
  text->pose.position.x = position.x() + nx * (scale + 0.04) * 1.4;
  text->pose.position.y = position.y() + ny * (scale + 0.04) * 1.4;
  text->pose.position.z = position.z() + nz * (scale + 0.04) * 1.4;
  
  text->pose.orientation.x = orientation.x();
  text->pose.orientation.y = orientation.y();
  text->pose.orientation.z = orientation.z();
  text->pose.orientation.w = orientation.w();
  
  text->scale.x = 0.03;
  text->scale.y = 0.03;
  text->scale.z = 0.03;
  
  text->color.r = 1.0;
  text->color.g = 0;
  text->color.b = 0;
  text->color.a = 1.0;
  
  text->lifetime = ros::Duration(DURATION_ARROW);
  
  char s[20];
  sprintf(s, "%.1fN", scale/SCALING_FACTOR);
  text->text = s;
    
}

void RosPressureVisualizer::DisplayPressure(
					    const hrl_haptic_manipulation_in_clutter_msgs::TaxelArray::ConstPtr& msg) {
  if (frequency_count < FREQUENCY_DIVISOR) {
    frequency_count += 1;
    return;
  }

  //ROS_INFO("Unlatching");
  
  frequency_count = 0;
  
  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::MarkerArray textMarkerArray;
  
  // Compute the norm over all taxels and find out
  // which one we should display
  int size = msg->values_z.size();
  for (int i = 0; i < size; i++) {
    double n = norm(msg->values_x[i], msg->values_y[i], msg->values_z[i]);
    double dot_prod = msg->values_x[i] * msg->normals_x[i] +		\
      msg->values_y[i] * msg->normals_y[i] +				\
      msg->values_z[i] * msg->normals_z[i];
    double sign = 1;
    if (dot_prod < 0)
      sign = -1;

    //ROS_INFO("Norm %f", n);
    if (n > MIN_PRESSURE_VALUE) {
      visualization_msgs::Marker m1, m2;
      GetArrowTextMarkers(taxels_transforms[i].getOrigin(),
			  taxels_arrows_quaternions[i],
			  n * SCALING_FACTOR * sign, &m1, &m2, msg->normals_x[i],
			  msg->normals_y[i], msg->normals_z[i]);
      m1.header.stamp = msg->header.stamp - ros::Duration(TF_TIME_OFFSET);
      m1.header.frame_id = msg->header.frame_id;
      m1.id = i;
      markerArray.markers.push_back(m1);
      
      m2.header.stamp = msg->header.stamp - ros::Duration(TF_TIME_OFFSET);
      m2.header.frame_id = msg->header.frame_id;
      m2.id = i;
      textMarkerArray.markers.push_back(m2);
      //ROS_INFO("Sending marker. Taxel %d, norm %f", i, n);
    }
    
  }
  
  publisher_marker_array.publish(markerArray);
  publisher_text_marker_array.publish(textMarkerArray);
}


void RosPressureVisualizer::GetTaxelTransforms() {
  m3skin_ros::None_TransformArray srv;

  if (local_coords_client.call(srv)) {
    ROS_INFO("Got taxels frames.");
    
    transforms = srv.response.data;
    std::vector<geometry_msgs::Transform>::iterator it;
    for (it = transforms.begin(); it != transforms.end(); it++) {
      //tf::Transform tf = GeometryTransformToTf(*it);
      tf::Transform tf;
      tf::transformMsgToTF(*it, tf);
      
      taxels_transforms.push_back(tf);
      taxels_transforms_inv.push_back(tf.inverse());
      taxels_arrows_quaternions.push_back(tf*tf::Quaternion(0, -1, 0, 1));
    }
  } else {
    ROS_ERROR("Failed to get taxels frames");
  }
}

void RosPressureVisualizer::GetLinkName() {
	m3skin_ros::None_String srv;

	if (link_name_client.call(srv)) {
		linkName = srv.response.data;

		ROS_INFO("Got link name %s", linkName.c_str());
	} else {
		ROS_ERROR("Failed to get link name");
	}
}

void RosPressureVisualizer::Init() {
    std::string local_coord_srv_name = "taxels/srv/local_coord_frames";
    std::string link_name_srv_name = "taxels/srv/link_name";

    local_coords_client = n.serviceClient<m3skin_ros::None_TransformArray> (local_coord_srv_name.c_str());
    link_name_client = n.serviceClient<m3skin_ros::None_String> (link_name_srv_name.c_str());
    
    publisher_marker_array = n.advertise<visualization_msgs::MarkerArray> ("viz/taxel_array_array", 1000);
    
    publisher_text_marker_array = n.advertise<visualization_msgs::MarkerArray> (
										"viz/taxel_array_text_array", 1000);
    
    subscriber = n.subscribe<hrl_haptic_manipulation_in_clutter_msgs::TaxelArray> (
										   "taxels/forces", 1000,
										   &RosPressureVisualizer::DisplayPressure, this);
    
    ROS_INFO("Waiting for services");
    ros::service::waitForService(local_coord_srv_name.c_str());
    ros::service::waitForService(link_name_srv_name.c_str());
    ROS_INFO("Done waiting");

    GetTaxelTransforms();
    GetLinkName();

    frequency_count = 0;
}


RosPressureVisualizer::RosPressureVisualizer() {

}

RosPressureVisualizer::~RosPressureVisualizer() {
	// TODO Auto-generated destructor stub
}

}
