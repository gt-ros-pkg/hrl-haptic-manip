/*
 * M3 Component to abstract PDO over serial (rather than ethercat)
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

#include "RosTaxelsTransformNode.h"
#include <m3skin_ros/None_TransformArray.h>
#include <m3skin_ros/None_String.h>
#include <math.h>
namespace m3 {

double RosTaxelsTransformNode::GetUnbiased(unsigned value, unsigned index)
{
    double unbiased = (((double) value) - ((double)biases[index]))/(65535.0 - ((double) biases[index]));
    if (unbiased < 0.0) {
    	unbiased = 0.0;
    }

    //ROS_INFO("Before : %d After %f Bias %d In Newtons %f", value, unbiased, biases[index], unbiased*MAX_VALUE);

    return unbiased;
}

m3skin_ros::RawTaxelArray RosTaxelsTransformNode::FilterRaw(const m3skin_ros::RawTaxelArray::ConstPtr& msg)
{
	m3skin_ros::RawTaxelArray outMsg;
	outMsg.val_x = msg->val_x;
	outMsg.val_y = msg->val_y;

	std::size_t n = msg->val_z.size();
	for (unsigned i = 0; i < n; i++) {
		outMsg.val_z.push_back(GetUnbiased(msg->val_z[i], i)*MAX_VALUE);
	}

	return outMsg;
}

void RosTaxelsTransformNode::TransformMessage(
		const m3skin_ros::RawTaxelArray::ConstPtr& msg) {

	if (IsCalibrateMode()) {
		CalibrateBias(msg->val_z);
		return;
	}

	m3skin_ros::RawTaxelArray filteredMsg = FilterRaw(msg);
	std::size_t n = filteredMsg.val_z.size();

	hrl_haptic_manipulation_in_clutter_msgs::TaxelArray output_msg;
	// Header
	output_msg.header.frame_id = linkName;
	output_msg.header.stamp = ros::Time::now();

	// Body
	for (unsigned i = 0; i < n; i++) {
		if (i >= fullTfs.size()) {
			ROS_ERROR("No transform for taxel %d", i);
			break;
		}
		tf::Vector3 center = centers[i];
		tf::Vector3 normal = normals[i];
		tf::Vector3 force = fullTfs[i](tf::Vector3(0, 0, filteredMsg.val_z[i]));

		output_msg.centers_x.push_back(center.getX());
		output_msg.centers_y.push_back(center.getY());
		output_msg.centers_z.push_back(center.getZ());

		output_msg.normals_x.push_back(normal.getX());
		output_msg.normals_y.push_back(normal.getY());
		output_msg.normals_z.push_back(normal.getZ());

		output_msg.values_x.push_back(force.getX());
		output_msg.values_y.push_back(force.getY());
		output_msg.values_z.push_back(force.getZ());
	}

	publisher.publish(output_msg);
}

tf::Transform RosTaxelsTransformNode::GeometryTransformToTf(geometry_msgs::Transform& gtf) {
	tf::Transform tf;

	tf.setOrigin(tf::Vector3(gtf.translation.x,
			gtf.translation.y,
			gtf.translation.z));

	tf.setRotation(tf::Quaternion(gtf.rotation.x,
			gtf.rotation.y,
			gtf.rotation.z,
			gtf.rotation.w));

	return tf;
}

void RosTaxelsTransformNode::GetTaxelTransforms() {
	m3skin_ros::None_TransformArray srv;

	if (local_coords_client.call(srv)) {
		ROS_INFO("Got taxels frames.");

		transforms = srv.response.data;
		std::vector<geometry_msgs::Transform>::iterator it;
		for (it = transforms.begin(); it != transforms.end(); it++) {
			fullTfs.push_back(GeometryTransformToTf(*it));
		}
	} else {
		ROS_ERROR("Failed to get taxels frames");
	}
}

void RosTaxelsTransformNode::InitStaticVectors() {
	centers.clear();
	normals.clear();

	std::vector<tf::Transform>::iterator it;
	for (it = fullTfs.begin(); it != fullTfs.end(); it++) {
		centers.push_back((*it)(tf::Vector3(0, 0, 0)));
		normals.push_back((*it)(tf::Vector3(0, 0, 1)));
	}
}

void RosTaxelsTransformNode::GetLinkName() {
	m3skin_ros::None_String srv;

	if (link_name_client.call(srv)) {
		linkName = srv.response.data;

		ROS_INFO("Got link name %s", linkName.c_str());
	} else {
		ROS_ERROR("Failed to get link name");
	}
}

void RosTaxelsTransformNode::SetCalibrateMode(bool on)
{
	if (on) {
		number_samples = 0;
	}

	calibrating = on;
}

bool RosTaxelsTransformNode::IsCalibrateMode() {
	return calibrating;
}

void RosTaxelsTransformNode::CalibrateBias(std::vector<int> values) {

	if (number_samples == 0) {
		biases.clear();

		std::vector<int>::iterator it;
		for (it = values.begin(); it != values.end(); it++) {
			biases.push_back((*it));
		}

		ROS_INFO("Initialized bias");
		number_samples = 1;
		return;
	}

	ROS_INFO("Calibrating bias %d/%d", number_samples, number_samples_target);

	for (unsigned i = 0; i < values.size(); i++) {
		//ROS_INFO("%d %d = %d", biases[i], values[i], (biases[i] + values[i])/2);
//		if ((biases[i] + values[i])/2.0 != 65535) {
//			ROS_INFO("index value %d %f %d %d", i, biases[i], values[i], (unsigned) floor(biases[i] + 0.5));
//		}

		biases[i] = floor(((biases[i] + values[i])/2.0) + 0.5);
	}

	number_samples += 1;

	if (number_samples == number_samples_target) {
		SetCalibrateMode(false);
		number_samples = 0;
	}
}


void RosTaxelsTransformNode::Init(unsigned calibration_samples) {
    std::string local_coord_srv_name = "/skin_patch_forearm_right/taxels/srv/local_coord_frames";
    std::string link_name_srv_name = "/skin_patch_forearm_right/taxels/srv/link_name";

    local_coords_client = n.serviceClient<m3skin_ros::None_TransformArray> (local_coord_srv_name.c_str());
    link_name_client = n.serviceClient<m3skin_ros::None_String> (link_name_srv_name.c_str());

	publisher = n.advertise<hrl_haptic_manipulation_in_clutter_msgs::TaxelArray> (
			"/skin_patch_forearm_right/taxels/forces", 1000);

	subscriber = n.subscribe<m3skin_ros::RawTaxelArray> (
			"/skin_patch_forearm_right/taxels/raw_data", 1000,
			&RosTaxelsTransformNode::TransformMessage, this);

    ROS_INFO("Waiting for services");
    ros::service::waitForService(local_coord_srv_name.c_str());
    ros::service::waitForService(link_name_srv_name.c_str());
    ROS_INFO("Done waiting");

	GetTaxelTransforms();
	InitStaticVectors();
	GetLinkName();

	// Initialize the biases full of 0
	biases = std::vector<double>(number_taxels, 0);

	number_samples_target = calibration_samples;

	if (number_samples_target) {
	    SetCalibrateMode(true);
	}
}


RosTaxelsTransformNode::RosTaxelsTransformNode() : number_samples(0), calibrating(false) {

}

RosTaxelsTransformNode::~RosTaxelsTransformNode() {
	// TODO Auto-generated destructor stub
}

}
