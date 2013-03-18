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

#ifndef ROSTAXELSTRANSFORMNODE_H_
#define ROSTAXELSTRANSFORMNODE_H_

#include <m3skin_ros/RawTaxelArray.h>
#include <m3skin_ros/TaxelArray.h>

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>

 #include <tf/transform_broadcaster.h>

namespace m3 {

class RosTaxelsTransformNode {
public:
	RosTaxelsTransformNode();
	virtual ~RosTaxelsTransformNode();

	/**
	 * Initialize the ROS node.
	 * @param calibration_samples The number of samples to collect before passing on the data
	 * If set to 0, no calibration phase will be carried on.
	 */
	void Init(unsigned calibration_samples = 0);

	/**
	 * Set the calibration mode to on or off.
	 * @param on True if is calibrating, False otherwise
	 * @postcondition After being call, a calibration flag will be set and
	 * whenever a new message will be received, the calibration biases will be
	 * updated for each taxel.
	 */
	void SetCalibrateMode(bool on);

	/**
	 * @return true if currently calibrating
	 */
	bool IsCalibrateMode();

	static const double MAX_VALUE = 30;

protected:
	/**
	 * Publishes a TaxelArray message from the RawTaxelArray message
	 * @param msg The RawTaxeArray message to transform
	 */
	void TransformMessage(const m3skin_ros::RawTaxelArray::ConstPtr& msg);

	/**
	 * Filter the raw taxels array messages
	 * @param msg The input message to filter
	 * @return a filtered message
	 */
	virtual m3skin_ros::RawTaxelArray FilterRaw(const m3skin_ros::RawTaxelArray::ConstPtr& msg);

	/**
	 * Fetches all the taxels tfs from the
	 * /skin_patch_forearm_right/taxels/srv/local_coord_frames service
	 * @postcondition The transforms will be stored inside this object.
	 */
	void GetTaxelTransforms();

	/**
	 * Fetches the link name
	 * @postcondition The link name will be stored inside this object.
	 */
	void GetLinkName();

	/**
	 * Converts a geometry_msgs/Transform to a tf::Transform
	 * @param gtf The geometry_msgs/Transform
	 * @return a tf::Transform object for the input
	 */
	tf::Transform GeometryTransformToTf(geometry_msgs::Transform& gtf);

	/**
	 * Update the average biases with the new values
	 * @param values An array of raw taxel values
	 */
	void CalibrateBias(std::vector<int> values);

private:

	/**
	 * Initialize the centers and normals vectors
	 */
	void InitStaticVectors();

	/**
	 * @return The unbiased value of a taxel
	 * @precondition The bias vector should be contain a bias value for each taxel
	 */
	double GetUnbiased(unsigned value, unsigned index);

	ros::NodeHandle n;
	ros::Publisher publisher;
	ros::Subscriber subscriber;
	ros::ServiceClient local_coords_client;
	ros::ServiceClient link_name_client;

	std::vector<geometry_msgs::Transform> transforms;
	std::vector<tf::Transform> fullTfs;

	std::vector<tf::Vector3> centers;
	std::vector<tf::Vector3> normals;

	std::vector<double> biases;
	unsigned number_samples;
	unsigned number_samples_target;
	bool calibrating;

	std::string linkName;

	static const unsigned number_taxels = 384; // TODO Get this value from the server
};

}

#endif /* ROSTAXELSTRANSFORMNODE_H_ */
