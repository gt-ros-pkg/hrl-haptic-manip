/* 
 MEKA CONFIDENTIAL

 Copyright 2011
 Meka Robotics LLC
 All Rights Reserved.

 NOTICE:  All information contained herein is, and remains
 the property of Meka Robotics LLC. The intellectual and
 technical concepts contained herein are proprietary to
 Meka Robotics LLC and may be covered by U.S. and Foreign Patents,
 patents in process, and are protected by trade secret or copyright law.
 Dissemination of this information or reproduction of this material
 is strictly forbidden unless prior written permission is obtained
 from Meka Robotics LLC.
 */

#include "taxel_array.h"

#include <m3rt/base/m3rt_def.h>
#include <m3rt/base/component_factory.h>
#include <m3skin_ros/RawTaxelArray.h>

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>

namespace m3skin {

//unsigned (*M3TaxelArray::TaxelIdToTf[2])(unsigned) = { &M3TaxelArray::MapBranchA, &M3TaxelArray::MapBranchB } ;

using namespace m3rt;
using namespace std;
using namespace m3;
using namespace ros;

Publisher M3TaxelArray::RosInitPublish(NodeHandle * node_handle) {
	// FIXME : Kind of a hack. Does not comply with the usual Status, Param, Cmd architecture in M3
	srv_local_coords = node_handle->advertiseService("/skin_patch_forearm_right/taxels/srv/local_coord_frames",
			   &M3TaxelArray::RosLocalCoordFramesCb, this);

	srv_taxel_frames = node_handle->advertiseService("/skin_patch_forearm_right/taxels/srv/taxel_frames",
			   &M3TaxelArray::RosTaxelFramesCb, this);

	srv_joint_to_skin = node_handle->advertiseService("/skin_patch_forearm_right/taxels/srv/joint_to_skin_transform",
			   &M3TaxelArray::RosJointToSkinOriginTfCb, this);

	srv_link_name = node_handle->advertiseService("/skin_patch_forearm_right/taxels/srv/link_name",
			   &M3TaxelArray::RosLinkNameCb, this);

	return node_handle->advertise<m3skin_ros::RawTaxelArray> (
			"/skin_patch_forearm_right/taxels/raw_data", 1000);
}

bool M3TaxelArray::RosJointToSkinOriginTfCb(m3skin_ros::JointToSkinOriginTf::Request &req, m3skin_ros::JointToSkinOriginTf::Response &res)
{
	tf::Vector3 trans = joint_to_skin_tf.getOrigin();
	tf::Quaternion quat = joint_to_skin_tf.getRotation();

	geometry_msgs::Transform transform;
	transform.translation.x = trans.x();
	transform.translation.y = trans.y();
	transform.translation.z = trans.z();

	transform.rotation.x = quat.x();
	transform.rotation.y = quat.y();
	transform.rotation.z = quat.z();
	transform.rotation.w = quat.w();

	res.data = transform;
}

bool M3TaxelArray::RosLocalCoordFramesCb(m3skin_ros::LocalCoordFrames::Request &req, m3skin_ros::LocalCoordFrames::Response &res)
{
	std::vector<tf::Transform>::iterator it;
	for (it = taxel_tfs.begin(); it != taxel_tfs.end(); it++) {
		tf::Transform tf = joint_to_skin_tf*(*it);
		tf::Quaternion quat = tf.getRotation();
		tf::Vector3 trans = tf.getOrigin();

		geometry_msgs::Transform transform;
		transform.translation.x = trans.x();
		transform.translation.y = trans.y();
		transform.translation.z = trans.z();

		transform.rotation.x = quat.x();
		transform.rotation.y = quat.y();
		transform.rotation.z = quat.z();
		transform.rotation.w = quat.w();

		res.data.push_back(transform);
	}

	return true;
}

bool M3TaxelArray::RosTaxelFramesCb(m3skin_ros::TaxelFrames::Request &req, m3skin_ros::TaxelFrames::Response &res)
{
	std::vector<tf::Transform>::iterator it;
	for (it = taxel_tfs.begin(); it != taxel_tfs.end(); it++) {
		tf::Quaternion quat = (*it).getRotation();
		tf::Vector3 trans = (*it).getOrigin();

		geometry_msgs::Transform transform;
		transform.translation.x = trans.x();
		transform.translation.y = trans.y();
		transform.translation.z = trans.z();

		transform.rotation.x = quat.x();
		transform.rotation.y = quat.y();
		transform.rotation.z = quat.z();
		transform.rotation.w = quat.w();

		res.data.push_back(transform);
	}

	return true;
}

bool M3TaxelArray::RosLinkNameCb(m3skin_ros::LinkName::Request &req, m3skin_ros::LinkName::Response &res)
{
	// FIXME : De-hardcode
	res.data = "/wrist_RIGHT";

	return true;
}

bool M3TaxelArray::RosPublish(Publisher * pub) {
	m3skin_ros::RawTaxelArray msg;

	for (unsigned i = 0; i < rawTaxelValues.size(); i++) {
		msg.val_z.push_back(rawTaxelValues[i]);
	}

	pub->publish(msg);
}

bool M3TaxelArray::ReadConfig(const char * filename) {
	if (!M3Component::ReadConfig(filename))
		return false;

	M3_INFO("Reading config\n");

	YAML::Node doc;
	GetYamlDoc(filename, doc);

	// Number of taxels
	doc["number_taxels"] >> number_taxels;

	// Taxel area
	doc["taxel_area"] >> taxel_area;

	// Join to skin transformation
	const YAML::Node& joinToSkinTfDoc = doc["joint_to_skin_tf"];
	joinToSkinTfDoc[0] >> joint_to_skin_matrix;
	M3_INFO("joint_to_skin_matrix 0 0 %e\n", joint_to_skin_matrix[0][0]);

	joint_to_skin_tf = GetTfTransform(joint_to_skin_matrix);

	// Individual taxels transformations
	const YAML::Node& taxels = doc["taxels_tf"];
	for (YAML::Iterator it = taxels.begin(); it != taxels.end(); ++it) {

		TfMatrix taxelMat;
		(*it) >> taxelMat;

		taxel_matrices.push_back(taxelMat);
		taxel_tfs.push_back(GetTfTransform(taxelMat));
	}

	InitRawTaxelArray();

	M3_INFO("Got all TF for joints\n");

	return true;
}

XmlRpc::XmlRpcValue M3TaxelArray::VectorToXmlRpcList(const TfMatrix& input) {
	XmlRpc::XmlRpcValue rows;
	for (unsigned i = 0; i < input.size(); i++) {
		std::vector<double> row = input[i];
		for (unsigned j = 0; j < row.size(); j++) {
			rows[i][j] = row[j];
		}
	}

	return rows;
}

XmlRpc::XmlRpcValue M3TaxelArray::VectorToXmlRpcList(
		const std::vector<TfMatrix>& input) {
	XmlRpc::XmlRpcValue tfs;

	for (unsigned k = 0; k < input.size(); k++) {
		const TfMatrix& tfMatrix = input[k];

		for (unsigned i = 0; i < tfMatrix.size(); i++) {
			std::vector<double> row = tfMatrix[i];

			for (unsigned j = 0; j < row.size(); j++) {
				tfs[k][i][j] = row[j];
			}
		}
	}

	return tfs;
}

bool M3TaxelArray::RosExportParam(ros::NodeHandle * node_handle) {
	M3_INFO("Exporting parameters to ROS Parameter server.\n");

	node_handle->setParam("taxel_area", taxel_area);

	node_handle->setParam("number_taxels", (int) number_taxels);

	XmlRpc::XmlRpcValue jointToSkinTfXml = VectorToXmlRpcList(
			joint_to_skin_matrix);
	node_handle->setParam("joint_to_skin_tf", jointToSkinTfXml);

	XmlRpc::XmlRpcValue taxelsTfXml = VectorToXmlRpcList(taxel_matrices);
	node_handle->setParam("taxel_matrices", taxelsTfXml);
}

bool M3TaxelArray::LinkDependentComponents() {
	std::string ecName = "m3taxel_array_ec01";
	taxelArrayEc = (M3TaxelArrayEc*) factory->GetComponent(ecName);
	if (taxelArrayEc == NULL) {
		M3_INFO("m3taxel_array_ec component %s not found for component %s\n",
				ecName.c_str(), GetName().c_str());

		return false;
	}

	return true;
}

tf::Transform M3TaxelArray::GetTfTransform(const TfMatrix& matrix) {
	tf::Transform t;
	t.setOrigin(tf::Vector3(matrix[0][3], matrix[1][3], matrix[2][3]));
	t.setBasis(
			btMatrix3x3(matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0],
					matrix[1][1], matrix[1][2], matrix[2][0], matrix[2][1],
					matrix[2][2]));

	return t;
}

void M3TaxelArray::BroadcastTaxelTransforms() {
	static tf::TransformBroadcaster br;

	// Send the tf from the joint to the origin on the skin patch
	br.sendTransform(
			tf::StampedTransform(joint_to_skin_tf, ros::Time::now(),
					"wrist_RIGHT", "joint_to_skin_tf"));

	// Send the tfs from the origin on the skin patch to the taxels
	std::vector<tf::Transform>::iterator it;
	unsigned i = 0;
	for (it = taxel_tfs.begin(); it != taxel_tfs.end(); it++, i++) {
		std::ostringstream ss;
		ss << "taxel_tf_" << i;
		br.sendTransform(
				tf::StampedTransform((*it), ros::Time::now(),
						"joint_to_skin_tf", ss.str()));
	}
}

void M3TaxelArray::InitRawTaxelArray()
{
	rawTaxelValues.clear();
	for (unsigned i = 0; i < number_taxels; i++) {
		rawTaxelValues.push_back(0);
	}

	rawTaxelValues[0] = 32767;
	rawTaxelValues[number_taxels-1] = 5000;

	M3_INFO("Raw taxel values init\n");
}

void M3TaxelArray::Startup() {
	if (taxelArrayEc != NULL)
		SetStateSafeOp();
	else
		SetStateError();

	InitRawTaxelArray();
}

void M3TaxelArray::Shutdown() {

}

inline unsigned MapBranchA(unsigned idx) {
	return std::floor(idx/12)*12 + idx;
}

inline unsigned MapBranchB(unsigned idx) {
	return (idx - 192) + (std::floor((idx - 192)/12)+1)*12;
}

void M3TaxelArray::StepStatus() {
	if (!taxelArrayEc && IsStateError())
		return;

	taxelArrayEcStatus = (M3TaxelArrayEcStatus*) taxelArrayEc->GetStatus();
	int n = taxelArrayEcStatus->taxel_value_size();
	rawTaxelValues = std::vector<unsigned>(n, 0);

	for (int i = 0; i < n; i++) {
		int idxm = (i < 192) ? MapBranchA(i) : MapBranchB(i);
		//M3_INFO("Mapping taxel %d to %d with function %d\n", i, idxm, (i < 192));

		rawTaxelValues[idxm] = taxelArrayEcStatus->taxel_value(i);
	}
}

void M3TaxelArray::StepCommand() {
	if (!taxelArrayEc && IsStateSafeOp())
		return;
}
}
