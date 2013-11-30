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

#ifndef M3_TAXEL_ARRAY_H
#define M3_TAXEL_ARRAY_H

#include "m3/toolbox/toolbox.h"
#include "m3rt/base/component.h"

#include <m3skin/hardware/taxel_array.pb.h>
#include <m3skin/hardware/taxel_array_ec.h>
#include <m3skin/hardware/taxel_array_ec.pb.h>

#include <m3skin_ros/LinkName.h>
#include <m3skin_ros/TaxelFrames.h>
#include <m3skin_ros/LocalCoordFrames.h>
#include <m3skin_ros/JointToSkinOriginTf.h>

#include <google/protobuf/message.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>


namespace m3skin {
using namespace std;
using namespace m3;
using namespace ros;

class M3TaxelArray: public m3rt::M3Component {
public:
	M3TaxelArray() :
		tmp_cnt(0), taxelArrayEc(NULL), number_taxels(0) {
		RegisterVersion("default", DEFAULT);
	}

	google::protobuf::Message * GetCommand() {
		return &command;
	}
	google::protobuf::Message * GetStatus() {
		return &status;
	}
	google::protobuf::Message * GetParam() {
		return &param;
	}

	//Utility
	int64_t GetTimestamp() {
		return status.base().timestamp();
	}

	M3BaseStatus* StatusBase() {
		return status.mutable_base();
	}

	/**
	 * Maximum number of taxels per skin patch.
	 * Arbitrary value.
	 */
	static const unsigned MAX_TAXELS = 1024;

	bool RosExportParam(ros::NodeHandle * node_handle);

protected:
	static const unsigned ROTATION_MATRIX_DIM = 16;
	enum {
		DEFAULT
	};//Version

	Publisher RosInitPublish(NodeHandle * node_handle);

	bool RosPublish(Publisher * pub);

	bool RosJointToSkinOriginTfCb(m3skin_ros::JointToSkinOriginTf::Request &req, m3skin_ros::JointToSkinOriginTf::Response &res);

	bool RosLocalCoordFramesCb(m3skin_ros::LocalCoordFrames::Request &req, m3skin_ros::LocalCoordFrames::Response &res);

	bool RosTaxelFramesCb(m3skin_ros::TaxelFrames::Request &req, m3skin_ros::TaxelFrames::Response &res);

	bool RosLinkNameCb(m3skin_ros::LinkName::Request &req, m3skin_ros::LinkName::Response &res);

	void BroadcastTaxelTransforms();

	void Startup();

	void Shutdown();

	void StepStatus();

	void StepCommand();

	bool LinkDependentComponents();

	bool ReadConfig(const char * filename);

	M3BaseStatus * GetBaseStatus() {
		return status.mutable_base();
	}

	typedef std::vector<std::vector<double> > TfMatrix;

	tf::Transform GetTfTransform(const TfMatrix& matrix);
	XmlRpc::XmlRpcValue VectorToXmlRpcList(const TfMatrix& input);
	XmlRpc::XmlRpcValue VectorToXmlRpcList(const std::vector<TfMatrix>& input);

	M3TaxelArrayEcParam param; // Set from Python or external component.
	M3TaxelArrayEcCommand command; // Set from Python or external component.
	M3TaxelArrayEcStatus status;

	int tmp_cnt;

	double taxel_area;
	unsigned number_taxels;
	TfMatrix joint_to_skin_matrix;
	tf::Transform joint_to_skin_tf;
	std::vector<TfMatrix> taxel_matrices;
	std::vector<tf::Transform> taxel_tfs;

private:
	void InitRawTaxelArray();

	ros::ServiceServer srv_local_coords;
	ros::ServiceServer srv_taxel_frames;
	ros::ServiceServer srv_joint_to_skin;
	ros::ServiceServer srv_link_name;

	M3TaxelArrayEc* taxelArrayEc;
	M3TaxelArrayEcStatus* taxelArrayEcStatus;
	std::vector<unsigned> rawTaxelValues;
};

}

#endif

