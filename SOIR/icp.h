#pragma once
#include <iostream>
#include <fstream>

#include "Eigen.h"
#include "VirtualSensor.h"
#include "SimpleMesh.h"
#include "ICPOptimizer.h"
#include "PointCloud.h"

using namespace simple_mesh1;
using namespace point_cloud;

//ill kept this from the exercise, can be integrated into parameters later
#define USE_POINT_TO_PLANE	1


int constructObject(VirtualSensorOpenNI sensor) {

	// We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
	sensor.processNextFrame();
	point_cloud::PointCloud target{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };

	// Setup the optimizer.
	ICPOptimizer optimizer;
	optimizer.setMatchingMaxDistance(0.1f);
	if (USE_POINT_TO_PLANE) {
		optimizer.usePointToPlaneConstraints(true);
		optimizer.setNbOfIterations(10);
	}
	else {
		optimizer.usePointToPlaneConstraints(false);
		optimizer.setNbOfIterations(20);
	}

	// We store the estimated camera poses.
	std::vector<Matrix4f> estimatedPoses;
	Matrix4f currentCameraToWorld = Matrix4f::Identity();
	estimatedPoses.push_back(currentCameraToWorld.inverse());
	//PointCloud target;

	int i = 0;
	//const int iMax = 50;
	const int iMax = 10;
	while (sensor.processNextFrame() && i <= iMax) {
		float* depthMap = sensor.getDepth();
		BYTE* colorMap = sensor.getColorRGBX();
		Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
		Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();

		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.
		point_cloud::PointCloud source{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight(), 2 };
		currentCameraToWorld = optimizer.estimatePose(source, target, currentCameraToWorld);


		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

		if (i % 5 == 0) {

			simple_mesh1::SimpleMesh currentDepthMesh{ sensor, currentCameraPose, 0.1f };
			simple_mesh1::SimpleMesh currentCameraMesh = simple_mesh1::SimpleMesh::camera(currentCameraPose, 0.0015f);
			simple_mesh1::SimpleMesh resultingMesh = simple_mesh1::SimpleMesh::joinMeshes(currentDepthMesh, currentCameraMesh, Matrix4f::Identity());

			//i'll keep that in, to be able to return a mesh for debugging purposes.
			// BEWARE, fix the return if you want to use it.
			//
			std::string filenameBaseOut = "merged_pointcloud";
			std::stringstream ss;
			ss << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off";
			if (!resultingMesh.writeMesh(ss.str())) {
				std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
				return -1;
			
			}
		}

		i++;
	}

	return 0;
}

