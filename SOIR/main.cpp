#pragma once

#include <iostream>
#include <fstream>
#include <omp.h>


#include "Eigen.h"
#include "VirtualSensorOpenNI.h"
#include "VirtualSensor.h"
#include "SimpleMesh.h"
#include "ICPOptimizer.h"
#include "ProcrustesAligner.h"
#include "PointCloud.h"


// --------------------------------
// Defines
// --------------------------------
#define CAPTURED_FRAMES_DIR_NAME "../data/Dataset_7.0/"
#define OUTPUT_FILE_NAME "../results/fixedsource_nobackgroundforreal_hierach_vartargetsource_every20_"
#define USE_POINT_TO_PLANE	1



// --------------------------------
// Types
// --------------------------------







int main() {

	// initialize Camera (Intrinsics/Extrinsics)

	

	// load data 
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensorOpenNI sensor;
	if (!sensor.init(CAPTURED_FRAMES_DIR_NAME)) {
		std::cout << " Failed to initialize the sensor!\n Check file path!" << std::endl;
		return -1;
	}

	// We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
	sensor.processNextFrame();
	float* depthMapObj = removeBackground(sensor.getDepth(), sensor.getColorRGBX(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight());
	PointCloud target_first = PointCloud{ depthMapObj, sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };
	PointCloud target = PointCloud{ depthMapObj, sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };



	// Setup the optimizer; 
	ICPOptimizer optimizer;
	optimizer.setMatchingMaxDistance(0.0001f);
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
	SimpleMesh finalMesh = SimpleMesh{ sensor, currentCameraToWorld, 0.0015f };

	int i = 0;
	while (sensor.processNextFrame()) {
		

		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.getDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.getColorRGBX();

		Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
		Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();



		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.

		float* depthMapObj2;
		PointCloud source;

		int nbLayers = 5;
		for (unsigned int i = nbLayers; i > 0; --i) {
			std::cout << " ----- Starting with hierachical level " << i << std::endl;
			float* depthMapObj2 = removeBackground(sensor.getDepth(), sensor.getColorRGBX(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight());
			source = PointCloud{ depthMapObj2, sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight(), (i-1) * 4 + 1};
			currentCameraToWorld = optimizer.estimatePose(source, target, currentCameraToWorld);
		}
		
		if (i % 20 == 0) {
			target = target_first.joinClouds(source);
		}
		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

		
		// We write out the mesh to file for debugging.
		if (i % 10 == 0) {
			SimpleMesh currentDepthMesh{ sensor, currentCameraPose, 0.0015f };
			SimpleMesh currentCameraMesh = SimpleMesh::camera(currentCameraPose, 0.001f);
			SimpleMesh resultingMesh = SimpleMesh::joinMeshes(currentDepthMesh, currentCameraMesh, Matrix4f::Identity());
			finalMesh = SimpleMesh::joinMeshes(finalMesh, currentDepthMesh, Matrix4f::Identity());
			std::stringstream ss;
			ss << OUTPUT_FILE_NAME << sensor.getCurrentFrameCnt() << ".off";
			if (!currentDepthMesh.writeMesh(ss.str())) {
				std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
				return -1;
			}
		}






		i++;
	}
	
	return 0;


		
	


}


