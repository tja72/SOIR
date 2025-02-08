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
#include "icp.h"
#include "rbf.h"



// --------------------------------
// Defines
// --------------------------------
#define CAPTURED_FRAMES_DIR_NAME "../data/Dataset_1/"
#define OUTPUT_FILE_NAME "../results/"
#define USE_POINT_TO_PLANE	1


// --------------------------------
// Types
// --------------------------------


// TODOS:
//		include libraries and dependecies from exercise 3
//		implement removeBackground
//		adapt VirtualSensorOpenNI (width & height)
//		adapt NiViewer_modified to safe the data in by FreeImageHelper supported file type and in other File structure 
//		find out extrinsics and intrinsics & add in data capturing process
//		find out where to include RBF
//		Include/add ICP/Optimizer
//		after that: add task specific steps (turn around object etc...)





void removeBackground(PointCloud* pointCloud) {
	//should return VirtualSensor
	return;
}

int main() {

	// initialize Camera (Intrinsics/Extrinsics)

	

	// load data 
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensorOpenNI sensor;
	if (!sensor.init(CAPTURED_FRAMES_DIR_NAME)) {
		std::cout << " Failed to initialize the sensor!\n Check file path!" << std::endl;
		return -1;
	}

	// Setup the optimizer; TODO -----------------------------------------------------------------------------------------------------------------
	
	ICPOptimizer optimizer;
	optimizer.setMatchingMaxDistance(0.0003f);
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
	PointCloud target;

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
		if (i == 0) {
			// We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
			target = PointCloud{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };
			removeBackground(&target); // TODO
			
		}
		else {
			PointCloud source{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight(), 8 };
			removeBackground(&source); // TODO
			currentCameraToWorld = optimizer.estimatePose(source, target, currentCameraToWorld);
		}

		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

		
		// We write out the mesh to file for debugging.
		SimpleMesh currentDepthMesh{ sensor, currentCameraPose, 0.1f };
		SimpleMesh currentCameraMesh = SimpleMesh::camera(currentCameraPose, 0.0015f);
		SimpleMesh resultingMesh = SimpleMesh::joinMeshes(currentDepthMesh, currentCameraMesh, Matrix4f::Identity());

		std::stringstream ss;
		ss << OUTPUT_FILE_NAME << sensor.getCurrentFrameCnt() << ".off";
		if (!currentDepthMesh.writeMesh(ss.str())) {
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}
		

		i++;
	}

	return 0;

	/////////////////////////////////////
		
	

	// extract Background
	//VirtualSensor OutputFromVio;
	
	// ICP
	//constructObject(sensor);

	// RBF
	//std::string filenameIn = "./data/normalized.pcb";
	//applyRBF_MarchingCubes(filenameIn);
	// save result

	

}


