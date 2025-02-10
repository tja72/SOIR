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

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

using PointT = pcl::PointXYZRGB; // Assuming your point cloud has color info
using PointCloudT = pcl::PointCloud<PointT>;

// --------------------------------
// Defines
// --------------------------------
#define CAPTURED_FRAMES_DIR_NAME "../data/Dataset_5.0/"
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

PointCloudT removeColorRange(const PointCloudT& inputCloud,
	int r_min, int r_max,
	int g_min, int g_max,
	int b_min, int b_max) {
	PointCloudT outputCloud;

	for (const auto& point : inputCloud.points) {
		// Extract RGB values
		int r = point.r;
		int g = point.g;
		int b = point.b;

		// Check if the point falls within the unwanted color range
		if (!(r_min <= r && r <= r_max &&
			g_min <= g && g <= g_max &&
			b_min <= b && b <= b_max)) {
			outputCloud.points.push_back(point);
		}
	}

	outputCloud.width = outputCloud.points.size();
	outputCloud.height = 1;
	outputCloud.is_dense = true;

	return outputCloud;
}

PointCloudT removeBackground(const PointCloudT& inputCloud) {
	PointCloudT::Ptr cloudFiltered(new PointCloudT);

	// Filter points beyond a depth threshold
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(inputCloud.makeShared()); // Convert to shared pointer
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.15); // Keep points below 1.15cm (discance between points and camera)
	pass.filter(*cloudFiltered);

	// Remove statistical outliers
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloudFiltered);
	sor.setMeanK(50); // Consider 50 nearest neighbors
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloudFiltered);

	// Cluster extraction using KD tree to isolate foreground object
	std::vector<pcl::PointIndices> clusterIndices;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloudFiltered);

	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(0.02); // 2 cm cluster distance
	ec.setMinClusterSize(100); // Minimum 100 points per cluster
	ec.setMaxClusterSize(50000); // Max 50,000 points per cluster
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloudFiltered);
	ec.extract(clusterIndices);

	// Extract largest cluster (assuming the object in the foreground)
	PointCloudT::Ptr largestCluster(new PointCloudT);
	if (!clusterIndices.empty()) {
		pcl::PointIndices largest = clusterIndices[0]; // The first cluster is usually the largest
		for (int index : largest.indices) {
			largestCluster->points.push_back(cloudFiltered->points[index]);
		}
		largestCluster->width = largestCluster->points.size();
		largestCluster->height = 1;
		largestCluster->is_dense = true;
	}

	return *largestCluster;
}


pcl::PointCloud<PointT> mapToPointCloud(float* depthMap, BYTE* colorMap, unsigned int width, unsigned int height, Eigen::Matrix3f colorIntrinsics, Eigen::Matrix4f colorExtrinsics, Eigen::Matrix3f depthIntrinsics, Eigen::Matrix4f depthExtrinsics) {


	pcl::PointCloud<PointT> cloud;
	float fx = depthIntrinsics(0, 0);
	float fy = depthIntrinsics(1, 1);
	float cx = depthIntrinsics(0, 2);
	float cy = depthIntrinsics(1, 2);

	for (unsigned int i = 0; i < height; ++i) {
		for (unsigned int j = 0; j < width; ++j) {
			unsigned int idx = i * width + j;
			float depth = depthMap[idx];

			if (depth > 0) { // Nur gültige Tiefenwerte verwenden
				float x = (j - cx) * depth / fx;
				float y = (i - cy) * depth / fy;
				float z = depth;

				// Transformiere die Tiefenkoordinaten in die RGB-Kamera-Koordinaten
				Eigen::Vector4f depthPoint(x, y, z, 1.0f);
				Eigen::Vector4f rgbPoint = colorExtrinsics * depthExtrinsics.inverse() * depthPoint;

				BYTE r = colorMap[4 * idx + 0];
				BYTE g = colorMap[4 * idx + 1];
				BYTE b = colorMap[4 * idx + 2];

				PointT point;
				point.x = rgbPoint.x();
				point.y = rgbPoint.y();
				point.z = rgbPoint.z();
				point.r = r;
				point.g = g;
				point.b = b;

				cloud.points.push_back(point);
			}
		}
	}

	cloud.width = cloud.points.size();
	cloud.height = 1;
	cloud.is_dense = false;

	return cloud;
}

PointCloud convertToPointCloud(pcl::PointCloud<PointT>& pc) {
	SimpleMesh mesh;
	// iterate over all points in the pc
	for (const auto& point : pc.points) {
		// add the point to the cloud
		mesh.addVertex({ Vector4f(point.x, point.y, point.z, 1.0f), Vector4uc(point.r, point.g, point.b, 1.0f) });
	}

	PointCloud cloud = PointCloud(mesh);
	return cloud;
}

int main() {
	int r_min, r_max, g_min, g_max, b_min, b_max; // TODO set to color range of background
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
	PointCloud target = PointCloud{ depthMapObj, sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };
	PointCloudT targetWithoutBackground = removeBackground(&target);
	// Remove points of a certain color spectrum only found in the background
	PointCloud targetWithoutBackgroundColoredBits = removeColorRange(
		&targetWithoutBackground,
		r_min, r_max,
		g_min, g_max,
		b_min, b_max
	);

	
			// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.



	// Setup the optimizer; TODO -----------------------------------------------------------------------------------------------------------------
	ICPOptimizer optimizer;
	optimizer.setMatchingMaxDistance(0.0001f);
	if (USE_POINT_TO_PLANE) {
		optimizer.usePointToPlaneConstraints(true);
		optimizer.setNbOfIterations(40);
	}
	else {
		optimizer.usePointToPlaneConstraints(false);
		optimizer.setNbOfIterations(20);
	}
	
	// We store the estimated camera poses.
	std::vector<Matrix4f> estimatedPoses;
	Matrix4f currentCameraToWorld = Matrix4f::Identity();
	estimatedPoses.push_back(currentCameraToWorld.inverse());
	SimpleMesh finalMesh = SimpleMesh{ sensor, currentCameraToWorld, 0.1f};

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
		depthMapObj2 = removeBackground(sensor.getDepth(), sensor.getColorRGBX(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight());
		PointCloud source{ depthMapObj2, sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight(), 8 };
		PointCloud sourceWithoutBackground = removeBackground(&source);
		PointCloud targetWithoutBackgroundColoredBits = removeColorRange(
			&sourceWithoutBackground,
			r_min, r_max,
			g_min, g_max,
			b_min, b_max
		);
		currentCameraToWorld = optimizer.estimatePose(source, target, currentCameraToWorld);
		

		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

		
		// We write out the mesh to file for debugging.
		SimpleMesh currentDepthMesh{ sensor, currentCameraPose, 0.0015f};
		SimpleMesh currentCameraMesh = SimpleMesh::camera(currentCameraPose, 0.001f);
		SimpleMesh resultingMesh = SimpleMesh::joinMeshes(currentDepthMesh, currentCameraMesh, Matrix4f::Identity());
		finalMesh = SimpleMesh::joinMeshes(finalMesh, currentDepthMesh, currentCameraToWorld);
		std::stringstream ss;
		ss << OUTPUT_FILE_NAME << sensor.getCurrentFrameCnt() << ".off";
		if (!currentDepthMesh.writeMesh(ss.str())) {
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		
		i++;
	}

	return 0;


		
	

	// extract Background


	// ICP

	// RBF

	// save result

	

}


