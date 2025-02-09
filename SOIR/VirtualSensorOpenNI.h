#pragma once

#include <vector>
#include <iostream>
#include <cstring>
#include <fstream>

#include "Eigen.h"
#include "FreeImageHelper.h"

typedef unsigned char BYTE;

enum FileType
{
	DEPTH,
	IMAGE
};

struct RawHeader {
	FileType filetype;
	int width;
	int height;
	uint64_t timestamp;

};

// adapted from VirtualSensor.h from https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
class VirtualSensorOpenNI {
public:

	VirtualSensorOpenNI() : m_currentIdx(-1), m_increment(1) {}

	~VirtualSensorOpenNI() {
		SAFE_DELETE_ARRAY(m_depthFrame);
		SAFE_DELETE_ARRAY(m_colorFrame);
	}

	bool init(const std::string& datasetDir) {
		m_baseDir = datasetDir;

		// Read filename lists
		if (!readFileList(datasetDir + "depth.txt", m_filenameDepthImages, m_depthImagesTimeStamps)) return false;
		if (!readFileList(datasetDir + "rgb.txt", m_filenameColorImages, m_colorImagesTimeStamps)) return false;

		// Read tracking
		//if (!readTrajectoryFile(datasetDir + "groundtruth.txt", m_trajectory, m_trajectoryTimeStamps)) return false;

		if (m_filenameDepthImages.size() != m_filenameColorImages.size()) return false;

		// Image resolutions TODO -----------------------------------------------------------------------------------------------------------------
		m_colorImageWidth = 320;
		m_colorImageHeight = 240;
		m_depthImageWidth = 320;
		m_depthImageHeight = 240;

		// Intrinsics TODO
		m_colorIntrinsics << 537.002f, 0.0f, 319.259f,
			0.0f, 536.66f, 233.608f,
			0.0f, 0.0f, 1.0f;

		m_depthIntrinsics = m_colorIntrinsics; //TODO
		

		m_colorExtrinsics.setIdentity();
		m_depthExtrinsics.setIdentity();

		m_depthFrame = new float[m_depthImageWidth * m_depthImageHeight];
		for (unsigned int i = 0; i < m_depthImageWidth * m_depthImageHeight; ++i) m_depthFrame[i] = 0.5f;

		m_colorFrame = new BYTE[4 * m_colorImageWidth * m_colorImageHeight];
		for (unsigned int i = 0; i < 4 * m_colorImageWidth * m_colorImageHeight; ++i) m_colorFrame[i] = 255;


		m_currentIdx = -1;
		return true;
	}

	bool LoadImageFromRAWFile(std::string filename) {
		// Open the .raw file in binary mode
		std::ifstream inputFile(filename, std::ios::binary);

		if (!inputFile.is_open()) {
			std::cerr << "Failed to open file: " << filename << "\n" << std::endl;
			return false;
		}

		// read the header from File
		RawHeader header;
		inputFile.read(reinterpret_cast<char*>(&header), sizeof(header));

		if (inputFile.fail()) {
			std::cerr << "Failed to read header from file.\n" << std::endl;
			return false;
		}

		// check width, height, timestamp and format
		// why different timestamp here???? m_colorImagesTimeStamps[m_currentIdx] != header.timestamp ||s
		
		if (m_colorImageHeight != header.height || m_colorImageWidth != header.width || IMAGE != header.filetype) {
			std::cerr << "Header error while trying to read file " << filename << "\n" << std::endl;
			return false;
		}

		// read data
		inputFile.read(reinterpret_cast<char*>(m_colorFrame), m_colorImageHeight * m_colorImageWidth * sizeof(BYTE) * 3); // TODO * 4? -------------------

		if (inputFile.fail()) {
			std::cerr << "Failed to read image data from file." << std::endl;
			return false;
		}

		// clean up
		inputFile.close();
		return true;
	}

	bool LoadDepthFromRAWFile(std::string filename) {
		// Open the .raw file in binary mode
		std::ifstream inputFile(filename, std::ios::binary);

		if (!inputFile.is_open()) {
			std::cerr << "Failed to open file: " << filename << "\n" << std::endl;
			return false;
		}

		// read the header from File
		RawHeader header;
		inputFile.read(reinterpret_cast<char*>(&header), sizeof(header));

		if (inputFile.fail()) {
			std::cerr << "Failed to read header from file.\n" << std::endl;
			return false;
		}

		// check width, height, timestamp and format
		// why different timestamp here??? m_depthImagesTimeStamps[m_currentIdx] != header.timestamp ||
		if (m_depthImageHeight != header.height || m_depthImageWidth != header.width ||  DEPTH != header.filetype) {
			std::cerr << "Header error while trying to read file " << filename << "\n" << std::endl;
			return false;
		}

		// read data
		uint16_t* depthData = new uint16_t[m_depthImageWidth * m_depthImageHeight];
		inputFile.read(reinterpret_cast<char*>(depthData), m_depthImageWidth * m_depthImageHeight * sizeof(uint16_t));

		if (inputFile.fail()) {
			std::cerr << "Failed to read depth data from file." << std::endl;
			delete[] depthData;
			return false;
		}
		
		//format depth data to same format as https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
		for (unsigned int i = 0; i < m_depthImageWidth * m_depthImageHeight; ++i) {
			if (depthData[i] == 0)
				m_depthFrame[i] = MINF;
			else
				m_depthFrame[i] = depthData[i] * 1.0f / 5000.0f; // TODO correct? / 5000.0f; -----------------------------------------
		}

		// clean up
		delete[] depthData;
		inputFile.close();
		return true;
	}


	bool processNextFrame() {
		if (m_currentIdx == -1) m_currentIdx = 0;
		else m_currentIdx += m_increment;

		if ((unsigned int)m_currentIdx >= (unsigned int)m_filenameColorImages.size()) return false;

		std::cout << "ProcessNextFrame [" << m_currentIdx << " | " << m_filenameColorImages.size() << "]" << std::endl;

		//FreeImageB rgbImage;
		//rgbImage.LoadImageFromFile(m_baseDir + m_filenameColorImages[m_currentIdx]);
		//memcpy(m_colorFrame, rgbImage.data, 4 * m_colorImageWidth * m_colorImageHeight);

		if (!LoadImageFromRAWFile(m_baseDir + m_filenameColorImages[m_currentIdx]))
			return false;
		if (!LoadDepthFromRAWFile(m_baseDir + m_filenameDepthImages[m_currentIdx]))
			return false;

		// depth images are scaled by 5000 (see https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats)
		/**FreeImageU16F dImage;
		dImage.LoadImageFromFile(m_baseDir + m_filenameDepthImages[m_currentIdx]);

		for (unsigned int i = 0; i < m_depthImageWidth * m_depthImageHeight; ++i) {
			if (dImage.data[i] == 0)
				m_depthFrame[i] = MINF;
			else
				m_depthFrame[i] = dImage.data[i] * 1.0f; // TODO correct? / 5000.0f; -----------------------------------------
		}**/


		/** only if you have a trajectory file
		// find transformation (simple nearest neighbor, linear search)
		double timestamp = m_depthImagesTimeStamps[m_currentIdx];
		double min = std::numeric_limits<double>::max();
		int idx = 0;
		for (unsigned int i = 0; i < m_trajectory.size(); ++i) {
			double d = abs(m_trajectoryTimeStamps[i] - timestamp);
			if (min > d) {
				min = d;
				idx = i;
			}
		}
		m_currentTrajectory = m_trajectory[idx];**/

		return true;
	}

	unsigned int getCurrentFrameCnt() {
		return (unsigned int)m_currentIdx;
	}

	// get current color data
	BYTE* getColorRGBX() {
		return m_colorFrame;
	}

	// get current depth data
	float* getDepth() {
		return m_depthFrame;
	}

	// color camera info
	Eigen::Matrix3f getColorIntrinsics() {
		return m_colorIntrinsics;
	}

	Eigen::Matrix4f getColorExtrinsics() {
		return m_colorExtrinsics;
	}

	unsigned int getColorImageWidth() {
		return m_colorImageWidth;
	}

	unsigned int getColorImageHeight() {
		return m_colorImageHeight;
	}

	// depth (ir) camera info
	Eigen::Matrix3f getDepthIntrinsics() {
		return m_depthIntrinsics;
	}

	Eigen::Matrix4f getDepthExtrinsics() {
		return m_depthExtrinsics;
	}

	unsigned int getDepthImageWidth() {
		return m_depthImageWidth;
	}

	unsigned int getDepthImageHeight() {
		return m_depthImageHeight;
	}
	/**
	// get current trajectory transformation
	Eigen::Matrix4f getTrajectory() {
		return m_currentTrajectory;
	}**/

private:
	bool readFileList(const std::string& filename, std::vector<std::string>& result, std::vector<uint64_t>& timestamps) {
		std::ifstream fileDepthList(filename, std::ios::in);
		if (!fileDepthList.is_open()) return false;
		result.clear();
		timestamps.clear();
		std::string dump;
		std::getline(fileDepthList, dump);
		std::getline(fileDepthList, dump);
		std::getline(fileDepthList, dump);
		while (fileDepthList.good()) {
			double timestamp;
			fileDepthList >> timestamp;
			std::string filename;
			fileDepthList >> filename;
			if (filename == "") break;
			timestamps.push_back(timestamp);
			result.push_back(filename);
		}
		fileDepthList.close();
		return true;
	}
	/**
	bool readTrajectoryFile(const std::string& filename, std::vector<Eigen::Matrix4f>& result,
		std::vector<double>& timestamps) {
		std::ifstream file(filename, std::ios::in);
		if (!file.is_open()) return false;
		result.clear();
		std::string dump;
		std::getline(file, dump);
		std::getline(file, dump);
		std::getline(file, dump);

		while (file.good()) {
			double timestamp;
			file >> timestamp;
			Eigen::Vector3f translation;
			file >> translation.x() >> translation.y() >> translation.z();
			Eigen::Quaternionf rot;
			file >> rot;

			Eigen::Matrix4f transf;
			transf.setIdentity();
			transf.block<3, 3>(0, 0) = rot.toRotationMatrix();
			transf.block<3, 1>(0, 3) = translation;

			if (rot.norm() == 0) break;

			transf = transf.inverse().eval();

			timestamps.push_back(timestamp);
			result.push_back(transf);
		}
		file.close();
		return true;
	}**/

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// current frame index
		int m_currentIdx;

	int m_increment;

	// frame data
	float* m_depthFrame;
	BYTE* m_colorFrame;
	Eigen::Matrix4f m_currentTrajectory;

	// color camera info
	Eigen::Matrix3f m_colorIntrinsics;
	Eigen::Matrix4f m_colorExtrinsics;
	unsigned int m_colorImageWidth;
	unsigned int m_colorImageHeight;

	// depth (ir) camera info
	Eigen::Matrix3f m_depthIntrinsics;
	Eigen::Matrix4f m_depthExtrinsics;
	unsigned int m_depthImageWidth;
	unsigned int m_depthImageHeight;

	// base dir
	std::string m_baseDir;
	// filenamelist depth
	std::vector<std::string> m_filenameDepthImages;
	std::vector<uint64_t> m_depthImagesTimeStamps;
	// filenamelist color
	std::vector<std::string> m_filenameColorImages;
	std::vector<uint64_t> m_colorImagesTimeStamps;

	// trajectory
	std::vector<Eigen::Matrix4f> m_trajectory;
	std::vector<double> m_trajectoryTimeStamps;
};
