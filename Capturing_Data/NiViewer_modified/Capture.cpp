/****************************************************************************
*                                                                           *
*  OpenNI 1.x Alpha                                                         *
*  Copyright (C) 2011 PrimeSense Ltd.                                       *
*                                                                           *
*  This file is part of OpenNI.                                             *
*                                                                           *
*  OpenNI is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU Lesser General Public License as published *
*  by the Free Software Foundation, either version 3 of the License, or     *
*  (at your option) any later version.                                      *
*                                                                           *
*  OpenNI is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
*  GNU Lesser General Public License for more details.                      *
*                                                                           *
*  You should have received a copy of the GNU Lesser General Public License *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
*                                                                           *
****************************************************************************/
// --------------------------------
// Includes
// --------------------------------
#include "Capture.h"
#include "Device.h"
#include "Draw.h"
#include <XnCppWrapper.h>
#include <XnCodecIDs.h>
using namespace xn;

#if (XN_PLATFORM == XN_PLATFORM_WIN32)
#include <Commdlg.h>
#endif
#include <fstream>
#include <chrono>
#include <iostream>
#include <vector>
#include <tuple>

// --------------------------------
// Defines
// --------------------------------
#define CAPTURED_FRAMES_DIR_NAME "../../data/Dataset_9.0"

// --------------------------------
// Types
// --------------------------------
typedef enum
{
	NOT_CAPTURING,
	SHOULD_CAPTURE,
	CAPTURING,
} CapturingState;

typedef enum
{
	CAPTURE_DEPTH_NODE,
	CAPTURE_IMAGE_NODE,
	CAPTURE_IR_NODE,
	CAPTURE_AUDIO_NODE,
	CAPTURE_NODE_COUNT
} CaptureNodeType;

typedef struct NodeCapturingData
{
	XnCodecID captureFormat;
	XnUInt32 nCapturedFrames;
	bool bRecording;
	xn::Generator* pGenerator;
} NodeCapturingData;

typedef struct CapturingData
{
	NodeCapturingData nodes[CAPTURE_NODE_COUNT];
	Recorder* pRecorder;
	char csFileName[XN_FILE_MAX_PATH];
	XnUInt32 nStartOn; // time to start, in seconds
	bool bSkipFirstFrame;
	CapturingState State;
	XnUInt32 nCapturedFrameUniqueID;
	char csDisplayMessage[500];
} CapturingData;

enum FileType
{
	DEPTH,
	IMAGE
};

typedef struct RawHeader {
	FileType filetype;
	int width;
	int height;
	uint64_t timestamp;

} RawHeader;

// --------------------------------
// Global Variables
// --------------------------------
CapturingData g_Capture;

NodeCodec g_DepthFormat;
NodeCodec g_ImageFormat;
NodeCodec g_IRFormat;
NodeCodec g_AudioFormat;

static const XnCodecID CODEC_DONT_CAPTURE = XN_CODEC_NULL;


bool isCapturingTxt = false;
bool closeRecording = false;
int hasNewFrame = 0;
int captureFrameRate = 60;
bool closeCapturingTxt = false;

std::vector<std::tuple<std::string, std::string>> listOfDepthNamesTxt;
std::vector<std::tuple<std::string, std::string>> listOfImageNamesTxt;


// --------------------------------
// Code
// --------------------------------
void captureInit()
{
	// Depth Formats
	int nIndex = 0;

	g_DepthFormat.pValues[nIndex] = XN_CODEC_16Z_EMB_TABLES;
	g_DepthFormat.pIndexToName[nIndex] = "PS Compression (16z ET)";
	nIndex++;

	g_DepthFormat.pValues[nIndex] = XN_CODEC_UNCOMPRESSED;
	g_DepthFormat.pIndexToName[nIndex] = "Uncompressed";
	nIndex++;

	g_DepthFormat.pValues[nIndex] = CODEC_DONT_CAPTURE;
	g_DepthFormat.pIndexToName[nIndex] = "Not Captured";
	nIndex++;

	g_DepthFormat.nValuesCount = nIndex;

	// Image Formats
	nIndex = 0;

	g_ImageFormat.pValues[nIndex] = XN_CODEC_JPEG;
	g_ImageFormat.pIndexToName[nIndex] = "JPEG";
	nIndex++;

	g_ImageFormat.pValues[nIndex] = XN_CODEC_UNCOMPRESSED;
	g_ImageFormat.pIndexToName[nIndex] = "Uncompressed";
	nIndex++;

	g_ImageFormat.pValues[nIndex] = CODEC_DONT_CAPTURE;
	g_ImageFormat.pIndexToName[nIndex] = "Not Captured";
	nIndex++;

	g_ImageFormat.nValuesCount = nIndex;

	// IR Formats
	nIndex = 0;

	g_IRFormat.pValues[nIndex] = XN_CODEC_UNCOMPRESSED;
	g_IRFormat.pIndexToName[nIndex] = "Uncompressed";
	nIndex++;

	g_IRFormat.pValues[nIndex] = CODEC_DONT_CAPTURE;
	g_IRFormat.pIndexToName[nIndex] = "Not Captured";
	nIndex++;

	g_IRFormat.nValuesCount = nIndex;

	// Audio Formats
	nIndex = 0;

	g_AudioFormat.pValues[nIndex] = XN_CODEC_UNCOMPRESSED;
	g_AudioFormat.pIndexToName[nIndex] = "Uncompressed";
	nIndex++;

	g_AudioFormat.pValues[nIndex] = CODEC_DONT_CAPTURE;
	g_AudioFormat.pIndexToName[nIndex] = "Not Captured";
	nIndex++;

	g_AudioFormat.nValuesCount = nIndex;

	// Init
	g_Capture.csFileName[0] = 0;
	g_Capture.State = NOT_CAPTURING;
	g_Capture.nCapturedFrameUniqueID = 0;
	g_Capture.csDisplayMessage[0] = '\0';
	g_Capture.bSkipFirstFrame = false;

	g_Capture.nodes[CAPTURE_DEPTH_NODE].captureFormat = XN_CODEC_16Z_EMB_TABLES;
	g_Capture.nodes[CAPTURE_IMAGE_NODE].captureFormat = XN_CODEC_JPEG;
	g_Capture.nodes[CAPTURE_IR_NODE].captureFormat = XN_CODEC_UNCOMPRESSED;
	g_Capture.nodes[CAPTURE_AUDIO_NODE].captureFormat = XN_CODEC_UNCOMPRESSED;
}

bool isCapturing()
{
	return (g_Capture.State != NOT_CAPTURING);
}

#define START_CAPTURE_CHECK_RC(rc, what)												\
	if (nRetVal != XN_STATUS_OK)														\
	{																					\
		displayMessage("Failed to %s: %s\n", what, xnGetStatusString(rc));				\
		delete g_Capture.pRecorder;														\
		g_Capture.pRecorder = NULL;														\
		return false;																	\
	}

bool captureOpenWriteDevice()
{
	XnStatus nRetVal = XN_STATUS_OK;
	NodeInfoList recordersList;
	nRetVal = g_Context.EnumerateProductionTrees(XN_NODE_TYPE_RECORDER, NULL, recordersList);
	START_CAPTURE_CHECK_RC(nRetVal, "Enumerate recorders");
	// take first
	NodeInfo chosen = *recordersList.Begin();

	g_Capture.pRecorder = new Recorder;
	nRetVal = g_Context.CreateProductionTree(chosen, *g_Capture.pRecorder);
	START_CAPTURE_CHECK_RC(nRetVal, "Create recorder");

	nRetVal = g_Capture.pRecorder->SetDestination(XN_RECORD_MEDIUM_FILE, g_Capture.csFileName);
	START_CAPTURE_CHECK_RC(nRetVal, "Set output file");

	return true;
}

void captureBrowse(int)
{
#if (XN_PLATFORM == XN_PLATFORM_WIN32)
	OPENFILENAMEA ofn;
	TCHAR* szFilter = TEXT("ONI Files (*.oni)\0")
		TEXT("*.oni\0")
		TEXT("All Files (*.*)\0")
		TEXT("*.*\0");

	ZeroMemory(&ofn, sizeof(OPENFILENAME));

	ofn.lStructSize = sizeof(OPENFILENAME);
	ofn.lpstrFilter = szFilter;
	ofn.nFilterIndex = 1;
	ofn.lpstrFile = g_Capture.csFileName;
	ofn.nMaxFile = sizeof(g_Capture.csFileName);
	ofn.lpstrTitle = TEXT("Capture to...");
	ofn.Flags = OFN_EXPLORER | OFN_NOCHANGEDIR;

	GetSaveFileName(&ofn);

	if (g_Capture.csFileName[0] != 0)
	{
		if (strstr(g_Capture.csFileName, ".oni") == NULL)
		{
			strcat(g_Capture.csFileName, ".oni");
		}
	}
#else // not Win32
	strcpy(g_Capture.csFileName, "./Captured.oni");
#endif

	// as we waited for user input, it's probably better to discard first frame (especially if an accumulating
	// stream is on, like audio).
	g_Capture.bSkipFirstFrame = true;

	captureOpenWriteDevice();
}

void captureStart(int nDelay)
{
	if (g_Capture.csFileName[0] == 0)
	{
		captureBrowse(0);
	}

	if (g_Capture.csFileName[0] == 0)
		return;

	if (g_Capture.pRecorder == NULL)
	{
		if (!captureOpenWriteDevice())
			return;
	}

	XnUInt64 nNow;
	xnOSGetTimeStamp(&nNow);
	nNow /= 1000;

	g_Capture.nStartOn = (XnUInt32)nNow + nDelay;
	g_Capture.State = SHOULD_CAPTURE;
}

void captureCloseWriteDevice()
{
	if (g_Capture.pRecorder != NULL)
	{
		g_Capture.pRecorder->Release();
		delete g_Capture.pRecorder;
		g_Capture.pRecorder = NULL;
	}
}

void captureRestart(int)
{
	captureCloseWriteDevice();
	if (captureOpenWriteDevice())
		captureStart(0);
}

void captureStop(int)
{
	if (g_Capture.State != NOT_CAPTURING)
	{
		g_Capture.State = NOT_CAPTURING;
		captureCloseWriteDevice();
	}
}


XnStatus captureFrame()
{
	XnStatus nRetVal = XN_STATUS_OK;

	if (g_Capture.State == SHOULD_CAPTURE)
	{
		XnUInt64 nNow;
		xnOSGetTimeStamp(&nNow);
		nNow /= 1000;

		// check if time has arrived
		if (nNow >= g_Capture.nStartOn)
		{
			// check if we need to discard first frame
			if (g_Capture.bSkipFirstFrame)
			{
				g_Capture.bSkipFirstFrame = false;
			}
			else
			{
				// start recording
				for (int i = 0; i < CAPTURE_NODE_COUNT; ++i)
				{
					g_Capture.nodes[i].nCapturedFrames = 0;
					g_Capture.nodes[i].bRecording = false;
				}
				g_Capture.State = CAPTURING;

				// add all captured nodes
				if (getDevice() != NULL)
				{
					nRetVal = g_Capture.pRecorder->AddNodeToRecording(*getDevice(), XN_CODEC_UNCOMPRESSED);
					START_CAPTURE_CHECK_RC(nRetVal, "add device node");
				}

				if (isDepthOn() && (g_Capture.nodes[CAPTURE_DEPTH_NODE].captureFormat != CODEC_DONT_CAPTURE))
				{
					nRetVal = g_Capture.pRecorder->AddNodeToRecording(*getDepthGenerator(), g_Capture.nodes[CAPTURE_DEPTH_NODE].captureFormat);
					START_CAPTURE_CHECK_RC(nRetVal, "add depth node");
					g_Capture.nodes[CAPTURE_DEPTH_NODE].bRecording = TRUE;
					g_Capture.nodes[CAPTURE_DEPTH_NODE].pGenerator = getDepthGenerator();
				}

				if (isImageOn() && (g_Capture.nodes[CAPTURE_IMAGE_NODE].captureFormat != CODEC_DONT_CAPTURE))
				{
					nRetVal = g_Capture.pRecorder->AddNodeToRecording(*getImageGenerator(), g_Capture.nodes[CAPTURE_IMAGE_NODE].captureFormat);
					START_CAPTURE_CHECK_RC(nRetVal, "add image node");
					g_Capture.nodes[CAPTURE_IMAGE_NODE].bRecording = TRUE;
					g_Capture.nodes[CAPTURE_IMAGE_NODE].pGenerator = getImageGenerator();
				}

				if (isIROn() && (g_Capture.nodes[CAPTURE_IR_NODE].captureFormat != CODEC_DONT_CAPTURE))
				{
					nRetVal = g_Capture.pRecorder->AddNodeToRecording(*getIRGenerator(), g_Capture.nodes[CAPTURE_IR_NODE].captureFormat);
					START_CAPTURE_CHECK_RC(nRetVal, "add IR stream");
					g_Capture.nodes[CAPTURE_IR_NODE].bRecording = TRUE;
					g_Capture.nodes[CAPTURE_IR_NODE].pGenerator = getIRGenerator();
				}

				if (isAudioOn() && (g_Capture.nodes[CAPTURE_AUDIO_NODE].captureFormat != CODEC_DONT_CAPTURE))
				{
					nRetVal = g_Capture.pRecorder->AddNodeToRecording(*getAudioGenerator(), g_Capture.nodes[CAPTURE_AUDIO_NODE].captureFormat);
					START_CAPTURE_CHECK_RC(nRetVal, "add Audio stream");
					g_Capture.nodes[CAPTURE_AUDIO_NODE].bRecording = TRUE;
					g_Capture.nodes[CAPTURE_AUDIO_NODE].pGenerator = getAudioGenerator();
				}
			}
		}
	}

	if (g_Capture.State == CAPTURING)
	{
		// There isn't a real need to call Record() here, as the WaitXUpdateAll() call already makes sure
		// recording is performed.
		nRetVal = g_Capture.pRecorder->Record();
		XN_IS_STATUS_OK(nRetVal);

		// count recorded frames
		for (int i = 0; i < CAPTURE_NODE_COUNT; ++i)
		{
			if (g_Capture.nodes[i].bRecording && g_Capture.nodes[i].pGenerator->IsDataNew())
				g_Capture.nodes[i].nCapturedFrames++;
		}
	}
	return XN_STATUS_OK;
}

void captureSetFormat(XnCodecID* pMember, XnCodecID newFormat, ProductionNode& node)
{
	if (*pMember == newFormat)
		return;

	if (g_Capture.pRecorder != NULL)
	{
		// check if it was off before
		if (*pMember == CODEC_DONT_CAPTURE)
		{
			g_Capture.pRecorder->AddNodeToRecording(node, newFormat);
		}
		// check if it is off now
		else if (newFormat == CODEC_DONT_CAPTURE)
		{
			g_Capture.pRecorder->RemoveNodeFromRecording(node);
		}
		else // just a change in compression
		{
			g_Capture.pRecorder->RemoveNodeFromRecording(node);
			g_Capture.pRecorder->AddNodeToRecording(node, newFormat);
		}
	}

	*pMember = newFormat;
}

void captureSetDepthFormat(int format)
{
	captureSetFormat(&g_Capture.nodes[CAPTURE_DEPTH_NODE].captureFormat, format, *getDepthGenerator());
}

void captureSetImageFormat(int format)
{
	captureSetFormat(&g_Capture.nodes[CAPTURE_IMAGE_NODE].captureFormat, format, *getImageGenerator());
}

void captureSetIRFormat(int format)
{
	captureSetFormat(&g_Capture.nodes[CAPTURE_IR_NODE].captureFormat, format, *getIRGenerator());
}

void captureSetAudioFormat(int format)
{
	captureSetFormat(&g_Capture.nodes[CAPTURE_AUDIO_NODE].captureFormat, format, *getAudioGenerator());
}

const char* getCodecName(NodeCodec* pNodeCodec, XnCodecID codecID)
{
	for (int i = 0; i < pNodeCodec->nValuesCount; i++)
	{
		if (pNodeCodec->pValues[i] == codecID)
		{
			return pNodeCodec->pIndexToName[i];
		}
	}
	return NULL;
}

const char* captureGetDepthFormatName()
{
	return getCodecName(&g_DepthFormat, g_Capture.nodes[CAPTURE_DEPTH_NODE].captureFormat);
}

const char* captureGetImageFormatName()
{
	return getCodecName(&g_ImageFormat, g_Capture.nodes[CAPTURE_IMAGE_NODE].captureFormat);
}

const char* captureGetIRFormatName()
{
	return getCodecName(&g_IRFormat, g_Capture.nodes[CAPTURE_IR_NODE].captureFormat);
}

const char* captureGetAudioFormatName()
{
	return getCodecName(&g_AudioFormat, g_Capture.nodes[CAPTURE_AUDIO_NODE].captureFormat);
}

void getCaptureMessage(char* pMessage)
{
	switch (g_Capture.State)
	{
	case SHOULD_CAPTURE:
	{
		XnUInt64 nNow;
		xnOSGetTimeStamp(&nNow);
		nNow /= 1000;
		sprintf(pMessage, "Capturing will start in %u seconds...", g_Capture.nStartOn - (XnUInt32)nNow);
	}
	break;
	case CAPTURING:
	{
		int nChars = sprintf(pMessage, "* Recording! Press any key or use menu to stop *\nRecorded Frames: ");
		for (int i = 0; i < CAPTURE_NODE_COUNT; ++i)
		{
			if (g_Capture.nodes[i].bRecording)
			{
				nChars += sprintf(pMessage + nChars, "%s-%d ", g_Capture.nodes[i].pGenerator->GetName(), g_Capture.nodes[i].nCapturedFrames);
			}
		}
	}
	break;
	default:
		pMessage[0] = 0;
	}
}

void getImageFileName(int num, char* csName)
{
	sprintf(csName, "%s/Image_%d.raw", CAPTURED_FRAMES_DIR_NAME, num);
}
void getImageFileNameTxt(std::time_t currTime, int num, char* csName)
{
	sprintf(csName, "%s/%s/Image_%d.%d.raw", CAPTURED_FRAMES_DIR_NAME, "Image", currTime, num);
}

void getDepthFileName(int num, char* csName)
{
	sprintf(csName, "%s/Depth_%d.raw", CAPTURED_FRAMES_DIR_NAME, num);
}
void getDepthFileNameTxt(std::time_t currTime, int num, char* csName)
{
	sprintf(csName, "%s/%s/Depth_%d.%d.raw", CAPTURED_FRAMES_DIR_NAME, "Depth", currTime, num);
}

void getIRFileName(int num, char* csName)
{
	sprintf(csName, "%s/IR_%d.raw", CAPTURED_FRAMES_DIR_NAME, num);
}


int findUniqueFileName()
{
	xnOSCreateDirectory(CAPTURED_FRAMES_DIR_NAME);

	int num = g_Capture.nCapturedFrameUniqueID;

	XnBool bExist = FALSE;
	XnStatus nRetVal = XN_STATUS_OK;
	XnChar csImageFileName[XN_FILE_MAX_PATH];
	XnChar csDepthFileName[XN_FILE_MAX_PATH];
	XnChar csIRFileName[XN_FILE_MAX_PATH];

	while (true)
	{
		// check image
		getImageFileName(num, csImageFileName);

		nRetVal = xnOSDoesFileExist(csImageFileName, &bExist);
		if (nRetVal != XN_STATUS_OK)
			break;

		if (!bExist)
		{
			// check depth
			getDepthFileName(num, csDepthFileName);

			nRetVal = xnOSDoesFileExist(csDepthFileName, &bExist);
			if (nRetVal != XN_STATUS_OK || !bExist)
				break;
		}

		if (!bExist)
		{
			// check IR
			getIRFileName(num, csIRFileName);

			nRetVal = xnOSDoesFileExist(csIRFileName, &bExist);
			if (nRetVal != XN_STATUS_OK || !bExist)
				break;
		}

		++num;
	}

	return num;
}


void safeImageAsTxt(std::string filename, const ImageMetaData* pImageMD) {
	/**
	std::fstream txtfile;
	txtfile.open(filename, std::ios::out);
	if (!txtfile.is_open()) {
		printf("Failed to open a txt file for Image");
		return;
	}
	int width = pImageMD->XRes();
	int height = pImageMD->YRes();
	txtfile << "RGB Data \n";
	txtfile << "Timestamp: " << pImageMD->Timestamp() << "\n";
	txtfile << "Size: " << width << " " << height << "\n";

	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			const RGB24Map& rgbMap = pImageMD->RGB24Map();
			XnRGB24Pixel pixel = rgbMap(x, y);
			//printf("RGB: %d, %d, %d\n", pixel.nRed, pixel.nGreen, pixel.nBlue);
			txtfile << (int)pixel.nRed << " " << (int)pixel.nGreen << " " << (int)pixel.nBlue << " ";
		}
		txtfile << "\n";
	}
	txtfile.close();
	std::cout << "TXT saved in " << filename << "\n";**/

	int width = pImageMD->XRes();
	int height = pImageMD->YRes();
	XnUInt64 timestamp = pImageMD->Timestamp();

	RawHeader header;
	header.filetype = IMAGE;
	header.width = width;
	header.height = height;
	header.timestamp = timestamp;

	std::ofstream rawFile(filename, std::ios::binary);
	if (!rawFile.is_open()) {
		printf("Failed to open a txt file for Image");
		return;
	}

	const uint8_t* const imageData = pImageMD->Data();

	rawFile.write(reinterpret_cast<char*>(&header), sizeof(header));
	rawFile.write(reinterpret_cast<const char*>(imageData), width * height * sizeof(uint8_t)*3);

	rawFile.close();

	std::cout << "RAW saved in " << filename << "\n";

}



void safeDepthAsTxt(std::string filename, const DepthMetaData* pDepthMD) {
	/**
	std::fstream txtfile;
	txtfile.open(filename, std::ios::out);
	if (!txtfile.is_open()) {
		printf("Failed to open a txt file for Image");
		return;
	}
	int width = pDepthMD->XRes();
	int height = pDepthMD->YRes();
	txtfile << "Depth Data \n";
	txtfile << "Timestamp: " << pDepthMD->Timestamp() << "\n";
	txtfile << "Size: " << width << " " << height << "\n";

	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			const DepthMap& a = pDepthMD->DepthMap();

			txtfile << (int)a(x, y) << " ";
		}
		txtfile << "\n";
	}
	txtfile.close();
	std::cout << "TXT saved in " << filename << "\n";**/
	int width = pDepthMD->XRes();
	int height = pDepthMD->YRes();
	XnUInt64 timestamp = pDepthMD->Timestamp();

	RawHeader header;
	header.filetype = DEPTH;
	header.width = width;
	header.height = height;
	header.timestamp = timestamp;

	std::ofstream rawFile(filename, std::ios::binary);
	if (!rawFile.is_open()) {
		printf("Failed to open a txt file for Image");
		return;
	}

	const uint16_t* const depthData = pDepthMD->Data();
	
	rawFile.write(reinterpret_cast<char*>(&header), sizeof(header));
	rawFile.write(reinterpret_cast<const char*>(depthData), width * height * sizeof(uint16_t));

	rawFile.close();

	std::cout << "Raw saved in " << filename << "\n";
}

void recordFramesTxt(int) {
	while (!closeRecording) {
		if (isCapturingTxt && hasNewFrame > 60/captureFrameRate) {
			captureSingleFrame(0);
			hasNewFrame = 0;
		}
		if (closeCapturingTxt) {

			XnChar fileNameDepth[XN_FILE_MAX_PATH];
			sprintf(fileNameDepth, "%s/depth.txt", CAPTURED_FRAMES_DIR_NAME);
			std::ofstream depthFileTxt(fileNameDepth);
			if (!depthFileTxt.is_open()) {
				std::cerr << "error saving the depth txt file";
				return;
			}
			depthFileTxt << "# depth maps\n";
			depthFileTxt << "# file: '" << CAPTURED_FRAMES_DIR_NAME << "'\n";
			depthFileTxt << "# timestamp filename\n";
			for (std::tuple<std::string, std::string>& tuple : listOfDepthNamesTxt) {
				std::string timestamp, fileName;
				std::tie(timestamp, fileName) = tuple;
				depthFileTxt << timestamp << " Depth/" << fileName << "\n";
			}
			depthFileTxt.close();


			// writing file with all image filenames and timestamps
			XnChar fileNameImage[XN_FILE_MAX_PATH];
			sprintf(fileNameImage, "%s/rgb.txt", CAPTURED_FRAMES_DIR_NAME);
			std::ofstream imageFileTxt(fileNameImage);
			if (!imageFileTxt.is_open()) {
				std::cerr << "error saving the image txt file";
				return;
			}

			imageFileTxt << "# color imges\n";
			imageFileTxt << "# file: '" << CAPTURED_FRAMES_DIR_NAME << "'\n";
			imageFileTxt << "# timestamp filename\n";
			for (std::tuple<std::string, std::string>& tuple : listOfImageNamesTxt) {
				std::string timestamp, fileName;
				std::tie(timestamp, fileName) = tuple;
				imageFileTxt << timestamp << " Image/" << fileName << "\n";
			}
			imageFileTxt.close();

			closeCapturingTxt = false;
		}
	}
	if (closeCapturingTxt) {

		XnChar fileNameDepth[XN_FILE_MAX_PATH];
		sprintf(fileNameDepth, "%s/depth.txt", CAPTURED_FRAMES_DIR_NAME);
		std::ofstream depthFileTxt(fileNameDepth);
		if (!depthFileTxt.is_open()) {
			std::cerr << "error saving the depth txt file";
			return;
		}
		depthFileTxt << "# depth maps\n";
		depthFileTxt << "# file: '" << CAPTURED_FRAMES_DIR_NAME << "'\n";
		depthFileTxt << "# timestamp filename\n";
		for (std::tuple<std::string, std::string>& tuple : listOfDepthNamesTxt) {
			std::string timestamp, fileName;
			std::tie(timestamp, fileName) = tuple;
			depthFileTxt << timestamp << " Depth/" << fileName << "\n";
		}
		depthFileTxt.close();


		// writing file with all image filenames and timestamps
		XnChar fileNameImage[XN_FILE_MAX_PATH];
		sprintf(fileNameImage, "%s/rgb.txt", CAPTURED_FRAMES_DIR_NAME);
		std::ofstream imageFileTxt(fileNameImage);
		if (!imageFileTxt.is_open()) {
			std::cerr << "error saving the image txt file";
			return;
		}

		imageFileTxt << "# color imges\n";
		imageFileTxt << "# file: '" << CAPTURED_FRAMES_DIR_NAME << "'\n";
		imageFileTxt << "# timestamp filename\n";
		for (std::tuple<std::string, std::string>& tuple : listOfImageNamesTxt) {
			std::string timestamp, fileName;
			std::tie(timestamp, fileName) = tuple;
			imageFileTxt << timestamp << " Image/" << fileName << "\n";
		}
		imageFileTxt.close();

		closeCapturingTxt = false;
	}

}

void saveTxtOverview() {
	// writing file with all image filenames and timestamps

	XnChar fileNameDepth[XN_FILE_MAX_PATH];
	sprintf(fileNameDepth, "%s/depth.txt", CAPTURED_FRAMES_DIR_NAME);
	std::ofstream depthFileTxt(fileNameDepth);
	if (!depthFileTxt.is_open()) {
		std::cerr << "error saving the depth txt file";
		return;
	}
	depthFileTxt << "# depth maps\n";
	depthFileTxt << "# file: '" << CAPTURED_FRAMES_DIR_NAME << "'\n";
	depthFileTxt << "# timestamp filename\n";
	for (std::tuple<std::string, std::string>& tuple : listOfDepthNamesTxt) {
		std::string timestamp, fileName;
		std::tie(timestamp, fileName) = tuple;
		depthFileTxt << timestamp << " Depth/" << fileName << "\n";
	}
	depthFileTxt.close();


	// writing file with all image filenames and timestamps
	XnChar fileNameImage[XN_FILE_MAX_PATH];
	sprintf(fileNameImage, "%s/rgb.txt", CAPTURED_FRAMES_DIR_NAME);
	std::ofstream imageFileTxt(fileNameImage);
	if (!imageFileTxt.is_open()) {
		std::cerr << "error saving the image txt file";
		return;
	}

	imageFileTxt << "# color imges\n";
	imageFileTxt << "# file: '" << CAPTURED_FRAMES_DIR_NAME << "'\n";
	imageFileTxt << "# timestamp filename\n";
	for (std::tuple<std::string, std::string>& tuple : listOfImageNamesTxt) {
		std::string timestamp, fileName;
		std::tie(timestamp, fileName) = tuple;
		imageFileTxt << timestamp << " Image/" << fileName << "\n";
	}
	imageFileTxt.close();

	closeCapturingTxt = false;
}

void setHasNewFrame(int) {
	if(isCapturing)
		++hasNewFrame;
}

void closeRecordingTxt(int) {
	printf("DONE!\n");
	closeRecording = true;
}

void setCapturingTxt(int capturing) {
	if (capturing == 1)
		isCapturingTxt = true;
	else if (capturing == 0) {
		if (isCapturingTxt)
			closeCapturingTxt = true;
		isCapturingTxt = false;
	}
	else
		printf("Wrong use of setCapturingTxt");
}

void captureSingleFrame(int)
{
	int num = findUniqueFileName();

	//XnChar csImageFileName[XN_FILE_MAX_PATH];
	//XnChar csDepthFileName[XN_FILE_MAX_PATH];

	//XnChar csIRFileName[XN_FILE_MAX_PATH];

	//getImageFileName(num, csImageFileName);
	//getDepthFileName(num, csDepthFileName);
	//getIRFileName(num, csIRFileName);




	// create names for files depth and image
	XnChar csImageFileNameTxt[XN_FILE_MAX_PATH];
	XnChar csDepthFileNameTxt[XN_FILE_MAX_PATH];
	const ImageMetaData* pImageMD = getImageMetaData();
	const DepthMetaData* pDepthMD = getDepthMetaData();
	std::time_t curr_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	getImageFileNameTxt(curr_time, pImageMD->Timestamp(), csImageFileNameTxt);
	getDepthFileNameTxt(curr_time, pDepthMD->Timestamp(), csDepthFileNameTxt);

	// safe filenames and timestamp for overview txt files
	XnChar tempTimestampName[XN_FILE_MAX_PATH];
	XnChar tempFileName[XN_FILE_MAX_PATH];
	sprintf(tempTimestampName, "%d", pImageMD->Timestamp());
	sprintf(tempFileName, "Image_%d.%d.raw", curr_time, pImageMD->Timestamp());
	listOfImageNamesTxt.push_back(std::make_tuple(tempTimestampName, tempFileName));

	XnChar tempTimestampName2[XN_FILE_MAX_PATH];
	sprintf(tempTimestampName2, "%d", pDepthMD->Timestamp());
	sprintf(tempFileName, "Depth_%d.%d.raw", curr_time, pDepthMD->Timestamp());
	listOfDepthNamesTxt.push_back(std::make_tuple(tempTimestampName2, tempFileName));

	// create Directories
	xnOSCreateDirectory(CAPTURED_FRAMES_DIR_NAME);

	XnChar dirNameDepth[XN_FILE_MAX_PATH];
	sprintf(dirNameDepth, "%s/Depth/", CAPTURED_FRAMES_DIR_NAME);
	xnOSCreateDirectory(dirNameDepth);
	XnChar dirNameImage[XN_FILE_MAX_PATH];
	sprintf(dirNameImage, "%s/Image/", CAPTURED_FRAMES_DIR_NAME);
	xnOSCreateDirectory(dirNameImage);


	if (pImageMD != NULL)
	{
		safeImageAsTxt(csImageFileNameTxt, pImageMD);
		//xnOSSaveFile(csImageFileName, pImageMD->Data(), pImageMD->DataSize());
	}

	//const IRMetaData* pIRMD = getIRMetaData();
	//if (pIRMD != NULL)
	//{
	//	xnOSSaveFile(csIRFileName, pIRMD->Data(), pIRMD->DataSize());
	//}


	if (pDepthMD != NULL)
	{
		safeDepthAsTxt(csDepthFileNameTxt, pDepthMD);
		//xnOSSaveFile(csDepthFileName, pDepthMD->Data(), pDepthMD->DataSize());
	}

	g_Capture.nCapturedFrameUniqueID = num + 1;

	displayMessage("Frames saved with ID %d", num);
}
