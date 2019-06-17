#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;


int acquireImages(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice)
{
	int result = 0;
	std::cout << "Beginning acquisition node." << std::endl; 

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera_array/cam0/image_raw", 1);

	try
	{
		//
		// Set acquisition mode to continuous
		//
		// *** NOTES ***
		// Because the example acquires and saves 10 images, setting acquisition 
		// mode to continuous lets the example finish. If set to single frame
		// or multiframe (at a lower number of images), the example would just
		// hang. This would happen because the example has been written to
		// acquire 10 images while the camera would have been programmed to 
		// retrieve less than that.
		// 
		// Setting the value of an enumeration node is slightly more complicated
		// than other node types. Two nodes must be retrieved: first, the 
		// enumeration node is retrieved from the nodemap; and second, the entry
		// node is retrieved from the enumeration node. The integer value of the
		// entry node is then set as the new value of the enumeration node.
		//
		// Notice that both the enumeration and the entry nodes are checked for
		// availability and readability/writability. Enumeration nodes are
		// generally readable and writable whereas their entry nodes are only
		// ever readable.
		// 
		// Retrieve enumeration node from nodemap
		CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
		if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
		{
			std::cout << "Unable to set acquisition to continuous." << std::endl;
			return -1;
		}

		// Retrieve entry node from enumeration node
		CEnumEntryPtr ptrAcquisitionModeContinuous = 
			ptrAcquisitionMode->GetEntryByName("Continuous");
		if (!IsAvailable(ptrAcquisitionModeContinuous) ||
				!IsReadable(ptrAcquisitionModeContinuous))
		{
			std::cout << "Unable to set acquisition to continuous." << std::endl;
			return -1;
		}

		Spinnaker::GenApi::CEnumerationPtr exposureAuto = pCam->GetNodeMap().GetNode("ExposureAuto");
		exposureAuto->SetIntValue(exposureAuto->GetEntryByName("Off")->GetValue());

		Spinnaker::GenApi::CEnumerationPtr exposureMode = pCam->GetNodeMap().GetNode("ExposureMode");
		exposureMode->SetIntValue(exposureMode->GetEntryByName("Timed")->GetValue());

		Spinnaker::GenApi::CFloatPtr exposureTime = pCam->GetNodeMap().GetNode("ExposureTime");
		exposureTime->SetValue(1305);

		// Retrieve integer value from entry node
		int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

		// Set integer value from entry node as new value of enumeration node
		ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

		std::cout << "Set acquisition to continuous." << std::endl;

		//
		// Begin acquiring images
		//
		// *** NOTES ***
		// What happens when the camera begins acquiring images depends on the
		// acquisition mode. Single frame captures only a single image, multi 
		// frame catures a set number of images, and continuous captures a 
		// continuous stream of images. Because the example calls for the 
		// retrieval of 10 images, continuous mode has been set.
		// 
		// *** LATER ***
		// Image acquisition must be ended when no more images are needed.
		//
		pCam->BeginAcquisition();
		std::cout << "Beginning acquistion." << std::endl;

		//
		// Retrieve device serial number for filename
		//
		// *** NOTES ***
		// The device serial number is retrieved in order to keep cameras from 
		// overwriting one another. Grabbing image IDs could also accomplish
		// this.
		//
		gcstring deviceSerialNumber("");
		CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
		if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
		{
			deviceSerialNumber = ptrStringSerial->GetValue();
			std::cout << "Serial number: " << deviceSerialNumber << std::endl;
		}

		// Retrieve, convert, and save images
		const unsigned int k_numImages = 10;

		// for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++)
		while (true)
		{
			try
			{
				//
				// Retrieve next received image
				//
				// *** NOTES ***
				// Capturing an image houses images on the camera buffer. Trying
				// to capture an image that does not exist will hang the camera.
				//
				// *** LATER ***
				// Once an image from the buffer is saved and/or no longer 
				// needed, the image must be released in order to keep the 
				// buffer from filling up.
				//
				std::cout << std::endl << "Attempting to get next image." << std::endl;
				ImagePtr pResultImage = pCam->GetNextImage();
				std::cout << "Next image received." << std::endl;

				//
				// Ensure image completion
				//
				// *** NOTES ***
				// Images can easily be checked for completion. This should be
				// done whenever a complete image is expected or required.
				// Further, check image status for a little more insight into
				// why an image is incomplete.
				//
				if (pResultImage->IsIncomplete())
				{

					std::cout << "Incomplete image with status " << 
						pResultImage->GetImageStatus() << std::endl;
				}
				else
				{
					//
					// Print image information; height and width recorded in pixels
					//
					// *** NOTES ***
					// Images have quite a bit of available metadata including
					// things such as CRC, image status, and offset values, to
					// name a few.
					//
					size_t width = pResultImage->GetWidth();
					size_t height = pResultImage->GetHeight();

					//
					// Convert images to OpenCV
					//
					// *** NOTES ***
					// Images can be converted between pixel formats by using 
					// the appropriate enumeration value. Unlike the original 
					// image, the converted one does not need to be released as 
					// it does not affect the camera buffer.
					//
					// When converting images, color processing algorithm is an
					// optional parameter.
					// 
					ImagePtr convertedImage = pResultImage->Convert(Spinnaker::PixelFormat_BGR8,
							Spinnaker::HQ_LINEAR); 
					cv::Mat img(convertedImage->GetHeight(), convertedImage->GetWidth(), 
							CV_8UC3, convertedImage->GetData(), convertedImage->GetStride()); 
					cv::Mat out;
					img.copyTo(out);

					// Publish image.
					sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
							"bgr8", out).toImageMsg();
					pub.publish(msg);
				}
				//
				// Release image
				//
				// *** NOTES ***
				// Images retrieved directly from the camera (i.e. non-converted
				// images) need to be released in order to keep from filling the
				// buffer.
				//
				pResultImage->Release();
			}
			catch (Spinnaker::Exception &e)
			{
				std::cout << "Error: " << e.what() << std::endl;
				result = -1;
			}
		}

		//
		// End acquisition
		//
		// *** NOTES ***
		// Ending acquisition appropriately helps ensure that devices clean up
		// properly and do not need to be power-cycled to maintain integrity.
		//
		pCam->EndAcquisition();
	}
	catch (Spinnaker::Exception &e)
	{
		std::cout << "Error: " << e.what() << std::endl;
		result = -1;
	}

	return result;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int printDeviceInfo(INodeMap & nodeMap)
{
	int result = 0;

	std::cout << std::endl << "*** DEVICE INFORMATION ***" << std::endl << std::endl;
	try
	{
		FeatureList_t features;
		CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
		if (IsAvailable(category) && IsReadable(category))
		{
			category->GetFeatures(features);
			FeatureList_t::const_iterator it;
			for (it = features.begin(); it != features.end(); ++it)
			{
				CNodePtr pfeatureNode = *it;
				std::cout << pfeatureNode->GetName() << " : ";
				CValuePtr pValue = (CValuePtr)pfeatureNode;
				std::cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
				std::cout << std::endl;
			}
		}
		else
		{
			std::cout << "Device control information not available." << std::endl;
		}
	}
	catch (Spinnaker::Exception &e)
	{
		std::cout << "Error: " << e.what() << std::endl;
		result = -1;
	}

	return result;
}

// This function acts as the body of the example; please see NodeMapInfo example 
// for more in-depth comments on setting up cameras.
int runSingleCamera(CameraPtr pCam)
{
	int result = 0;
	try
	{
		// Retrieve TL device nodemap and print device information
		INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
		// result = printDeviceInfo(nodeMapTLDevice);

		// Initialize camera
		pCam->Init();

		// Retrieve GenICam nodemap
		INodeMap & nodeMap = pCam->GetNodeMap();

		// Acquire images
		result = result | acquireImages(pCam, nodeMap, nodeMapTLDevice);

		// Deinitialize camera
		pCam->DeInit();
	}
	catch (Spinnaker::Exception &e)
	{
		std::cout << "Error: " << e.what() << std::endl;
		result = -1;
	}
	return result;
}

// Example entry point; please see Enumeration example for more in-depth 
// comments on preparing and cleaning up the system.
int main(int argc, char** argv)
{
	int result = 0;
	ros::init(argc, argv, "acquisition");

	// Retrieve singleton reference to system object
	SystemPtr system = System::GetInstance();

	// Retrieve list of cameras from the system
	CameraList camList = system->GetCameras();
	unsigned int numCameras = camList.GetSize();
	std::cout << "Number of cameras detected: " << numCameras << std::endl;

	// Finish if there are no cameras
	if (numCameras == 0)
	{
		// Clear camera list before releasing system
		camList.Clear();
		// Release system
		system->ReleaseInstance();
		std::cout << "Not enough cameras!" << std::endl;
		std::cout << "Done! Press Enter to exit..." << std::endl;
		getchar();

		return -1;
	}

	//
	// Create shared pointer to camera
	//
	// *** NOTES ***
	// The CameraPtr object is a shared pointer, and will generally clean itself
	// up upon exiting its scope. However, if a shared pointer is created in the
	// same scope that a system object is explicitly released (i.e. this scope),
	// the reference to the shared point must be broken manually.
	//
	// *** LATER ***
	// Shared pointers can be terminated manually by assigning them to NULL.
	// This keeps releasing the system from throwing an exception.
	//
	CameraPtr pCam = NULL;
	// Run example on each camera
	for (unsigned int i = 0; i < numCameras; i++)
	{
		// Select camera
		pCam = camList.GetByIndex(i);

		// Run example
		result = result | runSingleCamera(pCam);
	}

	//
	// Release reference to the camera
	//
	// *** NOTES ***
	// Had the CameraPtr object been created within the for-loop, it would not
	// be necessary to manually break the reference because the shared pointer
	// would have automatically cleaned itself up upon exiting the loop.
	//
	pCam = NULL;
	// Clear camera list before releasing system
	camList.Clear();
	// Release system
	system->ReleaseInstance();
	std::cout << std::endl << "Done! Press Enter to exit..." << std::endl;
	getchar();
	return result;
}

