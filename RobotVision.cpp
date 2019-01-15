#include <iostream>
#include <thread>
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include "GripOutput.h"
#include <SmartDashboard/SmartDashboard.h>
#include <Joystick.h>
//#include <RobotDrive.h>
#include <Timer.h>
// #include <ctrlib/CanTalon.h>
#include <ctre/Phoenix.h>
#include "DriveBase.h"
#define FPSOFCAMERA 20
//all references to the white object in the input will be referred to as simply "Object", for simplicity's sake
//GLOBAL VARIABLES
int object_max_x;
int object_max_y;
int object_min_x;
int object_min_x;

class Robot: public frc::IterativeRobot {

public:
	Robot() {
		myRobot = new DriveBase();
		timer.Start();
	}

private:

	DriveBase* myRobot;
	frc::Timer timer;
	double exposure;
	double whitebalance;

	static void VisionThread(VisionThread, &object_found) {

		// Get the USB camera from CameraServer
		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
		// Set the resolution
		camera.SetResolution(160, 120);

		// Stop the camera flicker
		camera.SetFPS(FPSOFCAMERA);
		// Fix the camera exposure
		camera.SetExposureManual(2);
		// Fix the colour cast
		camera.SetWhiteBalanceManual(4000);

		// Get a CvSink. This will capture Mats from the Camera
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();

		// Setup a CvSource. This will send images back to the Dashboard. Camera selection is CubeDetect
		cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("CubeDetect", 160, 120);
		// Match the frame rate to the camera frame rate
		outputStream.SetFPS(10);

		// Setup an instance of the vision processing pipeline you created with the GRIP tool.
		grip::GripOutput* MyPipeline = new grip::GripOutput();

		// Mat is to be re-used, mat is a mat
		cv::Mat mat;

		while (true) {
			// Tell the CvSink to grab a frame from the camera and put it
			// in the source mat.  If there is an error notify the output.
			if (cvSink.GrabFrame(mat) == 0) {
				// Send the output the error.
				outputStream.NotifyError(cvSink.GetError());
				// skip the rest of the current iteration
				continue;
			}

			// Run the whole vision processing process on the frame we just grabbed
			MyPipeline->Process(mat);

			// Copy the resulting image back to mat
			mat = *(MyPipeline->GetHslThresholdOutput()); //!! Changes to first image processing process of grip
			//mat = cv::Scalar(0);


			object_min_y = mat.rows - 1; //since the pixels start from 0, hence we are setting the largest possible value
			object_max_y = 0;

			object_min_x = mat.cols - 1;
			object_max_x = 0;
			const int thresh = 10;

			// images from USB cameras are in Blue:Green:Red (bgr) format. Each pixel has 3 unsigned
			// 8-bit values. Define a type that allows us to access each colour.
			struct Pixbgr {
				unsigned char b:8; //only blue has to be used since the picture is black and white
			};
			for(int i = 0; i < mat.rows; i++) {
				// const CV_8UC3 *Mi = mat.ptr<CV_8UC3>(i);
				// Make a pixel pointer that lets us see the red green and blue components
				const struct Pixbgr* Mi = mat.ptr<struct Pixbgr>(i);
				for(int j = 0; j < mat.cols; j++)
					// If average of blue is large enough, then we've found a blob
					if (Mi[j].b > thresh) {
						if (j < object_min_x)
							object_min_x = j;
					    if (j > object_max_x)
					    	object_max_x = j;
					    if (i < object_min_y)
					    	object_min_y = i;
					    if (i > object_max_y)
					    	object_max_y = i;
					    }
			}
			if ((object_max_x > (object_min_x + 2)) && (object_max_y > (object_min_y +2))) { //will create offsets for blobs bigger than 3x3 pixels
				// Draw a white rectangle around the blob
				*vision_found_object = true;
				rectangle(mat, cv::Point(object_min_x, object_min_y), cv::Point(object_max_x, object_max_y),
						cv::Scalar(255, 255, 255), 1);

			//things such as direction of object will be calculated here
			}
			else {
				*vision_found_object = false;
			}
			outputStream.PutFrame(mat);
		}
	}

	void AutonomousInit() override {
		//timer.Reset();
		//timer.Start();
	}

	void AutonomousPeriodic() override {

//this will run after RobotInit because of magic robot code

	}


	void RobotInit() {
		// We need to run our vision program in a separate Thread.
		SmartDashboard::init();
		std::thread vision_thread(VisionThread, &object_found);
		vision_thread.detach();
	}

};
