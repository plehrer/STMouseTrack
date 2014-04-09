//
//  main.cpp
//  PLGlobalCoordinates
//
//  Created by Peter Lehrer on 3/17/14.

//The tracking code is written by  Kyle Hounslow 2013 (objectTrackingTutorial.cpp) http://www.youtube.com/watch?v=bSeFrPrqZ2A
// https://dl.dropbox.com/u/28096936/tuts/objectTrackingTut.cpp

//3D World coordinates code written by Peter Lehrer 2014

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.

#include <sstream>
#include <string>
#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cv.h>

using namespace cv;
using namespace std;

//default capture width and height
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 720;

//const string trackbarWindowName = "HSV Threshold Values";
const string trackbarSwitchName = "Matrix Math";

int xRight=0, yRight=0;
int xLeft=0, yLeft=0;
bool trackObjectCameraRight = false;
bool trackObjectCameraLeft = false;

char key = 0;

void myMouseCallbackLeft(int, int, int, int, void*);
void myMouseCallbackRight(int, int, int, int, void*);
void drawObject(int, int, int, int, int, Mat&);
void drawObject(int,int,Mat&);
string intToString(int number);
void on_trackbar_switch(int, void*);


int main(int argc, char* argv[])
{
	//some boolean variables for different functionality within this
	//program
	int matrixMath = 0;  // use perspectiveTransform to do matrix math
	
	//Matrix to store each frame of the webcam feed
	Mat captureFeedRight;
	Mat captureFeedLeft;
	Mat captureFeedRightR, captureFeedLeftR; //to store rectified images
	
	//x and y values for the location of the object
	int x_3D = 0, y_3D = 0, z_3D=0;
	double w=0.0;  // divisor to find 3D coordinates
	//d is the disparity in x and will be used to calculate the distance
	double d = 0;
	
	// open the default cameras
	VideoCapture captureRight(0);  //right camera
	VideoCapture captureLeft(1);  //left camera
	
	// check for failure of first camera
	if (!captureRight.isOpened()) {
		printf("Failed to open video device 1 or video file!\n");
		return 1;
	}
	
	// check for failure of second camera
	if (!captureLeft.isOpened()) {
		printf("Failed to open video device 2 or video file!\n");
		return 1;
	}
	
	// Set Capture device properties.
	captureRight.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	captureRight.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	
	captureLeft.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	captureLeft.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	
	//create window for trackbar switch
	namedWindow(trackbarSwitchName,0);
	
	namedWindow("Right Camera Tracking", 0);   // right xRight
	namedWindow("Left Camera Tracking", 0);  // left xLeft
	
	moveWindow("Right Camera Tracking", 450, 0);
	moveWindow("Left Camera Tracking", 0, 0);
	

	createTrackbar( "On/Off", trackbarSwitchName, &matrixMath, 1, on_trackbar_switch );
	
	moveWindow(trackbarSwitchName, 10, 775);
	
	startWindowThread();
	
	//*******************************************
	// Load xml files created by Stereo calibration
	CvMat* Q   = (CvMat*) cvLoad("/Users/shimon/opencv/STMouseTrack/Q.xml");
	CvMat* mx1 = (CvMat*) cvLoad("/Users/shimon/opencv/STMouseTrack/mx1.xml");
	CvMat* my1 = (CvMat*) cvLoad("/Users/shimon/opencv/STMouseTrack/my1.xml");
	CvMat* mx2 = (CvMat*) cvLoad("/Users/shimon/opencv/STMouseTrack/mx2.xml");
	CvMat* my2 = (CvMat*) cvLoad("/Users/shimon/opencv/STMouseTrack/my2.xml");
	//*******************************************
	
	if (!Q || !mx1 || !my1 || !mx2 || !my2) {
		cout << "Error loading 1 or more matrix xml files\n";
		return -1;
	}
	
	//Convert CvMat to Mat
	Mat Mx1 = mx1;
	Mat My1 = my1;
	Mat Mx2 = mx2;
	Mat My2 = my2;
	Mat Q_mat = Q;
	
	vector<Point3d> pointsXYD;
	vector<Point3d> result3DPoints;
	
	setMouseCallback("Left Camera Tracking", myMouseCallbackLeft, &captureFeedLeftR);
	setMouseCallback("Right Camera Tracking", myMouseCallbackRight, &captureFeedRightR);
	
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while( key != 'q'){
		// get a new frame from camera
		captureRight >> captureFeedRight;    //retrieve image from right camera  camera1
		captureLeft >> captureFeedLeft;   //retrieve image from left camera, camera2
		
		// correct for rectification and lens distortion
		remap(captureFeedRight, captureFeedRightR, Mx2, My2, INTER_LINEAR);  // right camera
		remap(captureFeedLeft, captureFeedLeftR, Mx1, My1, INTER_LINEAR);  // left camera
				
		pointsXYD.clear();
		
		if (trackObjectCameraLeft && !trackObjectCameraRight) {
			drawObject(xLeft, yLeft, captureFeedLeftR);
		}
		else if(trackObjectCameraLeft && trackObjectCameraRight){
			//Calculate distance
			// use perspectiveTransfor function to do matrix_math
			if (!matrixMath)
			{
				d = xLeft - xRight;
				pointsXYD.push_back(Point3d(xLeft, yLeft, d));
				perspectiveTransform(pointsXYD, result3DPoints, Q_mat);
				z_3D = result3DPoints.at(0).z;
				x_3D = result3DPoints.at(0).x;
				y_3D = result3DPoints.at(0).y;
			}
			else // do the matrix math explicitly with out the perspectiveTransform function
			{
				d = xLeft - xRight;
				x_3D = xLeft * Q_mat.at<double>(0,0) + Q_mat.at<double>(0,3);
				y_3D = yLeft * Q_mat.at<double>(1,1) + Q_mat.at<double>(1,3);
				z_3D = Q_mat.at<double>(2,3); // assign focal lenght
				w = d * Q_mat.at<double>(3,2) + Q_mat.at<double>(3,3); // Homogenuous coordinate scaling
				x_3D = x_3D / w;
				y_3D = y_3D / w;
				z_3D = z_3D / w;
			}
			//draw object location on screen
			drawObject(x_3D, y_3D, xRight, yRight, z_3D, captureFeedRightR);
			drawObject(x_3D, y_3D, xLeft, yRight, z_3D, captureFeedLeftR);
		}
		
		imshow("Right Camera Tracking", captureFeedRightR);
		imshow("Left Camera Tracking", captureFeedLeftR);
		
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		key = waitKey(30);
	}
	
	destroyAllWindows();
	return 0;
}

void myMouseCallbackLeft(int event, int x, int y, int flags, void *param) {
	switch (event) {
		case EVENT_LBUTTONDOWN:
			xLeft = x;
			yLeft = y;
			cout << "Left camera (x,y): (" << xLeft << "," << yLeft << ")\n";
			trackObjectCameraLeft = true;
			trackObjectCameraRight = false;
			break;
		default:
			break;
	}
	
}

void myMouseCallbackRight(int event, int x, int y, int flags, void *param) {
	switch (event) {
		case EVENT_LBUTTONDOWN:
			xRight = x;
			yRight = y;
			cout << "Right camera (x,y): (" << xRight << "," << yRight << ")\n";
			if (!trackObjectCameraRight) {
				trackObjectCameraRight = true;
			}
			else trackObjectCameraRight = false;
			
			break;
		default:
			break;
	}
	
}


void drawObject(int x_3D, int y_3D, int x, int y, int z, Mat &frame){
	
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!
	
	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)
	
	circle(frame,Point(x,y),20,Scalar(0,255,0),2);
	if(y-25>0)
		line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
	else
		line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
	if(y+25<FRAME_HEIGHT)
		line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
	else
		line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
	if(x-25>0)
		line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
	else
		line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
	if(x+25<FRAME_WIDTH)
		line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
	else
		line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);
	
	putText(frame,intToString(x_3D)+", "+intToString(y_3D)+", "+intToString(z),Point(x,y+30),1,1,Scalar(0,255,0),2);
	putText(frame,"x = "+intToString(x_3D)+","+"y = "+intToString(y_3D)+", z = "+intToString(z)+" centimeters",Point(5,50),2,2,Scalar(0,255,0),2);
	putText(frame,"3D Coordiantes Determined",Point(10,700),1,2,Scalar(0,255,0),4);
}


void drawObject(int x, int y, Mat &frame){
	
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!
	
	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)
	
	circle(frame,Point(x,y),20,Scalar(0,255,0),2);
	if(y-25>0)
		line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
	else
		line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
	if(y+25<FRAME_HEIGHT)
		line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
	else
		line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
	if(x-25>0)
		line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
	else
		line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
	if(x+25<FRAME_WIDTH)
		line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
	else
		line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);
	
	putText(frame,intToString(x)+", "+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);
	putText(frame,"Left click on corressponding object in right camera.",Point(10,700),1,2,Scalar(0,0,255),4);
}

string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void on_trackbar_switch(int, void*) {
	
}



