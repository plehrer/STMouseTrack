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
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 10;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

const string trackbarWindowName = "HSV Threshold Values";
const string trackbarSwitchName = "HSV Red Yellow Blue Switch";

char key = 0;

void myMouseCallback(int, int, int, int, void*);
void drawObject(int, int, int, int, int, Mat&);

string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
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
	putText(frame,"Tracking Object",Point(5,700),1,2,Scalar(0,255,0),4);
}
void morphOps(Mat &thresh){
	
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
	
	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);
	
	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
	
}
bool trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){
	
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		unsigned long numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
				//count++;
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
				
				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we save a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
				}
				else objectFound = false;
			}
		}
		else putText(cameraFeed,"TOO MUCH NOISE!",Point(0,50),2,2,Scalar(0,0,255),4);
	}
	//cout << "Count for loop: " << count << endl;
	return objectFound;
}

int main(int argc, char* argv[])
{
	//some boolean variables for different functionality within this
	//program
//	bool trackObjects = true;
//	bool useMorphOps = true;
	bool matrixMath = false;  // use perspectiveTransform to do matrix math
//	bool trackObjectCamera1 = false, trackObjectCamera2 = false;
	
	//Matrix to store each frame of the webcam feed
	Mat captureFeedRight;
	Mat captureFeedLeft;
	Mat captureFeedRightR, captureFeedLeftR; //to store rectified images
	//matrix storage for HSV image
//	Mat HSV1;
//	Mat HSV2;
	//matrix storage for binary threshold image
//	Mat thresholdRight;
//	Mat thresholdLeft;
	//x and y values for the location of the object
	int xRight=0, yRight=0;
	int xLeft=0, yLeft=0;
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
	
//	namedWindow("Left Camera HSV Smoothed and Thresholded Video", 0);
//	namedWindow("Right Camera HSV Smoothed and Thresholded Video", 0);
	
	//create window for trackbars
//	namedWindow(trackbarWindowName,0);
//	namedWindow(trackbarSwitchName,0);
	
	namedWindow("Left Camera Tracking", 0);  // left xLeft
	namedWindow("Right Camera Tracking", 0);   // right xRight
	
//	moveWindow("Right Camera HSV Smoothed and Thresholded Video", 0, 450);
//	moveWindow("Left Camera HSV Smoothed and Thresholded Video", 800, 450);
	
	moveWindow("Right Camera Tracking", 0, 0);
	moveWindow("Left Camera Tracking", 800, 0);
	
	//create memory to store trackbar name on window
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH),
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//
	//                                  ---->    ---->     ---->
//	createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, 255, on_trackbar );
//	createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, 255, on_trackbar );
//	createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, 255, on_trackbar );
//	createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, 255, on_trackbar );
//	createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, 255, on_trackbar );
//	createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, 255, on_trackbar );
//	
//	createTrackbar( "M/R/B/Y", trackbarSwitchName, &HSV, 3, on_trackbar_switch );
	
//	moveWindow(trackbarWindowName, 75, 170);
//	moveWindow(trackbarSwitchName, 0, 170);
	
	startWindowThread();
	
	//*******************************************
	// Load xml files created by Stereo calibration
	CvMat* Q   = (CvMat*) cvLoad("/Users/shimon/opencv/PLGlobalCoordinates/Q.xml");
	CvMat* mx1 = (CvMat*) cvLoad("/Users/shimon/opencv/PLGlobalCoordinates/mx1.xml");
	CvMat* my1 = (CvMat*) cvLoad("/Users/shimon/opencv/PLGlobalCoordinates/my1.xml");
	CvMat* mx2 = (CvMat*) cvLoad("/Users/shimon/opencv/PLGlobalCoordinates/mx2.xml");
	CvMat* my2 = (CvMat*) cvLoad("/Users/shimon/opencv/PLGlobalCoordinates/my2.xml");
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
	
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while( key != 'q'){
		// get a new frame from camera
		captureRight >> captureFeedRight;    //retrieve image from right camera  camera1
		captureLeft >> captureFeedLeft;   //retrieve image from left camera, camera2
		
		// correct for rectification and lens distortion
		remap(captureFeedRight, captureFeedRightR, Mx2, My2, INTER_LINEAR);  // right camera
		remap(captureFeedLeft, captureFeedLeftR, Mx1, My1, INTER_LINEAR);  // left camera
		
		//convert frame from BGR to HSV colorspace
		//Converting color space to HSV makes it much easier to filter colors in the HSV color-space
//		cvtColor(captureFeedRightR, HSV1, CV_BGR2HSV);
//		cvtColor(captureFeedLeftR, HSV2, CV_BGR2HSV);
//		
//		//filter HSV image between values and store filtered image to
//		//threshold matrix
//		inRange(HSV1,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),thresholdRight);
//		inRange(HSV2,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),thresholdLeft);
//		
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
//		if(useMorphOps) {
//			morphOps(thresholdRight);
//			morphOps(thresholdLeft);
//		}
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
//		if(trackObjects) {
//			trackObjectCamera1 = trackFilteredObject(xRight,yRight,thresholdRight,captureFeedRightR);
//			trackObjectCamera2 = trackFilteredObject(xLeft,yLeft,thresholdLeft,captureFeedLeftR);
//		}
		
		pointsXYD.clear();
		
//		if(trackObjectCamera1 && trackObjectCamera2){
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
//		}
		
		imshow("Right Camera Tracking", captureFeedRightR);
		imshow("Left Camera Tracking", captureFeedLeftR);
		
//		imshow("Right Camera HSV Smoothed and Thresholded Video", thresholdRight);
//		imshow("Left Camera HSV Smoothed and Thresholded Video", thresholdLeft);
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		key = waitKey(30);
	}
	
	destroyAllWindows();
	return 0;
}

void myMouseCallback(int event, int x, int y, int flags, void *param) {
	Mat *image = (Mat *)param;
	switch (event) {
		case EVENT_LBUTTONDOWN:
			<#statements#>
			break;
			
		default:
			break;
	}
	
}
