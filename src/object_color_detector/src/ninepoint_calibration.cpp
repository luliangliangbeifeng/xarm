#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "yaml-cpp/yaml.h"
#include <stdio.h>
#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int flag = 1;
image_transport::Publisher image_pub;
vector<Point2f>points_camera;
vector<Point2f> corners;
sensor_msgs::ImagePtr msg1;

void imagecallback(const sensor_msgs::ImageConstPtr &msg)
{
	flag = 0;


	cv_bridge::CvImagePtr cv_ptr;
	try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

	Mat image = cv_ptr->image;

	cout << flag << endl;
    
    // Output modified video stream

 
   
    Mat srcgray, dstImage, normImage, scaledImage;

	cvtColor(image, srcgray, CV_BGR2GRAY);

	Mat srcbinary;
	threshold(srcgray, srcbinary, 0, 255, THRESH_OTSU | THRESH_BINARY);

	Mat kernel = getStructuringElement(MORPH_RECT, Size(15, 15), Point(-1, -1));
	morphologyEx(srcbinary, srcbinary, MORPH_OPEN, kernel, Point(-1, -1));

	Size patternSize = Size(8, 11);
	findChessboardCorners(srcgray, patternSize,  corners);

	//寻找亚像素角点
	Size winSize = Size(5, 5);  //搜素窗口的一半尺寸
	Size zeroZone = Size(-1, -1);//表示死区的一半尺寸
	//求角点的迭代过程的终止条件，即角点位置的确定
	TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);

	cornerSubPix(srcgray, corners, winSize, zeroZone, criteria);

	//绘制角点
	for (int i = 0; i < corners.size(); i++)
	{
		char temp[16];
		sprintf(temp, "%d", i+1);
		putText(image, temp, corners[i], FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, LINE_AA, false);
		circle(image, corners[i], 2, Scalar(255, 0, 0), -1, 8, 0);
	}
//    	msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
//	image_pub.publish(msg1);
    image_pub.publish(cv_ptr->toImageMsg());

	cout << "第1个角点的像素坐标，：" << corners[0] << endl;
	cout << "第4个角点的像素坐标，：" << corners[3] << endl;
	cout << "第8个角点的像素坐标，：" << corners[7] << endl;
	cout << "第49个角点的像素坐标，：" << corners[48] << endl;
	cout << "第52个角点的像素坐标，：" << corners[51] << endl;
	cout << "第56个角点的像素坐标，：" << corners[55] << endl;
	cout << "第81个角点的像素坐标，：" << corners[80] << endl;
	cout << "第84个角点的像素坐标，：" << corners[83] << endl;
	cout << "第88个角点的像素坐标，：" << corners[87] << endl;
	
	// Update GUI Window
    cv::imshow("Image window", image);
    cv::waitKey(2000);
	// 九点标定
	points_camera.push_back(corners[0]);
	points_camera.push_back(corners[3]);
	points_camera.push_back(corners[7]);
	points_camera.push_back(corners[48]);
	points_camera.push_back(corners[51]);
	points_camera.push_back(corners[55]);
	points_camera.push_back(corners[80]);
	points_camera.push_back(corners[83]);
	points_camera.push_back(corners[87]);
    double A, B, C, D, E, F;
	Mat warpMat;
	vector<Point2f>points_robot;

	for (int i = 0; i < 9; i++) {
		int index;
		if (i == 0) index = 1;
		if (i == 1) index = 4;
		if (i == 2) index = 8;
		if (i == 3) index = 49;
		if (i == 4) index = 52;
		if (i == 5) index = 56;
		if (i == 6) index = 81;
		if (i == 7) index = 84;
		if (i == 8) index = 88;
		cout << "标定针在第" << index << "个角点正上方的机械臂坐标：";
		double x, y;
		cin >> x >> y;
		points_robot.push_back(Point2f(x, y));

	}

	warpMat = estimateRigidTransform(points_camera, points_robot, true);
	cout << " " << endl;
    cout <<"3、机械臂坐标与像素坐标的变换关系：" << warpMat << endl;

	A = warpMat.ptr<double>(0)[0];
	B = warpMat.ptr<double>(0)[1];
	C = warpMat.ptr<double>(0)[2];
	D = warpMat.ptr<double>(1)[0];
	E = warpMat.ptr<double>(1)[1];
	F = warpMat.ptr<double>(1)[2];
	cout << "A = " << A << endl;
	cout << "B = " << B << endl;
	cout << "C = " << C << endl;
	cout << "D = " << D << endl;
	cout << "E = " << E << endl;
	cout << "F = " << F << endl;
	YAML:: Node config;
	config["A"] = A;
	config["B"] = B;
	config["C"] = C;
	config["D"] = D;
	config["E"] = E;
	config["F"] = F;
	std::ofstream fout("/home/tianbot/xarm/src/object_color_detector/config/123.yaml");
	fout<<config;
	fout.close();
	for(int i = 0; i < 80; i++) {
		cout << corners[i] << endl;
		cout << "x：" << A * corners[i].x + B * corners[i].y + C << "   y: " << D * corners[i].x + E * corners[i].y + F << endl;
	}
/*
	float A =12, B = 13, C = 14, D = 15, E = 16, F = 17;
	YAML:: Node config;
	config["A"] = A+1;
	config["B"] = B;
	config["C"] = C;
	config["D"] = D;
	config["E"] = E;
	config["F"] = F+1;
	std::ofstream fout("/home/tianbot/xarm/src/object_color_detector/config/123.yaml");
	fout<<config;
	fout.close();
    */

}

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "ninepoint_calibration");
  	ros::NodeHandle nh;
  	image_transport::ImageTransport it(nh);

	image_pub = it.advertise("/calibration_result", 1);
	image_transport::Subscriber image_sub = it.subscribe("camera/color/image_raw", 1, imagecallback);
    while(ros::ok()) {
		ros::spinOnce();
		if(flag == 0) break;
	}


	return 0;

}
