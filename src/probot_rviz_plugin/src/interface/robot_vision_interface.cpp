#include <image_transport/image_transport.h>

#include "probot_rviz_plugin/interface/robot_vision_interface.h"

using namespace std;
using namespace cv;

namespace probot_rviz_plugin
{

RobotVisionInterface::RobotVisionInterface(ros::NodeHandle& nh) : 
    nh_(nh)
{
}

RobotVisionInterface::~RobotVisionInterface()
{
}

/**
 * Input:
 *    imageTopicName：图像话题
 * Output：
 *    pixelPositionX：所有像素X坐标
 *    pixelPositionY：所有像素Y坐标
 **/
bool RobotVisionInterface::patternRecognitionProcess(std::string& imageTopicName, std::vector<float>& pixelPositionX, std::vector<float>& pixelPositionY)
{
    sensor_msgs::ImageConstPtr msg;

    try
    {
        msg = ros::topic::waitForMessage<sensor_msgs::Image>(imageTopicName, ros::Duration(5.0));
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("Timeout waiting for image data: %s", e.what());
        return false;
    }
    cv_bridge::CvImagePtr cv_ptr;
	try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
  //Image Process
    Mat image = cv_ptr->image;

    Mat srcgray, dstImage, normImage, scaledImage;

	cvtColor(image, srcgray, CV_BGR2GRAY);

	Mat srcbinary;
    threshold(srcgray, srcbinary, 0, 255, THRESH_OTSU | THRESH_BINARY);

	Mat kernel = getStructuringElement(MORPH_RECT, Size(15, 15), Point(-1, -1));
	morphologyEx(srcbinary, srcbinary, MORPH_OPEN, kernel, Point(-1, -1));

	vector<Point2f> corners;
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
        int index = 0;
        if (i == 0) index = 1;
		if (i == 3) index = 2;
		if (i == 7) index = 3;
		if (i == 40) index = 4;
		if (i == 43) index = 5;
		if (i == 47) index = 6;
		if (i == 80) index = 7;
		if (i == 83) index = 8;
		if (i == 87) index = 9;
        if(index > 0) {
		    sprintf(temp, "%d", index);
		    putText(image, temp, corners[i], FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, LINE_AA, false);
		    circle(image, corners[i], 2, Scalar(255, 0, 0), -1, 8, 0);
        }
	}

	std::cout << "第1个角点的像素坐标，：" << corners[0] << std::endl;
	std::cout << "第2个角点的像素坐标，：" << corners[3] << std::endl;
	std::cout << "第3个角点的像素坐标，：" << corners[7] << std::endl;
	std::cout << "第4个角点的像素坐标，：" << corners[40] << std::endl;
	std::cout << "第5个角点的像素坐标，：" << corners[43] << std::endl;
	std::cout << "第6个角点的像素坐标，：" << corners[47] << std::endl;
	std::cout << "第7个角点的像素坐标，：" << corners[80] << std::endl;
	std::cout << "第8个角点的像素坐标，：" << corners[83] << std::endl;
	std::cout << "第9个角点的像素坐标，：" << corners[87] << std::endl;
    pixelPositionX[0] = corners[0].x;
    pixelPositionX[1] = corners[3].x;
    pixelPositionX[2] = corners[7].x;
    pixelPositionX[3] = corners[48].x;
    pixelPositionX[4] = corners[51].x;
    pixelPositionX[5] = corners[55].x;
    pixelPositionX[6] = corners[80].x;
    pixelPositionX[7] = corners[83].x;
    pixelPositionX[8] = corners[87].x;
    pixelPositionY[0] = corners[0].y;
    pixelPositionY[1] = corners[3].y;
    pixelPositionY[2] = corners[7].y;
    pixelPositionY[3] = corners[48].y;
    pixelPositionY[4] = corners[51].y;
    pixelPositionY[5] = corners[55].y;
    pixelPositionY[6] = corners[80].y;
    pixelPositionY[7] = corners[83].y;
    pixelPositionY[8] = corners[87].y;

    imshow("Image window", image);
    waitKey(2000);
    return true;
}

/**
 * Input:
 *    robotPositionX：机器人X坐标
 *    robotPositionY：机器人Y坐标
 *    pixelPositionX：像素X坐标
 *    pixelPositionY：像素Y坐标
 * Output：
 *    calibrationResultX：标定X坐标系数
 *    calibrationResultY：标定Y坐标系数
 **/
bool RobotVisionInterface::imageCalibrationProcess(std::vector<float>& robotPositionX, std::vector<float>& robotPositionY, std::vector<float>& pixelPositionX, 
                             std::vector<float>& pixelPositionY, std::vector<float>& calibrationResultX, std::vector<float>& calibrationResultY)
{
  //Image Calibration  
    for(int i = 0; i < 9; i++) {
        std::cout <<  robotPositionX[i] << "   " <<  robotPositionY[i] << std::endl;
    }
    	// 九点标定
	Mat warpMat;

	vector<Point2f>points_robot;
    vector<Point2f>points_camera;
    for(int i = 0; i < 9; i++) {
        points_camera.push_back(Point2f(pixelPositionX[i], pixelPositionY[i]));
        points_robot.push_back(Point2f(robotPositionX[i], robotPositionY[i]));
    }

   //计算变换函数
	warpMat =  estimateRigidTransform(points_camera, points_robot, true);
    calibrationResultX[0] =  warpMat.ptr<double>(0)[0];
    calibrationResultX[1] =  warpMat.ptr<double>(0)[1];
    calibrationResultX[2] =  warpMat.ptr<double>(0)[2];  
    calibrationResultY[0] =  warpMat.ptr<double>(1)[0];
    calibrationResultY[1] =  warpMat.ptr<double>(1)[1];
    calibrationResultY[2] =  warpMat.ptr<double>(1)[2];  

	std::cout << "calibrationResultX[0] = " << calibrationResultX[0]<< std::endl;
	std::cout << "calibrationResultX[1] = " << calibrationResultX[1] << std::endl;
	std::cout << "calibrationResultX[2] = " << calibrationResultX[2] << std::endl;
	std::cout << "calibrationResultY[0] = " << calibrationResultY[0] << std::endl;
	std::cout << "calibrationResultY[1] = " << calibrationResultY[1] << std::endl;
	std::cout << "calibrationResultY[2] = " << calibrationResultY[2] << std::endl;
/*
    YAML:: Node configs;
	configs["calibrationResultX[0]"] = calibrationResultX[0];
	configs["calibrationResultX[1]"] = calibrationResultX[1];
	configs["calibrationResultX[2]"] = calibrationResultX[2];
	configs["calibrationResultY[0]"] = calibrationResultY[0];
	configs["calibrationResultY[1]"] = calibrationResultY[1];
	configs["calibrationResultY[2]"] = calibrationResultY[2];
    std::cout << "123" << std::endl;
	std::ofstream fout("/home/tianbot/xarm/src/object_color_detector/config/123.yaml");
	fout<<configs;
	fout.close();
    */
    return true;
}

}
