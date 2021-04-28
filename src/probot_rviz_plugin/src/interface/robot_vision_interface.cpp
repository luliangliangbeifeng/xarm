#include <image_transport/image_transport.h>
#include <string>

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
	string Mode = "sim";
    nh_.param<std::string>("mode", Mode, "sim");

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
    Size patternSize ;
	if(Mode != "sim") {
		patternSize = Size(8, 11);
	} else {
		patternSize = Size(7, 7);
	}
	int ret = findChessboardCorners(srcgray, patternSize, corners);

	if(ret == 0) {
        ROS_INFO("缺少标定板模型");
		return true;
	}

	//寻找亚像素角点
	Size winSize = Size(5, 5);  //搜素窗口的一半尺寸
	Size zeroZone = Size(-1, -1);//表示死区的一半尺寸
	//求角点的迭代过程的终止条件，即角点位置的确定
	TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
	cornerSubPix(srcgray, corners, winSize, zeroZone, criteria);
	if(Mode != "sim") {
       //对于8X11选择的九个点
	    for (int i = 0; i < corners.size(); i++) {
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

                pixelPositionX[index-1] = corners[i].x; 
                pixelPositionY[index-1] = corners[i].y;

                ROS_INFO("第 %d 个角点的像素坐标: (%3f, %3f)", index, corners[i].x, corners[i].y);  
            }
	    }  
    } else {
            //绘制角点
	    for (int i = 0; i < corners.size(); i++) {
		    char temp[16];
            int index = 0;
            if (i == 0) index = 1;
		    if (i == 3) index = 2;
		    if (i == 6) index = 3;
		    if (i == 21) index = 4;
		    if (i == 24) index = 5;
		    if (i == 27) index = 6;
		    if (i == 42) index = 7;
		    if (i == 45) index = 8;
		    if (i == 48) index = 9;
            if(index > 0) {
		        sprintf(temp, "%d", index);
		        putText(image, temp, corners[i], FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, LINE_AA, false);
		        circle(image, corners[i], 2, Scalar(255, 0, 0), -1, 8, 0);

                pixelPositionX[index-1] = corners[i].x; 
                pixelPositionY[index-1] = corners[i].y;

                ROS_INFO("第 %d 个角点的像素坐标: (%3f, %3f)", index, corners[i].x, corners[i].y);  
            }
	    }
    }

    image_transport::ImageTransport it_(nh_);
    image_transport::Publisher image_pub_ = it_.advertise("/object_detect", 1);
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	int pub_flag = 0;
    ros::Rate loop_rate(5);
    while(ros::ok) {
		pub_flag++;
		if(pub_flag > 10) break;
        image_pub_.publish(msg1);
        ros::spinOnce();
        loop_rate.sleep();

    }
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
                             std::vector<float>& pixelPositionY, std::vector<float>& calibrationResultX, std::vector<float>& calibrationResultY, std::string calibrationYaml)
{
  //Image Calibration  

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

    ofstream OutFile(calibrationYaml); 
    OutFile << "calibration: \n";  
    OutFile << "  calibrationResultX0: " + to_string( calibrationResultX[0]) + "\n";  
    OutFile << "  calibrationResultX1: "+ to_string( calibrationResultX[1]) + "\n";  
    OutFile << "  calibrationResultX2: "+ to_string(calibrationResultX[2]) + "\n";
    OutFile << "  calibrationResultY0: "+ to_string( calibrationResultY[0]) + "\n";  
    OutFile << "  calibrationResultY1: " + to_string( calibrationResultY[1]) + "\n";  
    OutFile << "  calibrationResultY2: "+ to_string( calibrationResultY[2]) + "\n";  
    OutFile.close();            //关闭文件 

    ROS_INFO("相机标定参数如下：");  
    ROS_INFO("calibrationResultX[0] = %f", calibrationResultX[0]);  
    ROS_INFO("calibrationResultX[1] = %f", calibrationResultX[1]);   
    ROS_INFO("calibrationResultX[2] = %f", calibrationResultX[2]);   
    ROS_INFO("calibrationResultY[0] = %f", calibrationResultY[0]);   
    ROS_INFO("calibrationResultY[1] = %f", calibrationResultY[1]);   
    ROS_INFO("calibrationResultY[2] = %f", calibrationResultY[2]);    

    return true;
}

/**
 * Input:
 *    roiX：ROI左上角点x像素坐标
 *    roiY：ROI左上角点y像素坐标
 *    roiHeight：ROI区域高度
 *    roiWidth：ROI区域宽度
 * Output：
 *    输出图像话题的变化
 **/
bool RobotVisionInterface::setRoiParameters(unsigned int roiX, unsigned int roiY, unsigned int roiHeight, unsigned int roiWidth)
{
    ROS_INFO("ROI 参数: (%d,%d,%d,%d)", roiX, roiY, roiHeight, roiWidth);
    sensor_msgs::ImageConstPtr msg;
    try
    {
        msg = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw", ros::Duration(5.0));
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("等待图像数据: %s", e.what());
        return false;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }

    //初始化存储目标检测框的vector
    std::vector<cv::Rect> box;

    // 图像裁剪
    cv::Rect select = cv::Rect(roiX, roiY, roiWidth, roiHeight);
    cv::Mat& img_input = cv_ptr->image;

    img_input = img_input(select);
    image_transport::ImageTransport it_(nh_);
    image_transport::Publisher image_pub_ = it_.advertise("/object_detect", 1);
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8",  img_input ).toImageMsg();

    int pub_flag = 0;
    ros::Rate loop_rate(5);
    while(ros::ok) {
		pub_flag++;
		if(pub_flag > 1) break;
        image_pub_.publish(msg1);
        ros::spinOnce();
        loop_rate.sleep();
    }
	ros::param::set("image/ROI_x", (int)roiX);
	ros::param::set("image/ROI_x", (int)roiY);
	ros::param::set("image/ROI_width", (int)roiWidth);
	ros::param::set("image/ROI_height", (int)roiHeight);
    roivalue[0] = roiHeight;
    roivalue[1] = roiWidth;
    roivalue[2] = roiX;
    roivalue[3] = roiY;

    return true;
}

/**
 * Input:
 *    imageMinHsv：hsv最小值，顺序是h、s、v
 *    imageMaxHsv：hsv最大值，顺序是h、s、v
 * Output：
 *    输出图像话题的变化
 **/
bool RobotVisionInterface::setHsvParameters(const std::vector<int>& imageMinHsv, const std::vector<int>& imageMaxHsv)
{
    ROS_INFO("HSV 参数: (%d,%d,%d)-(%d,%d,%d)", imageMinHsv[0], imageMinHsv[1],imageMinHsv[2],
    imageMaxHsv[0], imageMaxHsv[1], imageMaxHsv[2]);

    sensor_msgs::ImageConstPtr msg;

    try
    {
        msg = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw", ros::Duration(5.0));
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("等待图像数据: %s", e.what());
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
    Mat bgr, hsv;
    image.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
	//颜色空间转换
	cvtColor(bgr, hsv, COLOR_BGR2HSV);

    Mat dst;
    dst = Mat::zeros(image.size(), CV_32FC3);
	//掩码
	Mat mask;
	inRange(hsv, Scalar(imageMinHsv[0], imageMinHsv[1] / float(255), imageMinHsv[2]/ float(255)), Scalar(imageMaxHsv[0], imageMaxHsv[1] / float(255), imageMaxHsv[2] / float(255)), mask);
	//只保留
	for (int r = 0; r < bgr.rows; r++) {
		for (int c = 0; c < bgr.cols; c++) {
			if (mask.at<uchar>(r, c) == 255) {
				dst.at<Vec3f>(r, c) = bgr.at<Vec3f>(r, c);
			}
		}
	}
    Mat img;
    dst.convertTo(img, CV_8UC3, 255);    
    image_transport::ImageTransport it_(nh_);
    image_transport::Publisher image_pub_ = it_.advertise("/object_detect", 1);
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8",  img ).toImageMsg();

    int pub_flag = 0;
    ros::Rate loop_rate(5);
    while(ros::ok) {
		pub_flag++;
		if(pub_flag > 1) break;
        image_pub_.publish(msg1);
        ros::spinOnce();
    }

	ros::param::set("hsv/hmin", imageMinHsv[0]);
	ros::param::set("hsv/hmax", imageMaxHsv[0]);
	ros::param::set("hsv/smin", imageMinHsv[1]);
	ros::param::set("hsv/smax", imageMaxHsv[1]);
	ros::param::set("hsv/vmin", imageMinHsv[2]);
	ros::param::set("hsv/vmax", imageMaxHsv[2]);

    hsvmin[0] = imageMinHsv[0];
    hsvmax[0] = imageMaxHsv[0];
    hsvmin[1] = imageMinHsv[1];
    hsvmax[1] = imageMaxHsv[1];
    hsvmin[2] = imageMinHsv[2];
    hsvmax[2] = imageMaxHsv[2];

  return true;
}

/*
*  保存hsv和roi参数
*/
bool RobotVisionInterface::saveParameters(string yamlname)
{
    ofstream OutFile(yamlname); 
    OutFile << "image:\n";   
    OutFile << "  ROI_height: " + to_string( roivalue[0]) + "\n";  
    OutFile << "  ROI_width: " + to_string( roivalue[1]) + "\n";  
    OutFile << "  ROI_x: " + to_string( roivalue[2]) + "\n";
    OutFile << "  ROI_y: " + to_string( roivalue[3]) + "\n";  
    OutFile << "hsv:\n";   
    OutFile << "  hmin: " + to_string( hsvmin[0]) + "\n";  
    OutFile << "  hmax: " + to_string( hsvmax[0]) + "\n";  
    OutFile << "  smin: " + to_string( hsvmin[1]) + "\n";
    OutFile << "  smax: " + to_string( hsvmax[1]) + "\n";  
    OutFile << "  vmin: " + to_string( hsvmin[2]) + "\n"; 
    OutFile << "  vmax: " + to_string( hsvmax[2]) + "\n"; 
    OutFile.close();            //关闭文件 
}
/**
 * Input:
 *    图像话题和已经标定和调试的参数
 * Output：
 *    pixelPosition： 输出像素坐标位置
 *    objectPosition：输出图像识别结果
 **/
bool RobotVisionInterface::objectRecognitionProcess(std::vector<unsigned int>& pixelPosition, std::vector<float>& objectPosition)
{
    int hmax, hmin, smax, smin, vmax, vmin;
    int ROI_x, ROI_y, ROI_width, ROI_height;
    double calibrationResultX0, calibrationResultX1, calibrationResultX2, calibrationResultY0, calibrationResultY1, calibrationResultY2;
    image_transport::ImageTransport it_(nh_);

    nh_.param("hsv/hmin", hmin, 0);
    nh_.param("hsv/hmax", hmax, 40);
    nh_.param("hsv/smin", smin, 60);
    nh_.param("hsv/smax", smax, 140);
    nh_.param("hsv/vmin", vmin, 160);
    nh_.param("hsv/vmax", vmax, 230);

    nh_.param("image/ROI_x", ROI_x, 0);
    nh_.param("image/ROI_y", ROI_y, 0);
    nh_.param("image/ROI_width", ROI_width, 640);
    nh_.param("image/ROI_height", ROI_height, 480);

    nh_.param("calibration/calibrationResultX0", calibrationResultX0, 0.0);
    nh_.param("calibration/calibrationResultX1", calibrationResultX1, 0.0);
    nh_.param("calibration/calibrationResultX2", calibrationResultX2, 0.0);
    nh_.param("calibration/calibrationResultY0", calibrationResultY0, 0.0);
    nh_.param("calibration/calibrationResultY1", calibrationResultY1, 0.0);
    nh_.param("calibration/calibrationResultY2", calibrationResultY2, 0.0);

	sensor_msgs::ImageConstPtr msg;

    try
    {
        msg = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw", ros::Duration(5.0));
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("等待图像数据: %s", e.what());
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
      //初始化存储目标检测框的vector
    std::vector<cv::Rect> box;

    // 图像裁剪
    cv::Rect select = cv::Rect(ROI_x, ROI_y, ROI_width, ROI_height);
    cv::Mat& img_input = cv_ptr->image;

    img_input = img_input(select);


    //彩色图像的灰度值归一化，颜色空间转换，输出为HSV格式图像
    cv::Mat image2hsv, bgr;
    img_input.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
    cv::cvtColor(bgr, image2hsv,  cv::COLOR_BGR2HSV);

    int box2draw =0;

    //step3:设置H(色相)阈值，显示Inrange区域对应的输入图像(仅调试使用)
    cv::Mat img_Inrange;
    img_Inrange = cv::Mat::zeros(image2hsv.size(), CV_32FC3);

    cv::Mat mask;
    cv::inRange(image2hsv,
                cv::Scalar(hmin, smin / float(255), vmin / float(255)),
                cv::Scalar(hmax, smax / float(255), vmax / float(255)),
                mask);

    //step4:对mask区域进行形态学-开运算，抑制噪声点
    cv::Mat mask_Opened = cv::Mat::zeros(image2hsv.size(), CV_8U);

    //形态学滤波
    cv::Mat element_9(9, 9, CV_8U, cv::Scalar(1));
    cv::Mat element_3(3, 3, CV_8U, cv::Scalar(1));
    cv::morphologyEx(mask, mask_Opened, cv::MORPH_OPEN, element_9);
    cv::dilate(mask_Opened, mask_Opened, element_3);

    //step5：寻找mask_opened连通区域，标记最大与次大，通过形状因子确定目标区域
    //contours用于保存所有轮廓信息,hierarchy用于记录轮廓之间的继承关系
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask_Opened, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	//std::cout << "轮廓识别" << std::endl;
    int idx = 0;
    int idx_num = contours.size();  //轮廓总数量
    int idx_left = idx_num;         //筛选后剩余轮廓数量
    char contours_tag[20]={0};      //轮廓对应标签，字符串初始化
    cv::Point origin;               //字符标注原点初始化

    for(int i=0; i<idx_num; i++)
    {
        //计算轮廓对应的的矩形边界
        cv::Rect rect = cv::boundingRect(contours[i]);
        int x = rect.x;
        int y = rect.y;
        int w = rect.width;
        int h = rect.height;
        origin.x = x;
        origin.y = y+15;

        float contour_area = cv::contourArea(contours[i], false );
        float shape_ratio = contour_area*4*CV_PI/contours[i].size()/contours[i].size();

        //轮廓大小(像素数量)、长宽比约束
        if(contours[i].size() < 80 || w < 0.2 * h || w > 5 * h )
        {
            idx_left--;
            continue;
        }
        //轮廓形状因子约束
        if(shape_ratio < 0.45) //这个值需要人工调整，圆的形状因子为1,越不规整，形状因子越小
        {
            idx_left--;
            continue;

        }
        //如果这个轮廓有父轮廓，说明这个轮廓也不是我们的目标，删除之
        if(hierarchy[i][3] != -1)
        {
            idx_left--;
            continue;
        }
        else
            box.push_back(rect);

    }
    ROS_INFO("识别到的目标数量：%d", idx_left);   
    if(!box.empty())
    {
        for(; box2draw<box.size(); box2draw++)
        {
             rectangle(img_input, box.at(box2draw), cv::Scalar(0, 255, 255));

             pixelPosition[0]= box.at(box2draw).x + cvRound(box.at(box2draw).width/2)  + ROI_x;
             pixelPosition[1] = box.at(box2draw).y + cvRound(box.at(box2draw).height/2) + ROI_y;
             objectPosition[0] = calibrationResultX0 * pixelPosition[0] + calibrationResultX1 * pixelPosition[1] + calibrationResultX2; 
             objectPosition[1] = calibrationResultY0 * pixelPosition[0] + calibrationResultY1 * pixelPosition[1] + calibrationResultY2; 
        }
    } 

    image_transport::Publisher image_pub_ = it_.advertise("/object_detect", 1);
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_input ).toImageMsg();

    int pub_flag = 0;
    ros::Rate loop_rate(5);
    while(ros::ok) {
		pub_flag++;
		if(pub_flag > 1) break;
        image_pub_.publish(msg1);
        ros::spinOnce();
        loop_rate.sleep();
    }  

    ROS_INFO("图像坐标（pixelPosition）: (%d,%d,%d)， 实际目标坐标（objectPosition）: (%0.4f,%0.4f,%0.4f)", pixelPosition[0], pixelPosition[1], pixelPosition[2],
    objectPosition[0], objectPosition[1], objectPosition[2]);

    return true;
}

/**
 * Input:
 *    pickPosition：抓取的位置，顺序x，y, z
 *    placePosition：放置的位置，顺序x，y, z
 **/
bool RobotVisionInterface::pickAndPlaceProcess(std::vector<float>& pickPosition, std::vector<float>& placePosition)
{
    string Mode = "sim";
    nh_.param<std::string>("mode", Mode, "sim");
    ROS_INFO("抓取点位置（pickPosition）: (%0.4f,%0.4f,%0.4f)， 放置点位置（placePosition）: (%0.4f,%0.4f,%0.4f)", pickPosition[0], pickPosition[1], pickPosition[2],
    placePosition[0], placePosition[1], placePosition[2]);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("xarm6");
	//gazebo中配置甲爪运动规划组
    moveit::planning_interface::MoveGroupInterface grippergroup("xarm_gripper");

    //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "world";
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.001);

    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.1);
    arm.setMaxVelocityScalingFactor(0.1);
 
	//配置甲爪速度
	ros::ServiceClient config_client = nh_.serviceClient<xarm_msgs::GripperConfig>("/xarm/gripper_config");
    // 初始化请求数据
	xarm_msgs::GripperConfig srv_speed;
	srv_speed.request.pulse_vel  = 1500;
	config_client.call(srv_speed);

	//配置甲爪开合范围
	ros::ServiceClient move_client = nh_.serviceClient<xarm_msgs::GripperMove>("/xarm/gripper_move");
    //打开甲爪
	xarm_msgs::GripperMove srv_move;
	srv_move.request.pulse_pos = 850;
	move_client.call(srv_move);

    // 设置机器人终端的目标抓取位置
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = pickPosition[0];
    target_pose1.position.y = pickPosition[1];
    target_pose1.position.z = pickPosition[2];
    target_pose1.orientation.x = 1.0;

    if(Mode != "sim") {
        //发送服务打开甲爪
        srv_move.request.pulse_pos = 850;
	    move_client.call(srv_move);
	    sleep(1);
    } else {
        // gazebo中打开甲爪
        ros::WallDuration(1.0).sleep();
		grippergroup.setNamedTarget("open");
    	grippergroup.move();
    	sleep(1);       
    }

    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();

    arm.setPoseTarget(target_pose1);

    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    moveit::planning_interface::MoveItErrorCode success1 = arm.plan(plan1);

    ROS_INFO("%s",success1?"目标抓取路径规划成功":"目标抓取路径规划失败");   

    //让机械臂按照规划的轨迹开始运动。
    if(success1) {
    	arm.execute(plan1);
    	sleep(1);
	} else {
		return true;
	}

    if(Mode != "sim") {
        //发送服务合上甲爪
        srv_move.request.pulse_pos = 275;
	    move_client.call(srv_move);
	    sleep(1);
    } else {
        // gazebo中合上甲爪
        ros::WallDuration(1.0).sleep();
        grippergroup.setNamedTarget("close");
        grippergroup.move();     
    }

    // 设置机器人终端的目标摆放位置
    geometry_msgs::Pose target_pose2;
    target_pose2.position.x = placePosition[0];
    target_pose2.position.y = placePosition[1];
    target_pose2.position.z = placePosition[2];
    target_pose2.orientation.x = 1.0;

    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();

    arm.setPoseTarget(target_pose2);

    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");  
    ROS_INFO("%s",success?"目标放置路径规划成功":"目标放置路径规划失败");   

    //让机械臂按照规划的轨迹开始运动。
    if(success) {
    	arm.execute(plan);
    	sleep(1);
	} else {
		return true;
	}

    if(Mode != "sim") {
        //发送服务打开甲爪
        srv_move.request.pulse_pos = 850;
	    move_client.call(srv_move);
	    sleep(1);
    } else {
        // gazebo中打开甲爪
        ros::WallDuration(1.0).sleep();
        grippergroup.setNamedTarget("open");
        grippergroup.move();     
    }

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("hold-up");
    arm.move();
    sleep(1);

  return true;
}

/**
 * Input:
 *    posX：X坐标
 *    posY：Y坐标
 *    posZ：Z坐标
 **/
bool RobotVisionInterface::goToCaptureImagePosition(float posX, float posY, float posZ)
{
    ROS_INFO("移动到拍照点位置: (%0.4f,%0.4f,%0.4f)", posX, posY, posZ);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("xarm6");

    //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "world";
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.001);

    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.1);
    arm.setMaxVelocityScalingFactor(0.1);

    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;
    target_pose.position.x = posX;
    target_pose.position.y = posY;
    target_pose.position.z = posZ;
    target_pose.orientation.x = 1.0;

    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();

    arm.setPoseTarget(target_pose);

    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

    ROS_INFO("%s",success?"机械臂移动到拍摄点成功":"机械臂移动到拍摄点失败");   
    //让机械臂按照规划的轨迹开始运动。
    if(success) {
    	arm.execute(plan);
    	sleep(1);
	} else {
		return true;
	}

  return true;  
}

}
