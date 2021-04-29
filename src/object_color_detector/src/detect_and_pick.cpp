#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int flag_rgb = 0;
int flag_depth = 0;
int flag_tf = 0;
int ROI_x = 0;
int ROI_y = 0;
int ROI_width= 640;
int ROI_height= 480;
int hmax,hmin,smax,smin,vmax,vmin;
vector<float> pixelPosition(2);
Mat depth_pic;
image_transport::Publisher image_pub;

float camera_x, camera_y, camera_z;

void imagecallback(const sensor_msgs::ImageConstPtr &msg)
{
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
    hmax = 360;
    hmin = 63;
    smax = 255;
    smin = 70;
    vmax = 250;
    vmin = 60;
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

    for(int i=0; i<idx_num; i++) {
        //计算轮廓对应的的矩形边界
        cv::Rect rect = cv::boundingRect(contours[i]);
        int x = rect.x;
        int y = rect.y;
        int w = rect.width;
        int h = rect.height;

        float contour_area = cv::contourArea(contours[i], false );
        float shape_ratio = contour_area*4*CV_PI/contours[i].size()/contours[i].size();

        //轮廓大小(像素数量)、长宽比约束
        if(contours[i].size() < 80 || w < 0.2 * h || w > 5 * h ) {
            idx_left--;
            continue;
        }
        //轮廓形状因子约束,这个值需要人工调整，圆的形状因子为1,越不规整，形状因子越小
        if(shape_ratio < 0.45) {
            idx_left--;
            continue;
        }
        //如果这个轮廓有父轮廓，说明这个轮廓也不是我们的目标，删除之
        if(hierarchy[i][3] != -1) {
            idx_left--;
            continue;
        } else
            box.push_back(rect);
    }
    ROS_INFO("number：%d", idx_left);   
    if(!box.empty()) {
        for(; box2draw<box.size(); box2draw++) {
             rectangle(img_input, box.at(box2draw), cv::Scalar(0, 255, 255));

             pixelPosition[0]= box.at(box2draw).x + cvRound(box.at(box2draw).width/2)  + ROI_x;
             pixelPosition[1] = box.at(box2draw).y + cvRound(box.at(box2draw).height/2) + ROI_y;
             flag_rgb = 1;
        }
    }
    cout <<"flag_rgb ="<< flag_rgb << endl;
    std::cout<<"the total contours:"<<idx_num<<std::endl;
    std::cout<<"the left contours:"<<idx_left<<std::endl;
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_input ).toImageMsg();
    image_pub.publish(msg1);

}

void depthcallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    cv_bridge::CvImagePtr depth_ptr;
    try
    {
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1); 
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }

    depth_pic = depth_ptr->image;
//记得修改内参
//公司相机内参
	Mat Matrix1 =(Mat_<float>(3,3)<<611.0368041992188,0.0,334.54559326171875,0.0,610.8939208984375,233.1903839111328,0.0,0.0,1.0);

//我的相机
//    Mat Matrix1 =(Mat_<float>(3,3)<<608.9002685546875,0.0,323.1551818847656,0.0,608.9404296875,242.968017578125,0.0,0.0,1.0);
	Mat Matrix2 = Matrix1.inv();
    cout << Matrix2.ptr<float>(2)[0] << "  "<< Matrix2.ptr<float>(2)[1]<< endl;

    if(flag_rgb == 1) {
        float camera_depth = depth_pic.at<float>(pixelPosition[0],pixelPosition[1]) / 1000 - 0.03;
        cout << camera_depth << "shendu" << endl;
        camera_x = ((Matrix2.ptr<float>(0)[0]) * pixelPosition[0] + (Matrix2.ptr<float>(0)[1]) * pixelPosition[0] + (Matrix2.ptr<float>(0)[2]) * 1) * camera_depth;
        camera_y = ((Matrix2.ptr<float>(1)[0]) * pixelPosition[0] + (Matrix2.ptr<float>(1)[1]) * pixelPosition[1] + (Matrix2.ptr<float>(1)[2]) * 1) * camera_depth;
        camera_z = ((Matrix2.ptr<float>(2)[0]) * pixelPosition[0] + (Matrix2.ptr<float>(2)[1]) * pixelPosition[1] + (Matrix2.ptr<float>(2)[2]) * 1) * camera_depth;
        flag_depth = 1;
        cout << "pixelPosition[0]:" << pixelPosition[0] << endl;
        cout << "pixelPosition[1]:" << pixelPosition[1] << endl;        
        cout << "camera_x:" << camera_x << endl;
        cout << "camera_y:" << camera_y << endl;
        cout << "camera_z:" << camera_z << endl;
        int flag;
        for(int i = pixelPosition[0]- 00; i < pixelPosition[0]+ 100; i++) {
            for(int j = pixelPosition[1]- 100; j < pixelPosition[1]+ 100; j++) {
                if( depth_pic.at<float>(i,j) /1000- camera_depth  - 0.03 < -0.02&& depth_pic.at<float>(i,j) /1000- camera_depth  - 0.03 > -0.05)  {
                    cout <<"i: " <<i <<"j:  "<< j << "depth="<<depth_pic.at<float>(i,j) << endl;
                }
                flag++;
            }                   
        }
        cout << flag << endl;

        cout << "dian       :" << depth_pic.at<float>(pixelPosition[0] + 120,pixelPosition[1] + 120) / 1000 << endl;  
        cout << "dian       :" << depth_pic.at<float>(pixelPosition[0]  - 120,pixelPosition[1] - 120) / 1000 << endl;  
        cout << "dian       :" << depth_pic.at<float>(pixelPosition[0] + 80,pixelPosition[1] + 80) / 1000 << endl;    

    }
    cout <<"flag_depth ="<< flag_depth << endl;

}

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
    geometry_msgs::PointStamped link6_point;
    geometry_msgs::PointStamped base_point;
    link6_point.header.frame_id = "link6";
    
    //相机与link6转换关系
 //   float qw = 0.699078675233, qx = 0.00665694384891, qy = 0.00759983760925, qz = 0.714973379506;
 //   float movex =0.0364355554487, movey = -0.0183130358739, movez = -0.0515457652258;

 //0416参数
    float qw = 0.678207953483, qx = 0.0019622625681, qy = 0.00974521972942, qz = 0.734802798069;
    float movex =0.0576031636678, movey = -0.0439886148049, movez = 0.0100813353949;
 
    float R[4][4];
    float scale = 1 / sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    qx *= scale, qy *= scale, qz *= scale, qw *= scale;
    R[0][0] = 1 - 2 * qy * qy - 2 * qz * qz;
    R[0][1] = 2 * qx * qy - 2 * qz * qw;
    R[0][2] = 2 * qx * qz + 2 * qy * qw;
    R[0][3] = movex;
    R[1][0] = 2 * qx * qy + 2 * qz * qw;
    R[1][1] = 1 - 2 * qx * qx - 2 * qz * qz;
    R[1][2] = 2.0f * qy * qz - 2.0f * qx * qw;
    R[1][3] = movey;
    R[2][0] = 2.0f * qx * qz - 2.0f * qy * qw;
    R[2][1] = 2.0f * qy * qz + 2.0f * qx * qw;
    R[2][2] = 1.0f - 2.0f * qx * qx - 2.0f * qy * qy;
    R[2][3] = movez;
    R[3][0] = R[3][1] = R[3][2] = 0;
    R[3][3] = 1;

    float g1 = 2 * (qx * qz - qw * qy);
    float g2 = 2 * (qy * qz + qw *qx);
    float g3 = 1 - 2 *qx * qx - 2 * qy * qy;
    float g4 = 2 * (qx *qy + qw * qz);
    float g5 = 1 - 2* qy * qy - 2 * qz * qz;
    float a = -asin(g1) * 180 / CV_PI;
    float b = -atan(g2/g3)* 180 / CV_PI;
    float c = -atan(g4/g5)* 180 / CV_PI;
    cout << "a" << a << endl;
    cout << "b" << b << endl;
    cout << "c" << c  << endl;

//    Mat Matrix3 =(Mat_<float>(4,4)<<R[0][0],R[0][1],R[0][2],R[0][3],R[1][0],R[1][1],R[1][2],R[1][3],R[2][0],R[2][1],R[2][2],R[2][3],R[3][0],R[3][1],R[3][2],R[3][3]);
//    Mat Matrix4 = Matrix3.inv();
//    cout << Matrix4 << endl;
    //we'll just use the most recent transform available for our simple example
    link6_point.header.stamp = ros::Time();

  //just an arbitrary point in space
    if(flag_rgb == 1 && flag_depth == 1) {
         link6_point.point.x = R[0][0] * camera_x + R[0][1] * camera_y + R[0][2] * camera_z + R[0][3];
         link6_point.point.y = R[1][0] * camera_x + R[1][1] * camera_y + R[1][2] * camera_z + R[1][3];
         link6_point.point.z = R[2][0] * camera_x + R[2][1] * camera_y + R[2][2] * camera_z + R[2][3];
         cout << link6_point .point.x<< endl;
    //    link6_point.point.x = (Matrix4.ptr<float>(0)[0]) * camera_x + (Matrix4.ptr<float>(0)[1]) * camera_y + (Matrix4.ptr<float>(0)[2]) * camera_z + (Matrix4.ptr<float>(0)[3]);
   //    link6_point.point.y = (Matrix4.ptr<float>(1)[0]) * camera_x + (Matrix4.ptr<float>(1)[1]) * camera_y + (Matrix4.ptr<float>(1)[2]) * camera_z + (Matrix4.ptr<float>(1)[3]);
     //   link6_point.point.z = (Matrix4.ptr<float>(2)[0]) * camera_x + (Matrix4.ptr<float>(2)[1]) * camera_y + (Matrix4.ptr<float>(2)[2]) * camera_z + (Matrix4.ptr<float>(2)[3]);
    } else {
        cout << "未识别到深度和RGB信息" << endl;
        link6_point.point.x = R[0][3];
        link6_point.point.y = R[1][3];
        link6_point.point.z = R[2][3];    
    }

    try{
        if(flag_rgb == 1 && flag_depth == 1) {
            listener.transformPoint("link_base", link6_point, base_point);

            ROS_INFO("link6: (%.5f, %.5f. %.5f) -----> link_base: (%.5f, %.5f, %.5f) at time %.2f",
                                    link6_point.point.x, link6_point.point.y, link6_point.point.z,
                                    base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
        }
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("%s", ex.what());
    }
    flag_tf = flag_depth && flag_rgb;      
    cout <<"flag_tf ="<< flag_tf << endl;
}


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "detect_and_pick");
  	ros::NodeHandle nh;
  	image_transport::ImageTransport it(nh);

	image_transport::Subscriber image_sub = it.subscribe("camera/color/image_raw", 1, imagecallback);
    image_transport::Subscriber depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthcallback);
    tf::TransformListener listener(ros::Duration(10));//等待10s，如果10s之后都还没收到消息，那么之前的消息就被丢弃掉
 
     //we'll transform a point once every second
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
    image_pub = it.advertise("/3D_result", 1);

    ros::Rate loop_rate(5);

    while(ros::ok()) {
        if(flag_tf == 1) {
            break;             
        }
		ros::spinOnce();
        loop_rate.sleep();
	}

	return 0;

}

// void DepthCallback(const sensor_msgs::ImageConstPtr& depth_img) {
//   ROS_WARN("%s", depth_img->encoding.c_str());
//   cv::Mat data;

//   data =
//       cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_32FC1)
//           ->image;
//   std::cout << "image data: " << data.at<uint16_t>(240, 320) << std::endl;//表示获取图像坐标为240,320的深度值,单位是毫米
// }

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "detect_and_pick");
//   ros::NodeHandle node;
//   ros::Subscriber depth_sub;
//   depth_sub = node.subscribe<sensor_msgs::Image>(
//       "/camera/aligned_depth_to_color/image_raw", 10, DepthCallback);
//   ros::spin();
//   return 0;
// }
