#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.hpp"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
#define FIT_min_x 420
#define FIT_min_y 70
#define FIT_max_x 2450
#define FIT_max_y 2000
#endif

using namespace std;
using namespace cv;

vector<double> D{-0.064138, 0.096647, -0.002297, 0.000280, 0.000000};
boost::array<double, 9> K = {
    1246.309179,
    0.000000,
    622.211138,
    0.000000,
    1245.932236,
    525.492900,
    0.000000,
    0.000000,
    1.000000};

boost::array<double, 12>
    P = {1237.149780, 0.000000, 622.757711, 0.000000, 0.000000, 1236.707764, 523.835781, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000};

boost::array<double, 9>
    r = {1.000000, 0.000000, 0.000000,
         0.000000, 1.000000, 0.000000,
         0.000000, 0.000000, 1.000000};

sensor_msgs::CameraInfo camera_info_msg_last_1;
sensor_msgs::CameraInfo camera_info_msg_last_2;

sensor_msgs::Image image_msg_last_1;
sensor_msgs::Image image_msg_last_2;
sensor_msgs::Image image_msg_last_3;
ros::Time rcv_time_last;

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;
    // string src = "",image_pub = "";
    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    camera::Camera MVS_cap(hikrobot_camera);
    //********** rosnode init **********/
    image_transport::ImageTransport main_cam_image(hikrobot_camera);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("left_camera/image", 1000);

    sensor_msgs::Image image_msg;

    sensor_msgs::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; // 就是rgb格式

    std::string path_for_time_stamp = "/home/wilson/timeshare";
    const char *shared_file_name = path_for_time_stamp.c_str();
    int fd = open(shared_file_name, O_RDWR);

    pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, 0);

    // camera calib info
    camera_info_msg.height = 1024;
    camera_info_msg.width = 1280;
    camera_info_msg.distortion_model = "plumb_bob";
    camera_info_msg.D = D;
    camera_info_msg.K = K;
    camera_info_msg.P = P;
    camera_info_msg.R = r;
    camera_info_msg.binning_x = 0;
    camera_info_msg.binning_y = 0;

    //********** 10 Hz        **********/
    ros::Rate loop_rate(10);

    while (ros::ok())
    {

        loop_rate.sleep();
        ros::spinOnce();

        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }
#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x, FIT_min_y, FIT_max_x - FIT_min_x, FIT_max_y - FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else
        cv_ptr->image = src;
#endif
        int64_t low = pointt->low;
        double time_pc = low / 1000000000.0;
        ros::Time rcv_time = ros::Time(time_pc);

        // camre message

        image_msg = *(cv_ptr->toImageMsg());
        // image_msg.header.stamp = ros::Time::now(); // ros发出的时间不是快门时间
        image_msg.header.frame_id = "hikrobot_camera";

        // camre info message
        camera_info_msg.header.frame_id = image_msg.header.frame_id;
        camera_info_msg.header.stamp = image_msg.header.stamp;

        std::string debug;
        debug = "  rcv_time:  " + std::to_string(rcv_time.toSec());
        ROS_ERROR(debug.c_str());


        image_msg.header.stamp = rcv_time;
        camera_info_msg.header.stamp = image_msg.header.stamp;
        image_pub.publish(image_msg, camera_info_msg);



        std::string debug_info;
        debug_info = " Time -> image_msg_current:  " + std::to_string(image_msg.header.stamp.toSec());
        ROS_ERROR(debug_info.c_str());

        // std::string debug_time;
        // if (rcv_time_last.toSec() == rcv_time.toSec())
        // {
        //     image_msg_last_3 = image_msg_last_2;
        //     image_msg_last_2 = image_msg_last_1;
        //     image_msg_last_1 = image_msg;

        //     std::string debug_info_1;
        //     debug_info_1 = " Time -> image_msg_last_1:  " + std::to_string(image_msg_last_1.header.stamp.toSec());
        //     ROS_ERROR(debug_info_1.c_str());
        //     debug_info_1 = " Time -> image_msg_last_2:  " + std::to_string(image_msg_last_2.header.stamp.toSec());
        //     ROS_ERROR(debug_info_1.c_str());
        //     debug_info_1 = " Time -> image_msg_last_3:  " + std::to_string(image_msg_last_3.header.stamp.toSec());
        //     ROS_ERROR(debug_info_1.c_str());

        //     debug_time = "  cv_time_last.toSec()==rcv_time.toSec()<<<<<<<<< ";
        //     ROS_ERROR(debug_time.c_str());
        // }
        // else
        // {

        //     if (rcv_time_last.toSec() == 0)
        //     {
        //         debug_time = "  >>>>>>>>>>>>cv_time_last.toSec()!=rcv_time.toSec()*******First";
        //         ROS_ERROR(debug_time.c_str());
        //         rcv_time_last = rcv_time;
        //     }
        //     else
        //     {
        //         image_msg_last_3 = image_msg_last_2;
        //         image_msg_last_2 = image_msg_last_1;
        //         image_msg_last_1 = image_msg;
        //         image_msg_last_1.header.stamp = rcv_time;
        //         camera_info_msg.header.stamp = image_msg_last_1.header.stamp;

        //         std::string debug_info_1;
        //         debug_info_1 = " Time -> image_msg_last_1:  " + std::to_string(image_msg_last_1.header.stamp.toSec());
        //         ROS_ERROR(debug_info_1.c_str());
        //         debug_info_1 = " Time -> image_msg_last_2:  " + std::to_string(image_msg_last_2.header.stamp.toSec());
        //         ROS_ERROR(debug_info_1.c_str());
        //         debug_info_1 = " Time -> image_msg_last_3:  " + std::to_string(image_msg_last_3.header.stamp.toSec());
        //         ROS_ERROR(debug_info_1.c_str());

        //         debug_time = "  >>>>>>>>>>>>cv_time_last.toSec()!=rcv_time.toSec()";
        //         ROS_ERROR(debug_time.c_str());
        //         rcv_time_last = rcv_time;

        //         image_pub.publish(image_msg_last_1, camera_info_msg);

        //         // image_msg.header.stamp = rcv_time;
        //         // camera_info_msg.header.stamp = image_msg.header.stamp;
        //         // image_pub.publish(image_msg, camera_info_msg);
        //     }
        // }

        //*******************************************************************************************************************/
    }

    munmap(pointt, sizeof(time_stamp) * 5);

    return 0;
}
