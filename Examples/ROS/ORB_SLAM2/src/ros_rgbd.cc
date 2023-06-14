/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

ros::Publisher g_pubPose;
ros::Subscriber g_resetSub;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    g_pubPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/cube/data/vslam_localization/pose", 1);
    g_resetSub = nh.subscribe(
        "/cube/localization/vslam/command", 1, &ImageGrabber::onResetCommand, &igb);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat trackingResult = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    if (trackingResult.empty())
    {
        ROS_INFO("Tracking lost");
        return;
    }


    cv::Mat cvRotCamToInit = trackingResult.rowRange(0,3).colRange(0,3).t();
    cv::Mat cvTransCamToInit = -cvRotCamToInit * trackingResult.rowRange(0,3).col(3);

    tf::Matrix3x3 orbToRosCoord(
            0, 0, 1,
            1, 0, 0,
            0, 1, 0);
    tf::Matrix3x3 rot(
            cvRotCamToInit.at<float>(0,0), cvRotCamToInit.at<float>(0,1), cvRotCamToInit.at<float>(0,2),
            cvRotCamToInit.at<float>(1,0), cvRotCamToInit.at<float>(1,1), cvRotCamToInit.at<float>(1,2),
            cvRotCamToInit.at<float>(2,0), cvRotCamToInit.at<float>(2,1), cvRotCamToInit.at<float>(2,2));
    tf::Quaternion q;
    (orbToRosCoord * rot).getRotation(q);

    tf::Quaternion qBase;
    qBase.setRPY(0, -M_PI_2, 0);
    tf::Quaternion rosQ = q * qBase;

    tf::Vector3 trans = tf::Vector3(cvTransCamToInit.at<float>(0,0), cvTransCamToInit.at<float>(1,0), cvTransCamToInit.at<float>(2,0));
    tf::Vector3 rosTransCam = orbToRosCoord * trans;

    tf::Transform transform;
    transform.setRotation(rosQ);
    transform.setOrigin(rosTransCam);

    // Convert camera pose to vehicle pose, assuming camera and vehicle face the same direction.
    tf::Vector3 transCamToBase = tf::Vector3(-0.08, 0, 0);
    tf::Vector3 rosTransBase = transform * transCamToBase;

    geometry_msgs::PoseWithCovarianceStamped poseCovStamped;
    poseCovStamped.header.frame_id = "slam_base";
//    poseCovStamped.header.seq = g_seq;
    poseCovStamped.header.stamp = ros::Time::now();
    poseCovStamped.pose.pose.position.x = rosTransBase.x();
    poseCovStamped.pose.pose.position.y = rosTransBase.y();
    poseCovStamped.pose.pose.position.z = rosTransBase.z();
    tf::quaternionTFToMsg(rosQ, poseCovStamped.pose.pose.orientation);
    // clang-format off
    poseCovStamped.pose.covariance = {
        1e-4, 0,    0,    0,    0,    0,
        0,    1e-4, 0,    0,    0,    0,
        0,    0,    1e-4, 0,    0,    0,
        0,    0,    0,    1e-6, 0,    0,
        0,    0,    0,    0,    1e-6, 0,
        0,    0,    0,    0,    0,    1e-6
    };
    // clang-format on

    g_pubPose.publish(poseCovStamped);
}

void ImageGrabber::onResetCommand(const std_msgs::String::ConstPtr& data)
{
    ROS_INFO("Received control command: %s", data->data.c_str());
    if (data->data == "reset")
    {
        mpSLAM->Reset();
    }
    else
    {
        ROS_WARN("Unknown command: %s", data->data.c_str());
    }
}

