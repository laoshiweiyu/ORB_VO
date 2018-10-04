#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;
/*
const std::string rgb_frame_ = "openni_rgb_optical_frame";
const std::string dep_frame_ = "openni_rgb_optical_frame";
const std::string lnk_frame_ = "openni_camera";
const std::string map_frame_ = "world";
*/
const std::string rgb_frame_ = "camera_rgb_optical_frame";
const std::string dep_frame_ = "camera_depth_optical_frame";
const std::string lnk_frame_ = "camera_link";
const std::string map_frame_ = "map";

tf::StampedTransform transform1;
tf::StampedTransform transform2;
Eigen::Matrix4f T1 = Eigen::Matrix4f::Zero();
Eigen::Matrix4f T2 = Eigen::Matrix4f::Zero();

ros::Publisher pose_pub;
ros::Publisher nav_pub;

class ImageGrabber
{
  public:
  ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
  void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
  ORB_SLAM2::System* mpSLAM;
};


Eigen::Matrix4f getTransformationMatFromMsg(const tf::StampedTransform &transform){

  Eigen::Matrix4d tf_mat = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q;
  q.x() = transform.getRotation().x();
  q.y() = transform.getRotation().y();
  q.z() = transform.getRotation().z();
  q.w() = transform.getRotation().w();
  Eigen::Matrix3d rot = q.normalized().toRotationMatrix();
  Eigen::Vector3d trns = Eigen::Vector3d(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
  tf_mat.block<3,3>(0,0) = rot;
  tf_mat.col(3).head<3>() = trns;
  return tf_mat.cast<float>();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "RGBD");
  ros::start();
  
  if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_settings rgb_image_topic depth_image_topic" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],ORB_SLAM2::System::RGBD);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, argv[2], 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, argv[3], 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("camera_trajectory",1);
    nav_pub = nh.advertise<nav_msgs::Odometry>("camera_orb_odom",1);

    //listen and spin
    tf::TransformListener listener;
    while(nh.ok() && T1.isZero(0) && T2.isZero(0)){
      try{
	listener.lookupTransform(rgb_frame_, lnk_frame_, ros::Time(0), transform1);
	T1 = getTransformationMatFromMsg(transform1);
	listener.lookupTransform(rgb_frame_, map_frame_, ros::Time(0), transform2);
	T2 = getTransformationMatFromMsg(transform2);
      }
      catch (tf::TransformException &ex) {
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
	continue;
      }
    }

    ros::spin();
    
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

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


    cv::Mat tf_mat = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    if (tf_mat.empty()){
      ROS_ERROR("TrackRGBD failed");
      return;
    }

    Eigen::Matrix4f tf_rel;
    tf_rel << tf_mat.at<float>(0,0), tf_mat.at<float>(0,1), tf_mat.at<float>(0,2), tf_mat.at<float>(0,3),
              tf_mat.at<float>(1,0), tf_mat.at<float>(1,1), tf_mat.at<float>(1,2), tf_mat.at<float>(1,3),
              tf_mat.at<float>(2,0), tf_mat.at<float>(2,1), tf_mat.at<float>(2,2), tf_mat.at<float>(2,3),
                                  0,                     0,                     0,                     1;
    Eigen::Matrix4f tf_off;
    tf_off << 0, -1,  0, 0,
             -1,  0,  0, 0,
              0,  0, -1, 0,
              0,  0,  0, 1;
    Eigen::Matrix4f tf_ini = T2.inverse();      
    //Eigen::Matrix4f tf_abs = tf_rel;
    Eigen::Matrix4f tf_abs = tf_ini*tf_rel.inverse();
    std::cout << tf_abs << std::endl;
    std::cout << "-----------" << std::endl;

    Eigen::Quaternionf q(tf_abs.block<3,3>(0,0));
    geometry_msgs::PoseStamped pos_msg;
    pos_msg.header.stamp = msgRGB->header.stamp;
    pos_msg.header.frame_id = map_frame_;
    pos_msg.pose.position.x = tf_abs(0,3);
    pos_msg.pose.position.y = tf_abs(1,3);
    pos_msg.pose.position.z = tf_abs(2,3);
    pos_msg.pose.orientation.x = q.x();
    pos_msg.pose.orientation.y = q.y();
    pos_msg.pose.orientation.z = q.z();
    pos_msg.pose.orientation.w = q.w();
    pose_pub.publish(pos_msg);

    std::cout << "PoseStamped msg published" << std::endl;

    nav_msgs::Odometry nav_msg;
    nav_msg.header.stamp = msgRGB->header.stamp;
    nav_msg.header.frame_id = map_frame_;
    nav_msg.pose.pose = pos_msg.pose;
    nav_pub.publish(nav_msg);

    std::cout << "Odometry msg published" << std::endl;

    /*
    std::string new_frame_ = "camera_pose_estimate";
    Eigen::Matrix4f tf = tf_out;
    Eigen::Quaternionf tfq(tf.block<3,3>(0,0));
<    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(tf(0,3), tf(1,3), tf(2,3)));
    tf::Quaternion quaternion(tfq.x(), tfq.y(), tfq.z(), tfq.w());
    trpansform.setRotation(quaternion);
    br.sendTransform(tf::StampedTransform(transform, msgRGB->header.stamp, new_frame_, map_frame_));
    */

    return;
}
