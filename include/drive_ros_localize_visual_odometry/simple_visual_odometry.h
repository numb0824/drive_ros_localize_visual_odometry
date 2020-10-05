#ifndef SIMPLE_VISUAL_ODOMETRY_H
#define SIMPLE_VISUAL_ODOMETRY_H

#include <ros/ros.h>
#include "opencv2/opencv.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/TransformStamped.h>
#include "drive_ros_msgs/Homography.h"
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "drive_ros_localize_visual_odometry/moving_average.h"


class SimpleVisualOdometry {

    enum{
      SUCCESS = 0,
      FAIL = 1
    };


    // ros node handle
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    image_transport::ImageTransport it;

    // publisher
    ros::Publisher odo_pub;
    image_transport::Publisher ima_pub;
    tf2_ros::TransformBroadcaster br;

    // subscriber
    ros::Subscriber cam_info_sub;
    ros::Subscriber homog_sub;
    image_transport::Subscriber image_sub;
    image_geometry::PinholeCameraModel camera_model;
    bool calRequired;
    bool homoRequired;
    cv::Mat cam2world;
    ros::Time oldMsgTime;

    // output
    nav_msgs::Odometry odo;
    float theta_total;


    // todo use other type (so that it is clear what this is)
    cv::Mat currentPosition;
    cv::Mat transRotNew,transRotOld;

    cv_bridge::CvImagePtr oldImage;
    cv_bridge::CvImagePtr newImage;

    // region of interest (only search in this area)
    cv::Rect roi;

    // feature points
    std::vector<cv::Point2f> newImagePoints;
    std::vector<cv::Point2f> oldImagePoints;

    // tracking status
    std::vector<uchar> status;

    // parameter
    int fastThreshold;
    int minFeatureCount;

    std::string static_frame;
    std::string moving_frame;

    float dx_max;
    float dx_min;
    float dy_max;
    float dy_min;
    float theta_max;
    float theta_min;

    MovingAverage avg_dx, avg_dy, avg_theta;

public:

    SimpleVisualOdometry(const ros::NodeHandle nh, const ros::NodeHandle pnh);

    void camInfoCb(const sensor_msgs::CameraInfo& msg);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void homogCb(const drive_ros_msgs::HomographyConstPtr& msg);

    void detectFeaturePointsInOldImage(const cv::Rect rect, const int fastThreshold, const cv::Mat& old_im);
    bool checkNewFeaturePoints(const cv::Rect rect);
    bool validateMeasurement(const float vx,const float vy,const float dPhi);
    bool featureTracking(cv::Rect rect, const cv::Mat &new_im, const cv::Mat &old_im);

};

#endif // SIMPLE_VISUAL_ODOMETRY_H
