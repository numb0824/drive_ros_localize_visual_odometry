#ifndef SIMPLE_VISUAL_ODOMETRY_H
#define SIMPLE_VISUAL_ODOMETRY_H

#include <ros/ros.h>
#include <opencv/cv.h>
#include "ctrv_vxy.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/TransformStamped.h>
#include "drive_ros_msgs/Homography.h"
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


class SimpleVisualOdometry {
    //lms::ReadDataChannel<lms::imaging::Image> image;
    //lms::WriteDataChannel<lms::imaging::Image> debugImage,trajectoryImage;
    //lms::WriteDataChannel<lms::math::Pose2DHistory> poseHistory;
    //lms::imaging::Image oldImage;
    ros::Subscriber cam_info_sub;
    ros::Subscriber homog_sub;
    image_transport::Subscriber image_sub;

    ros::Publisher odo_pub;
    tf2_ros::TransformBroadcaster br;


    std::vector<cv::Point2f> oldImagePoints;

    cv::Mat cam2world;

    cv::Rect roi;

    std::string static_frame;
    std::string moving_frame;

    // todo use other type (so that it is clear what this is)
    cv::Mat currentPosition;
    cv::Mat transRotNew,transRotOld;

    cv::Mat oldImage;

    //tmp objects
    std::vector<cv::Point2f> newImagePoints;
    std::vector<uchar> status;
    kalman_filters::ctrv_vxy::MassModelUKF ukf;

    // ros node handle
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    image_transport::ImageTransport it;


    bool drawDebug;
    int fastThreshold;
    int minFeatureCount;

    ros::Time oldMsgTime;

    bool calLoaded;
    bool homoLoaded;
    image_geometry::PinholeCameraModel camera_model;



public:

    SimpleVisualOdometry(const ros::NodeHandle nh, const ros::NodeHandle pnh);

    void camInfoCb(const sensor_msgs::CameraInfo& msg);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void homogCb(const drive_ros_msgs::HomographyConstPtr& msg);

    void detectFeaturePointsInOldImage(const cv::Rect rect, const int fastThreshold);
    void checkNewFeaturePoints(const cv::Rect rect);
    bool validateMeasurement(const float vx,const float vy,const float dPhi);
    void featureTracking(cv::Rect rect, const cv::Mat &newImage, const cv::Mat &oldImage);
};

#endif // SIMPLE_VISUAL_ODOMETRY_H
