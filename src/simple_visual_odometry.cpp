#include "drive_ros_localize_visual_odometry/simple_visual_odometry.h"
#include "drive_ros_localize_visual_odometry/vo_features.h"




SimpleVisualOdometry::SimpleVisualOdometry(const ros::NodeHandle n, const ros::NodeHandle p):
  nh(n),  pnh(p), it(p)
{

  // reset parameters
  currentPosition.create(3,1,CV_64F);
  currentPosition.at<double>(0) = 0;
  currentPosition.at<double>(1) = 0;
  currentPosition.at<double>(2) = 1;
  transRotNew.create(3,3,CV_64F);
  transRotOld = cv::Mat::eye(3,3,CV_64F);

  // properly initialize
  oldImage =  cv_bridge::CvImagePtr(new cv_bridge::CvImage);


  //get parameters
  int x_min, x_max, y_min, y_max;
  pnh.getParam("roi/x_min", x_min);
  pnh.getParam("roi/x_max", x_max);
  pnh.getParam("roi/y_min", y_min);
  pnh.getParam("roi/y_max", y_max);
  roi = cv::Rect(x_min, y_min, x_max-x_min, y_max-y_min);
  ROS_INFO_STREAM("Loaded roi:\n" << roi);

  pnh.getParam("limits/dx_max", dx_max);
  pnh.getParam("limits/dx_min", dx_min);
  pnh.getParam("limits/dy_max", dy_max);
  pnh.getParam("limits/dy_min", dy_min);
  pnh.getParam("limits/theta_max", theta_max);
  pnh.getParam("limits/theta_min", theta_min);

  pnh.param<std::string>("static_frame", static_frame, "odometry");
  pnh.param<std::string>("moving_frame", moving_frame, "rear_axis_middle_ground");

  pnh.param<int>("minFeatureCount", minFeatureCount, 10);
  pnh.param<int>("fastThreshold", fastThreshold, 50);

  pnh.param<bool>("draw_debug", drawDebug, false);
  if(drawDebug)
  {
    cv::namedWindow("debug_image");
  }

  std::string cam_info_topic;
  pnh.param<std::string>("cam_info_topic", cam_info_topic);
  if(cam_info_topic.empty()){
    calRequired = false;
  }else{
    calRequired = true;
    cam_info_sub = pnh.subscribe(cam_info_topic, 1, &SimpleVisualOdometry::camInfoCb, this);
  }

  homoRequired = true;


  theta_total = 0;

  // publish
  odo_pub = pnh.advertise<nav_msgs::Odometry>("odom_out", 0);

  // subscribe to topics
  image_sub = it.subscribe("img_in", 1, &SimpleVisualOdometry::imageCb, this);
  homog_sub = pnh.subscribe("homog", 1, &SimpleVisualOdometry::homogCb, this);


  ROS_INFO("Simple Visual Odometry initialized.");
}

void SimpleVisualOdometry::camInfoCb(const sensor_msgs::CameraInfo& msg)
{
  if(calRequired){
    camera_model.fromCameraInfo(msg);
    calRequired = false;
    std::cout << camera_model.distortionCoeffs() << std::endl;
    ROS_INFO_STREAM("Got camera info message. Saving Parameters.");
  }

}


void SimpleVisualOdometry::imageCb(const sensor_msgs::ImageConstPtr& msg)
{

  // check if we got everything we need before starting
  if(calRequired || homoRequired)
  {
    return;
  }

  // convert message to cv pointer
  try
  {
    newImage = cv_bridge::toCvCopy(msg);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if(newImage->image.empty())
  {
    ROS_WARN("Empty image received, skipping!");
    return;
  }

  // undistort image if needed
  if(!camera_model.distortionCoeffs().empty()){
    cv_bridge::CvImagePtr undistortedImage;

    // we have to undistort the image
    cv::undistort(newImage->image, undistortedImage->image, camera_model.intrinsicMatrix(),
                                                            camera_model.distortionCoeffs());
    newImage = undistortedImage;
  }

  // check if old image is ok
  if(oldImage->image.empty())
  {
    oldImage = newImage;
    return;
  }


  // we try to find feature points in the old Image and try to redetect them in the new image
  // if we haven't found enough feature points in the new image we try to find more in the old image, therefore we store the old image
  // we crop the image to only find points on the road


  try{

      //clear tmp objects
      newImagePoints.clear();
      status.clear();

      bool alreadySearched = false;
      if(oldImagePoints.size() < minFeatureCount){
          alreadySearched = true;

          detectFeaturePointsInOldImage(roi, fastThreshold, oldImage->image);
          if(oldImagePoints.size() == 0){
              // save image as old image
              oldImage = newImage;
              ROS_WARN("No features detected!");
              return;
          }
      }

      ROS_DEBUG_STREAM("Number of old points: " << oldImagePoints.size());

      //track the old feature points
      featureTracking(roi, newImage->image, oldImage->image);

      // TODO improve this section
      if((int)newImagePoints.size() < minFeatureCount){
          ROS_WARN_STREAM("Not enough points tracked! New image points: " << newImagePoints.size());
          if(!alreadySearched){
              detectFeaturePointsInOldImage(roi,fastThreshold, oldImage->image);
          }else{
              ROS_WARN("already searched, found not enough points");
          }
          ROS_DEBUG_STREAM("detected new features: " << oldImagePoints.size());
          if(oldImagePoints.size() == 0){
              ROS_WARN("No features detected!");
          }else{
              featureTracking(roi, newImage->image, oldImage->image);
              ROS_DEBUG_STREAM("tracking new features: " << newImagePoints.size());
              if(newImagePoints.size() <= 1){
                  ROS_DEBUG_STREAM("Not enough features could be tracked! New image point size: " << newImagePoints.size());
              }
          }
      }


      // use ROS publisher instead
      if(drawDebug){

          cv::Mat colorImage;

          // convert to color
          cv::cvtColor(newImage->image, colorImage, CV_GRAY2BGR);

          // draw roi
          cv::rectangle(colorImage, roi, cv::Scalar(0, 0, 255));

          // draw old image points
          for(auto pt: oldImagePoints)
          {
            circle(colorImage, pt, 5, Scalar( 0, 255, 0 ), 2, 8 );
          }

          // draw new image points
          for(auto pt: newImagePoints)
          {
            circle(colorImage, pt, 5, Scalar( 255, 0, 0 ), 2, 8 );
          }

          // show image
          cv::imshow("debug_image", colorImage);
          cv::waitKey(1);


      }

      if(newImagePoints.size() > 1){

          //transform points to 2D-Coordinates
          std::vector<cv::Point2f> world_old, world_new;
          cv::perspectiveTransform(oldImagePoints, world_old, cam2world);
          cv::perspectiveTransform(newImagePoints, world_new, cam2world);

          //######################################################
          //from http://math.stackexchange.com/questions/77462/finding-transformation-matrix-between-two-2d-coordinate-frames-pixel-plane-to-w
          //create data
          cv::Mat leftSide, rightSide;
          rightSide.create(2*world_old.size(),1, CV_64F);
          leftSide.create(2*world_old.size(),4,CV_64F);
          for(std::size_t i = 0; i < 2*world_old.size(); i+=2){
              //we have the new points and would like to know how to get to the old ones as they moved closer to us
              leftSide.at<double>(i,0) = world_new[i/2].x;
              leftSide.at<double>(i,1) = -world_new[i/2].y;
              leftSide.at<double>(i,2) = 1;
              leftSide.at<double>(i,3) = 0;
              leftSide.at<double>(i+1,0) = world_new[i/2].y;
              leftSide.at<double>(i+1,1) = world_new[i/2].x;
              leftSide.at<double>(i+1,2) = 0;
              leftSide.at<double>(i+1,3) = 1;
              rightSide.at<double>(i,0) = world_old[i/2].x;
              rightSide.at<double>(i+1,0) = world_old[i/2].y;
          }
          //solve it
          cv::Mat res;
          //TODO we could use pseudo-inverse
          if(cv::solve(leftSide,rightSide,res,cv::DECOMP_SVD)){
              float dx = res.at<double>(2);
              float dy = res.at<double>(3);
              float theta = std::atan2(res.at<double>(1), res.at<double>(0));

              // check if valid
              if(validateMeasurement(dx, dy, theta)){

                  // update position
                  ROS_DEBUG_STREAM("update position: " << dx << " " << dy << " " << theta);

                  theta_total += theta;

                  float costh = cos(theta_total);
                  float sinth = sin(theta_total);

                  odo.pose.pose.position.x += costh*dx + sinth*dy;
                  odo.pose.pose.position.y += sinth*dx + costh*dy;

                  tf2::Quaternion q1;
                  q1.setRPY(0, 0, theta_total);

                  odo.pose.pose.orientation.w = q1.w();
                  odo.pose.pose.orientation.x = q1.x();
                  odo.pose.pose.orientation.y = q1.y();
                  odo.pose.pose.orientation.z = q1.z();

              }else{
                  ROS_WARN_STREAM("not updating, exceded limits: dx=" << dx << " dy=" << dy << " theta=" << theta);
              }
          }else{
              ROS_ERROR("solving SVD failed!");
          }
      }else{
          //we lost track, no update
      }


      odo.header.frame_id = static_frame;
      odo.child_frame_id = moving_frame;
      odo.header.stamp = ros::Time::now();


      geometry_msgs::TransformStamped tf;
      tf.header.frame_id = static_frame;
      tf.child_frame_id = moving_frame;
      tf.header.stamp = ros::Time::now();
      tf.transform.translation.x = odo.pose.pose.position.x;
      tf.transform.translation.y = odo.pose.pose.position.y;
      tf.transform.rotation.w = odo.pose.pose.orientation.w;
      tf.transform.rotation.x = odo.pose.pose.orientation.x;
      tf.transform.rotation.y = odo.pose.pose.orientation.y;
      tf.transform.rotation.z = odo.pose.pose.orientation.z;

      br.sendTransform(tf);
      odo_pub.publish(odo);

      //set old values
      oldImage = newImage;
      oldImagePoints = newImagePoints;
      oldMsgTime = msg->header.stamp;


      }catch(std::exception &e){
          ROS_ERROR_STREAM("exception thrown: " << e.what() << " reinitialising");

          // TODO reset
      }
}


void SimpleVisualOdometry::homogCb(const drive_ros_msgs::HomographyConstPtr& msg)
{

  // assume homography does not change during runtime
  if(homoRequired)
  {
    if(msg->cam2world.layout.dim[0].size != 3 ||
       msg->cam2world.layout.dim[1].size != 3 )
    {
      ROS_ERROR("Homography Matrix has wrong size!");
      return;
    }

    cam2world = cv::Mat::zeros(3,3,CV_64FC1);
    int k=0;
    for (int i=0; i < msg->cam2world.layout.dim[0].size; i++){
      for (int j=0; j < msg->cam2world.layout.dim[1].size; j++){
        cam2world.at<double>(i,j) = msg->cam2world.data[k++];
      }
    }

    homoRequired = false;
    ROS_INFO_STREAM("Homography msg received. Cam2World: \n" << cam2world);
  }

}




void SimpleVisualOdometry::detectFeaturePointsInOldImage(cv::Rect rect, const int fastThreshold, const cv::Mat& old_im){
    newImagePoints.clear();
    oldImagePoints.clear();
    status.clear();
    vo_features::featureDetection(old_im(rect), oldImagePoints, fastThreshold); //detect points
    for(cv::Point2f &v:oldImagePoints){
        v.x += rect.x;
        v.y += rect.y;
    }
    //TODO transform found points coord-sys of the full image
}

void SimpleVisualOdometry::featureTracking(cv::Rect roi, const cv::Mat& new_im, const cv::Mat& old_im){
    newImagePoints.clear();
    for(cv::Point2f &v:oldImagePoints){
        v.x -= roi.x;
        v.y -= roi.y;
    }
    vo_features::featureTracking(old_im(roi),new_im(roi),oldImagePoints,newImagePoints, status); //track those features to the new image
    for(cv::Point2f &v:newImagePoints){
        v.x += roi.x;
        v.y += roi.y;
    }
    for(cv::Point2f &v:oldImagePoints){
        v.x += roi.x;
        v.y += roi.y;
    }
    checkNewFeaturePoints(roi);
}

void SimpleVisualOdometry::checkNewFeaturePoints(const cv::Rect roi){
    if(newImagePoints.size() != oldImagePoints.size()){
        throw std::runtime_error( "newImagePoints size does not match oldImagePoints size" );
    }
    for(int i = 0; i < (int)newImagePoints.size(); ){
        if(roi.contains(newImagePoints[i])){
            i++;
        }else{
            newImagePoints.erase(newImagePoints.begin()+i);
            oldImagePoints.erase(oldImagePoints.begin()+i);
        }
    }
}



bool SimpleVisualOdometry::validateMeasurement(const float dx, const float dy, const float theta){
    if(std::isnan(dx)|| std::isnan(dy) || std::isnan(theta)){
        ROS_ERROR("dx or dy or theta is nan");
        return false;
    }
    if(dx > dx_max || dx < dx_min){
        return false;
    }
    if(dy > dy_max || dy < dy_min){
        return false;
    }
    if(theta > theta_max || theta < theta_min){
        return false;
    }
    return true;
}



//TODO We could try Kabasch_algoithm

/*
//test
//recovering the pose and the essential matrix
double focal = 718.8560;//https://en.wikipedia.org/wiki/Focal_length
cv::Point2d pp(image->width()/2, image->height()/2); //http://stackoverflow.com/questions/6658258/principle-point-in-camera-matrix-programming-issue
cv::Mat E, R, t, mask;
E = cv::findEssentialMat(tmpNewImagePoints, tmpOldImagePoints, focal, pp, RANSAC, 0.999, 1.0, mask);
cv::recoverPose(E, tmpNewImagePoints, tmpOldImagePoints, R, t, focal, pp, mask);
double scale = 1;
logger.error("wasd")<<t;
logger.error("wasd2")<<R;
t_f = t_f + scale*(R_f*t);
R_f = R*R_f;
cv::Mat newPos;
newPos.create(2,1,CV_32F);
newPos.at<double>(0)=t_f.at<double>(0);
newPos.at<double>(1)=t_f.at<double>(2);
logger.error("newPOS")<<newPos.at<double>(0)<<" "<<newPos.at<double>(1);
//END-test
*/
