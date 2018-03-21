#include "drive_ros_localize_visual_odometry/simple_visual_odometry.h"
#include "drive_ros_localize_visual_odometry/vo_features.h"

SimpleVisualOdometry::SimpleVisualOdometry(const ros::NodeHandle n, const ros::NodeHandle p):
  nh(n),  pnh(p), it(p)
{

  // reset parameters
  calLoaded = false;
  homoLoaded = false;

  currentPosition.create(3,1,CV_64F);
  currentPosition.at<double>(0) = 0;
  currentPosition.at<double>(1) = 0;
  currentPosition.at<double>(2) = 1;
  transRotNew.create(3,3,CV_64F);
  transRotOld = cv::Mat::eye(3,3,CV_64F);

  ukf.init();


  //get parameters
  int x_min, x_max, y_min, y_max;
  pnh.getParam("roi/x_min", x_min);
  pnh.getParam("roi/x_max", x_max);
  pnh.getParam("roi/y_min", y_min);
  pnh.getParam("roi/y_max", y_max);
  roi = cv::Rect(x_min, y_min, x_max-x_min, y_max-y_min);
  ROS_INFO_STREAM("Loaded roi:\n" << roi);

  pnh.param<std::string>("static_frame", static_frame, "odometry");
  pnh.param<std::string>("moving_frame", moving_frame, "rear_axis_middle_ground");

  pnh.param<int>("minFeatureCount", minFeatureCount, 10);
  pnh.param<int>("fastThreshold", fastThreshold, 50);

  pnh.param<bool>("draw_debug", drawDebug, false);
  if(drawDebug)
  {
    cv::namedWindow("debug_image");
  }


  // publish
  odo_pub = pnh.advertise<nav_msgs::Odometry>("odom_out", 0);

  // subscribe to topics
  image_sub = it.subscribe("img_in", 1, &SimpleVisualOdometry::imageCb, this);
  cam_info_sub = pnh.subscribe("cam_info", 1, &SimpleVisualOdometry::camInfoCb, this);
  homog_sub = pnh.subscribe("homog", 1, &SimpleVisualOdometry::homogCb, this);


  ROS_INFO("Simple Visual Odometry initialized.");
}

void SimpleVisualOdometry::camInfoCb(const sensor_msgs::CameraInfo& msg)
{
  if(!calLoaded){
    camera_model.fromCameraInfo(msg);
    calLoaded = true;
    std::cout << camera_model.distortionCoeffs() << std::endl;
    ROS_INFO_STREAM("Got camera info message. Saving Parameters.");
  }

}


void SimpleVisualOdometry::imageCb(const sensor_msgs::ImageConstPtr& msg)
{

  // check if we got everything we need before starting
  if(!calLoaded || !homoLoaded)
  {
    return;
  }

  // convert message to cv pointer
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if(cv_ptr->image.empty())
  {
    ROS_WARN("Empty image received, skipping!");
    return;
  }

  // undistort image
  cv::Mat newImage = cv_ptr->image;
  //cv::undistort(cv_ptr->image, newImage, camera_model.intrinsicMatrix(),
  //                                  camera_model.distortionCoeffs());
  // check if old image is ok
  if(oldImage.empty())
  {
    oldImage = newImage;
    return;
  }


  //we try to find feature points in the old Image and try to redetect them in the new image
  //if we haven't found enough feature points in the new image we try to find more in the old image, therefore we store the old image
  //we crop the image to only find points on the road


  try{
      //use the predict of the ukf to calculate new xy position if even if we haven't found feature points
      float dt = (msg->header.stamp - oldMsgTime).toSec();
      if(dt > 1 || dt < 0)
      {
        dt = 0.01;
      }

      // TODO
      dt = 0.1;
      ukf.predict(dt);


      //clear tmp objects
      newImagePoints.clear();
      status.clear();

      bool alreadySearched = false;
      if(oldImagePoints.size() < minFeatureCount){
          alreadySearched = true;
          /*
          oldImagePoints.clear();
          featureDetection(oldIm, oldImagePoints,fastThreshold); //detect points
          //TODO transform found points coord-sys of the full image
          */
          detectFeaturePointsInOldImage(roi, fastThreshold);
          if(oldImagePoints.size() == 0){
              // save image as old image
              oldImage = newImage;
              ROS_WARN("No features detected!");
              return;
          }
      }

      ROS_DEBUG_STREAM("Number of old points: " << oldImagePoints.size());
      //track the old feature points
      featureTracking(roi, newImage, oldImage);


      //TODO featureTracking(oldImFull,newIm,oldImagePoints,newImagePoints, status); //track those features to the new image
      //checkNewFeaturePoints(rect);
      if((int)newImagePoints.size() < minFeatureCount){
          ROS_WARN_STREAM("Not enough points tracked! New image points: " << newImagePoints.size());
          if(!alreadySearched){
              detectFeaturePointsInOldImage(roi,fastThreshold);
          }else{
              ROS_WARN("already searched, found not enough points");
          }
          ROS_DEBUG_STREAM("detected new features: " << oldImagePoints.size());
          if(oldImagePoints.size() == 0){
              ROS_WARN("No features detected!");
          }else{
              featureTracking(roi, newImage, oldImage);
              //TODO featureTracking(oldImFull,newIm,oldImagePoints,newImagePoints, status); //track those features to the new image
              //checkNewFeaturePoints(rect);
              ROS_DEBUG_STREAM("tracking new features: " << newImagePoints.size());
              if(newImagePoints.size() <= 1){
                  ROS_DEBUG_STREAM("Not enough features could be tracked! New image point size: " << newImagePoints.size());
              }
          }
      }



      if(drawDebug){

          cv::Mat colorImage;

          // convert to color
          cv::cvtColor(newImage, colorImage, CV_GRAY2BGR);

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
          std::vector<cv::Point2f> world_old,world_new;
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
              float angle = std::atan2(res.at<double>(1),res.at<double>(0));

              if(validateMeasurement(dx/dt,dy/dt,angle/dt)){
                  //update the ukf
                  ROS_DEBUG_STREAM("updating ukf" << dx/dt << " " << dy/dt << " " << angle/dt);




                  ukf.setMeasurementVec(dx/dt,dy/dt,angle/dt);
                  ukf.update();
                  /*
                  lms::imaging::BGRAImageGraphics traGraphics(*trajectoryImage);
                  if(drawDebug){
                      transRotNew.at<double>(0,0) = std::cos(angle);
                      transRotNew.at<double>(0,1) = -std::sin(angle);
                      transRotNew.at<double>(1,0) = std::sin(angle);
                      transRotNew.at<double>(1,1) = std::cos(angle);
                      transRotNew.at<double>(0,2) = dx;
                      transRotNew.at<double>(1,2) = dy;
                      transRotNew.at<double>(2,0) = 0;
                      transRotNew.at<double>(2,1) = 0;
                      transRotNew.at<double>(2,2) = 1;
                      //translate the current position
                      transRotOld = transRotOld*transRotNew;
                      //currentPosition = transRotNew*currentPosition;
                      cv::Mat newPos = transRotOld*currentPosition;
                      traGraphics.setColor(lms::imaging::red);
                      traGraphics.drawPixel(newPos.at<double>(0)*512/30+256,-newPos.at<double>(1)*512/30+256);
                  }
                  */
              }else{
                  ROS_WARN_STREAM("not updating ukf, invalid values: " << dx/dt << " " << dy/dt << " " << angle/dt);
              }
          }else{
              ROS_ERROR("solving SVD failed!");
          }
      }else{
          //we lost track, no update for the ukf
      }
      //add new pose

      //poseHistory->addPose(ukf.lastState.x(),ukf.lastState.y(),ukf.lastState.phi(),lms::Time::now().toFloat<std::milli, double>());


      tf2::Quaternion q1;
      q1.setRPY(0, 0, ukf.lastState.phi());

      nav_msgs::Odometry odo;
      odo.header.frame_id = "odom";
      odo.child_frame_id = "rear_axis_middle_ground";
      odo.header.stamp = ros::Time::now();
      odo.pose.pose.position.x = ukf.lastState.x();
      odo.pose.pose.position.y = ukf.lastState.y();
      odo.pose.pose.orientation.w = q1.w();
      odo.pose.pose.orientation.x = q1.x();
      odo.pose.pose.orientation.y = q1.y();
      odo.pose.pose.orientation.z = q1.z();




      geometry_msgs::TransformStamped tf;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "rear_axis_middle_ground";
      tf.header.stamp = ros::Time::now();
      tf.transform.translation.x = ukf.lastState.x();
      tf.transform.translation.y = ukf.lastState.y();
      tf.transform.rotation.w = q1.w();
      tf.transform.rotation.x = q1.x();
      tf.transform.rotation.y = q1.y();
      tf.transform.rotation.z = q1.z();

      br.sendTransform(tf);


      odo_pub.publish(odo);

      //set old values
      oldImage = newImage;
      oldImagePoints = newImagePoints;
      oldMsgTime = msg->header.stamp;
//      if(drawDebug){
//          lms::imaging::BGRAImageGraphics traGraphics(*trajectoryImage);
//          traGraphics.setColor(lms::imaging::blue);
//          traGraphics.drawPixel(poseHistory->currentPose().x*512/30+256,-poseHistory->currentPose().y*512/30+256);
//          //cv::namedWindow( "Camera", WINDOW_AUTOSIZE );
//          cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//          cv::imshow( "Display window", trajectoryImage->convertToOpenCVMat() );                   // Show our image inside it.

//          cv::waitKey(1);
//      }
        //oldImagePoints.clear();

      }catch(std::exception &e){
          ROS_ERROR_STREAM("exception thrown: " << e.what() << " reinitialising ukf");
          ukf.init(ukf.lastState.x(),ukf.lastState.y(),ukf.lastState.phi());
      }
}


void SimpleVisualOdometry::homogCb(const drive_ros_msgs::HomographyConstPtr& msg)
{

  // assume homography does not change during runtime
  if(!homoLoaded)
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

    homoLoaded = true;
    ROS_INFO_STREAM("Homography msg received. Cam2World: \n" << cam2world);
  }

}




void SimpleVisualOdometry::detectFeaturePointsInOldImage(cv::Rect rect, const int fastThreshold){
    newImagePoints.clear();
    oldImagePoints.clear();
    status.clear();
    vo_features::featureDetection(oldImage(rect), oldImagePoints, fastThreshold); //detect points
    for(cv::Point2f &v:oldImagePoints){
        v.x += rect.x;
        v.y += rect.y;
    }
    //TODO transform found points coord-sys of the full image
}

void SimpleVisualOdometry::featureTracking(cv::Rect roi, const cv::Mat& newImage, const cv::Mat& oldImage){
    newImagePoints.clear();
    for(cv::Point2f &v:oldImagePoints){
        v.x -= roi.x;
        v.y -= roi.y;
    }
    vo_features::featureTracking(oldImage(roi),newImage(roi),oldImagePoints,newImagePoints, status); //track those features to the new image
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




bool SimpleVisualOdometry::validateMeasurement(const float vx, const float vy, const float dPhi){
    if(std::isnan(vx)|| std::isnan(vy) || std::isnan(dPhi)){
        ROS_ERROR("vx or vy or dPhi is nan");
        return false;
    }
    float vxMax = 5;
    float vyMax = 5;
    float vxMin = -0.1;
    float vyMin = -0.1;
    float omegaMax = 0.5;
    if(vx > vxMax || vx < vxMin){
        return false;
    }
    if(vy > vyMax || vy < vyMin){
        return false;
    }
    if(std::fabs(dPhi) > omegaMax){
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
