#include "drive_ros_localize_visual_odometry/simple_visual_odometry.h"


// main function
int main(int argc, char **argv)
{
  // set up node
  ros::init(argc, argv, "visual_odometry");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  // create visual odometry
  SimpleVisualOdometry visualOdometry(nh, pnh);


  // spin node normally
  while(ros::ok()){
    ros::spin();
  }


  return true;
}
