#include <ros/ros.h>

#include "corner_event_detector/detector.h"
#include "corner_event_detector/harris_detector.h"
#include "corner_event_detector/fast_detector.h"
#include "corner_event_detector/fast_detector_15.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "corner_event_detector");
  ros::NodeHandle nh("~");

  // load parameter
  std::string feature_type;
  int sensor_width;
  int sensor_height;
  int harris_queue_size, harris_kernel_size, harris_window_size;
  double harris_threshold;
  ros::param::param<std::string>("~feature_type", feature_type, "harris");
  // Sensor dimensions params
  nh.param<int>("sensor_width", sensor_width, 7); 
  nh.param<int>("sensor_height", sensor_height, 7);
  // Harris params
  nh.param<int>("harris_queue_size", harris_queue_size, 25); 
  nh.param<int>("harris_window_size", harris_window_size, 4); 
  nh.param<int>("harris_kernel_size", harris_kernel_size, 5); 
  nh.param<double>("harris_threshold", harris_threshold, 8.0); 

  // create feature detecotr
  corner_event_detector::Detector* detector;
  if (feature_type == "harris")
  {
    ROS_INFO("Using Harris detector.");
    detector = new corner_event_detector::HarrisDetector(sensor_width, sensor_height, harris_queue_size, harris_window_size, harris_kernel_size, harris_threshold);
  }
  else if (feature_type == "fast")
  {
    ROS_INFO("Using fast detector.");
    detector = new corner_event_detector::FastDetector(sensor_width, sensor_height);
  }
  else if (feature_type == "fast_15")
  {
    ROS_INFO("Using fast detector 15 by 15 pixels.");
    detector = new corner_event_detector::FastDetector15(sensor_width, sensor_height);
  }
  else
  {
    ROS_ERROR("Feature type '%s' is unknown.", feature_type.c_str());
    return 1;
  }

  // run
  ros::spin();

  delete detector;

  return 0;
}
