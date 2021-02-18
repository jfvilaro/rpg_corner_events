#pragma once

#include <deque>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

#include <Eigen/Dense>

#include "corner_event_detector/detector.h"

namespace corner_event_detector
{

class FastDetector15 : public Detector
{
public:
  FastDetector15(int sensor_width, int sensor_height, bool connect = true);
  virtual ~FastDetector15();

  bool isFeature(const dvs_msgs::Event &e);

private:
  // SAE
  Eigen::MatrixXd sae_[2];

  // pixels on circle
  int circle6_[32][2];
  int circle7_[40][2];

  // parameters
  int sensor_width_ = 240;
  int sensor_height_ = 180;
};


} // namespace
