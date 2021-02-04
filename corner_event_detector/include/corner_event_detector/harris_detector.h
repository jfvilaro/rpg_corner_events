#pragma once

#include <deque>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

#include "corner_event_detector/detector.h"

#include "corner_event_detector/local_event_queues.h"
#include "corner_event_detector/distinct_queue.h"

namespace corner_event_detector
{

class HarrisDetector : public Detector
{
public:
  HarrisDetector(int sensor_width, int sensor_height, int harris_queue_size, int harris_window_size, int harris_kernel_size, double harris_threshold, bool connect = true);
  virtual ~HarrisDetector();

  bool isFeature(const dvs_msgs::Event &e);
  double getLastScore() const {
    return last_score_;
  }

private:
  // methods
  void updateQueue(const int x, const int y, const dvs_msgs::Event &e);
  double getHarrisScore(int x, int y, bool polarity);

  // queues
  LocalEventQueues* queues_;

  // parameters
  int queue_size_= 25;
  int window_size_= 4;
  int kernel_size_= 5;
  int sensor_width_ = 240;
  int sensor_height_ = 180;
  double harris_threshold_=8.0;
  
  double last_score_;

  // kernels
  Eigen::MatrixXd Gx_, h_;
  int factorial(int n) const;
  int pasc(int k, int n) const;
};


} // namespace
