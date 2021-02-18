#include "corner_event_detector/fast_detector_15.h"

namespace corner_event_detector
{

FastDetector15::FastDetector15(int sensor_width, int sensor_height,bool connect)
: Detector(connect),
  circle6_ {{0, 6}, {1, 6}, {2, 5}, {3, 5},
		 {4, 4}, {5, 3}, {5, 2}, {6, 1},
		 {6, 0}, {6,-1}, {5,-2}, {5,-3},
		 {4, -4}, {3,-5}, {2,-5}, {1,-6},
		 {0, -6}, {-1,-6}, {-2,-5}, {-3,-5},
		 {-4, -4}, {-5,-3}, {-5,-2}, {-6,-1},
		 {-6, 0}, {-6,1}, {-5,2}, {-5,3},
		 {-4, 4}, {-3,5}, {-2,5}, {-1,6}},
  circle7_ {{0, 7}, {1, 7}, {2, 7}, {3, 6},
		  {4, 6}, {5, 5}, {6, 4}, {6, 3},
		  {7, 2}, {7, 1}, {7, 0}, {7, -1},
		  {7, -2}, {6, -3}, {6, -4}, {5, -5},
		  {4, -6}, {3, -6}, {2, -7}, {1, -7},
		  {0, -7}, {-1, -7}, {-2, -7}, {-3, -6},
		  {-4, -6}, {-5, -5}, {-6, -4}, {-6, -3},
		  {-7, -2}, {-7, -1}, {-7, 0}, {-7, 1},
		  {-7, 2}, {-6, 3}, {-6, 4}, {-5, 5},
	          {-4, 6}, {-3, 6}, {-2, 7}, {1, -7}}          
            
{
  detector_name_ = "FAST_15";

  // constants
  sensor_width_ = sensor_width;
  sensor_height_ = sensor_height;

  // allocate SAE matrices
  sae_[0] = Eigen::MatrixXd::Zero(sensor_height_, sensor_width_);
  sae_[1] = Eigen::MatrixXd::Zero(sensor_height_, sensor_width_);
  
}

FastDetector15::~FastDetector15()
{
}

bool FastDetector15::isFeature(const dvs_msgs::Event &e)
{
  // update SAE
  const int pol = e.polarity ? 1 : 0;
  sae_[pol](e.x, e.y) = e.ts.toSec();

  const int max_scale = 1;

  // only check if not too close to border
  const int cs = max_scale*7;
  if (e.x < cs || e.x >= sensor_width_-cs ||
      e.y < cs || e.y >= sensor_height_-cs)
  {
    return false;
  }

  bool found_streak = false;

  for (int i=0; i<32; i++)
  {
    for (int streak_size = 6; streak_size<=12; streak_size++)
    {
      // check that streak event is larger than neighbor
      if (sae_[pol](e.x+circle6_[i][0], e.y+circle6_[i][1]) <  sae_[pol](e.x+circle6_[(i-1+32)%32][0], e.y+circle6_[(i-1+32)%32][1]))
        continue;

      // check that streak event is larger than neighbor
      if (sae_[pol](e.x+circle6_[(i+streak_size-1)%32][0], e.y+circle6_[(i+streak_size-1)%32][1]) <          sae_[pol](e.x+circle6_[(i+streak_size)%32][0], e.y+circle6_[(i+streak_size)%32][1]))
        continue;

      double min_t = sae_[pol](e.x+circle6_[i][0], e.y+circle6_[i][1]);
      for (int j=1; j<streak_size; j++)
      {
        const double tj = sae_[pol](e.x+circle6_[(i+j)%32][0], e.y+circle6_[(i+j)%32][1]);
        if (tj < min_t)
          min_t = tj;
      }

      bool did_break = false;
      for (int j=streak_size; j<32; j++)
      {
        const double tj = sae_[pol](e.x+circle6_[(i+j)%32][0], e.y+circle6_[(i+j)%32][1]);

        if (tj >= min_t)
        {
          did_break = true;
          break;
        }
      }

      if (!did_break)
      {
        found_streak = true;
        break;
      }

    }
    if (found_streak)
    {
      break;
    }
  }

  if (found_streak)
  {
    found_streak = false;
    for (int i=0; i<40; i++)
    {
      for (int streak_size = 7; streak_size<=14; streak_size++)
      {
        // check that first event is larger than neighbor
        if (sae_[pol](e.x+circle7_[i][0], e.y+circle7_[i][1]) <  sae_[pol](e.x+circle7_[(i-1+40)%40][0], e.y+circle7_[(i-1+40)%40][1]))
          continue;

        // check that streak event is larger than neighbor
        if (sae_[pol](e.x+circle7_[(i+streak_size-1)%40][0], e.y+circle7_[(i+streak_size-1)%40][1]) <          sae_[pol](e.x+circle7_[(i+streak_size)%40][0], e.y+circle7_[(i+streak_size)%40][1]))
          continue;

        double min_t = sae_[pol](e.x+circle7_[i][0], e.y+circle7_[i][1]);
        for (int j=1; j<streak_size; j++)
        {
          const double tj = sae_[pol](e.x+circle7_[(i+j)%40][0], e.y+circle7_[(i+j)%40][1]);
          if (tj < min_t)
            min_t = tj;
        }

        bool did_break = false;
        for (int j=streak_size; j<40; j++)
        {
          const double tj = sae_[pol](e.x+circle7_[(i+j)%40][0], e.y+circle7_[(i+j)%40][1]);
          if (tj >= min_t)
          {
            did_break = true;
            break;
          }
        }

        if (!did_break)
        {
          found_streak = true;
          break;
        }
      }
      if (found_streak)
      {
        break;
      }
    }
  }

  return found_streak;
}

} // namespace
