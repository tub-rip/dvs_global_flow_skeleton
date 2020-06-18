#pragma once

#include <opencv2/core/core.hpp>
#include <dvs_msgs/Event.h>


// Structure collecting the options for warping the events onto
// a histogram or image: the "image of warped events" (IWE)
struct OptionsWarp
{
  // Whether to use polarity or not in the IWE
  bool use_polarity_ = true;

  // Amounf ot Gaussian blur (in pixels) to make the IWE smoother,
  // and consequently, optimize a smoother objective function
  double blur_sigma_ = 1.0;
};


void computeImageOfWarpedEvents(
  const cv::Point2d& vel,
  const std::vector<dvs_msgs::Event>& events_subset,
  const cv::Size& img_size,
  cv::Mat* image_warped,
  const OptionsWarp& optsWarp
);
