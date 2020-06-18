#include "dvs_global_flow/image_warped_events.h"
#include <opencv2/imgproc/imgproc.hpp>


void warpEvent(
  const cv::Point2d& vel,
  const dvs_msgs::Event& event,
  const double t_ref,
  cv::Point2d* warped_pt
)
{
  // Warp event according to flow model: displacement = velocity * time
  // FILL IN ...
}


void accumulateWarpedEvent(
  const dvs_msgs::Event& event,
  const int img_width,
  const int img_height,
  const cv::Point2d& ev_warped_pt,
  cv::Mat* image_warped,
  const OptionsWarp& optsWarp
)
{
  const float polarity = (optsWarp.use_polarity_) ? 2.f * static_cast<float>(event.polarity) - 1.f : 1.f;

  // Accumulate warped events, using bilinear voting (polarity or count)
  const int xx = ev_warped_pt.x,
            yy = ev_warped_pt.y;

  // if warped point is within the image, accumulate polarity
  if (1 <= xx && xx < img_width-2 && 1 <= yy && yy < img_height-2)
  {
    // Accumulate warped events on the IWE
    // FILL IN ...  image_warped (4 pixels)
  }
}


void computeImageOfWarpedEvents(
  const cv::Point2d& vel,
  const std::vector<dvs_msgs::Event>& events_subset,
  const cv::Size& img_size,
  cv::Mat* image_warped,
  const OptionsWarp& optsWarp
)
{
  const int width = img_size.width;
  const int height = img_size.height;

  // Create image of warped events (IWE)
  // FILL IN ...
  // hint:  *image_warped = ...

  // Loop through all events
  const double t_ref = events_subset.front().ts.toSec(); // warp wrt 1st event
  for (const dvs_msgs::Event& ev : events_subset)
  {
    // Warp event according to candidate flow and accumulate on the IWE
    // FILL IN ...
    // hint: Call warpEvent() and accumulateWarpedEvent()
  }

  // Smooth the IWE (to spread the votes)
  if (optsWarp.blur_sigma_ > 0)
  {
    // FILL IN ...
    // hint: cv::GaussianBlur()
  }
}
