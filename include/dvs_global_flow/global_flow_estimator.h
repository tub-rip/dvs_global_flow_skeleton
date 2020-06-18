#pragma once

#include <ros/ros.h>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <deque>

#include "image_warped_events.h"

enum {
  VARIANCE_CONTRAST, // 0
  MEAN_SQUARE_CONTRAST
};

namespace dvs_global_flow
{

// Options of the method
struct OptionsMethod
{
  // Sliding Window options
  // Number of events used to synthetize an image of warped events
  int num_events_per_image_ = 15000;
  // Amount of overlap between consecutive packets of events: a number >= 0. (no overlap) and <1.0 (full)
  int num_events_slide_ = 15000;

  // Objective function to be optimized: 0=Variance, 1=RMS, etc.
  int contrast_measure_ = VARIANCE_CONTRAST;

  // Options of the image of warped events
  OptionsWarp opts_warp_;

  // Verbosity / printing level
  unsigned int verbose_ = 0;
};


class GlobalFlowEstimator {
public:
  GlobalFlowEstimator(ros::NodeHandle& nh);
  ~GlobalFlowEstimator();

private:
  ros::NodeHandle nh_;   // Node handle used to subscribe to ROS topics
  ros::NodeHandle pnh_;  // Private node handle for reading parameters

  // Subscribers
  ros::Subscriber event_sub_;
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);

  // Publishers
  image_transport::Publisher image_pub_;
  cv_bridge::CvImage cv_event_image_;
  ros::Publisher vel_pub_;
  void publishGlobalFlow();
  void publishEventImage();
  ros::Time time_packet_;

  // Sliding window of events
  std::deque<dvs_msgs::Event> events_;
  std::vector<dvs_msgs::Event> events_subset_;
  int idx_first_ev_;  // index of first event of processing window
  void getSubsetOfEvents();
  void slideWindow();

  // Motion estimation
  cv::Point2d vel_; // global flow (vx,vy)
  OptionsMethod opts_;  // Options of the method
  cv::Mat event_image_warped_;
  void findInitialFlow();
  cv::Point2d findBestFlowInRangeBruteForce(const cv::Vec4d& vel_range,
                                            const cv::Vec2d& vel_step);
  double maximizeContrast();

  // Camera information (size, intrinsics, lens distortion)
  cv::Size img_size_;
};

} // namespace
