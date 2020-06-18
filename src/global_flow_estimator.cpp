#include "dvs_global_flow/global_flow_estimator.h"
#include "dvs_global_flow/image_warped_events.h"
#include "dvs_global_flow/util.h"

#include <geometry_msgs/PointStamped.h>

#include <glog/logging.h>

#include <sstream>


namespace dvs_global_flow {

OptionsMethod loadBaseOptions(ros::NodeHandle& pnh)
{
  OptionsMethod opts;

  // Sliding window parameters
  opts.num_events_per_image_ = pnh.param("num_events_per_image", 5000);
  LOG(INFO) << "Found parameter: num_events_per_image = " << opts.num_events_per_image_;

  opts.num_events_slide_ = pnh.param("num_events_slide", 5000);
  LOG(INFO) << "Found parameter: num_events_slide = " << opts.num_events_slide_;

  // Objective function parameters
  opts.contrast_measure_ = pnh.param("contrast_measure", 0);
  LOG(INFO) << "Found parameter: contrast_measure = " << opts.contrast_measure_;

  // Event warping parameters
  opts.opts_warp_.use_polarity_ = pnh.param("use_polarity", true);
  LOG(INFO) << "Found parameter: use_polarity = " << ((opts.opts_warp_.use_polarity_) ? "true" : "false" );

  opts.opts_warp_.blur_sigma_ = pnh.param("gaussian_smoothing_sigma", 1.0);
  LOG(INFO) << "Found parameter: gaussian_smoothing_sigma = " << opts.opts_warp_.blur_sigma_;

  // Verbosity / printing level
  opts.verbose_ = pnh.param("verbosity", 0);
  LOG(INFO) << "Found parameter: verbosity = " << opts.verbose_;

  return opts;
}



GlobalFlowEstimator::
GlobalFlowEstimator(ros::NodeHandle& nh)
  : nh_(nh)
  , pnh_("~")
  , img_size_(0,0)
  , vel_(0.,0.)
{
  // Set up subscribers
  event_sub_ = nh_.subscribe("events", 0, &GlobalFlowEstimator::eventsCallback, this);

  // Set up publishers
  image_transport::ImageTransport it_(nh_);
  image_pub_ = it_.advertise("dvs_motion_compensated", 1);
  vel_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/dvs/global_flow", 1);

  opts_ = loadBaseOptions(pnh_);

  // Sliding window
  idx_first_ev_ = 0;  // Index of first event of processing window
  time_packet_ = ros::Time(0);
}


GlobalFlowEstimator::~GlobalFlowEstimator()
{
  image_pub_.shutdown();
  vel_pub_.shutdown();
}


void GlobalFlowEstimator::publishGlobalFlow()
{
  geometry_msgs::PointStamped global_flow_msg;
  global_flow_msg.point.x = vel_.x;
  global_flow_msg.point.y = vel_.y;
  global_flow_msg.point.z = 0.;
  global_flow_msg.header.stamp = time_packet_;
  vel_pub_.publish(global_flow_msg);
}


void GlobalFlowEstimator::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // Assume events are sorted in time and event messages come in correct order.
  // Read events, split the events into packets of constant number of events,
  // generate an image from those events and save/publish that image.

  // Append events of current message to the queue
  for(const dvs_msgs::Event& ev : msg->events)
    events_.push_back(ev);

  // Set image size with the arrival of the first messsage
  if(img_size_.height == 0)
  {
    img_size_ = cv::Size(msg->width, msg->height);
  }

  static unsigned int packet_number = 0;
  static unsigned long total_event_count = 0;
  total_event_count += msg->events.size();

  // If there are enough events in the queue, get subset of events
  while (events_.size() >= idx_first_ev_ + opts_.num_events_per_image_)
  {
    getSubsetOfEvents();

    // Initial value of global flow
    if(packet_number == 0)
    {
      findInitialFlow();
    }
    packet_number++;

    if (opts_.verbose_ >= 1)
    {
      LOG(INFO) << "Packet # " << packet_number << "  event# " << total_event_count;
    }

    // Process the events
    maximizeContrast();

    // Compute publication timestamp: midpoint between timestamps of events in the packet
    ros::Time time_first = events_subset_.front().ts;
    ros::Time time_last = events_subset_.back().ts;
    ros::Duration time_dt = time_last - time_first;
    time_packet_ = time_first + time_dt * 0.5;

    // Publish estimated motion parameters
    publishGlobalFlow();

    // Save / Publish image
    publishEventImage();

    // Slide the window, for next subset of events
    slideWindow();
  }
}


void GlobalFlowEstimator::publishEventImage()
{
  if (image_pub_.getNumSubscribers() <= 0)
    return;

  static cv::Mat image_original, image_warped, image_stacked;

  // Options to visualize the image of raw events and the image of warped events without blur
  OptionsWarp opts_warp_display = opts_.opts_warp_;
  opts_warp_display.blur_sigma_ = 0.;

  // Compute image of events assuming zero flow (i.e., without motion compensation)
  computeImageOfWarpedEvents(cv::Point2d(0.,0.), events_subset_, img_size_, &image_original,
                             opts_warp_display);

  // Compute motion compensated image with best flow
  computeImageOfWarpedEvents(vel_, events_subset_, img_size_, &image_warped,
                             opts_warp_display);

  // Stack both images side-by-side so that they are displayed with the same range
  concatHorizontal(image_original, image_warped, &image_stacked);

  if (opts_warp_display.use_polarity_)
  {
    // Visualize the image of warped events with the zero always at the mean grayscale level
        const float bmax = 5.f;
    image_stacked = (255.0f/(2.0f*bmax)) * (image_stacked + bmax);
  }
  else
  {
    // Scale the image to full range [0,255]
    cv::normalize(image_stacked, image_stacked, 0.f, 255.0f, cv::NORM_MINMAX, CV_32FC1);
    image_stacked = 255.0f - image_stacked; // invert "color": dark events over white background
  }

  // Publish images (without and with motion compensation)
  cv_event_image_.encoding = "mono8";
  image_stacked.convertTo(cv_event_image_.image,CV_8UC1);
  auto aux = cv_event_image_.toImageMsg();
  aux->header.stamp = time_packet_;
  image_pub_.publish(aux);
}


/** \brief Select vector(s) of events from the queue storing the events
 * \note In a separte function to remove clutter from main function
 */
void GlobalFlowEstimator::getSubsetOfEvents()
{
  events_subset_ = std::vector<dvs_msgs::Event>(events_.begin() + idx_first_ev_,
                                                events_.begin() + idx_first_ev_ + opts_.num_events_per_image_);
}


/** \brief Slide the window and remove those old events from the queue
 * \note In a separte function to remove clutter from main function
 */
void GlobalFlowEstimator::slideWindow()
{
  if ( opts_.num_events_slide_ <= events_.size() )
  {
    events_.erase(events_.begin(), events_.begin() + opts_.num_events_slide_);
    idx_first_ev_ = 0;
  } else
  {
    idx_first_ev_ += opts_.num_events_slide_;
  }
}

} // namespace
