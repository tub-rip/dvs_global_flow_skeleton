#include <ros/ros.h>
#include "dvs_global_flow/global_flow_estimator.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char* argv[])
{
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "dvs_global_flow");

  ros::NodeHandle nh;

  dvs_global_flow::GlobalFlowEstimator global_flow_estimator(nh);

  ros::spin();

  return 0;
}
