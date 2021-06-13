#include <r1_laser_filter/speckle_filter.h>
#include <ros/node_handle.h>

namespace r1_laser_filter
{
  LaserScanSpeckleFilter::LaserScanSpeckleFilter()
  {
    validator_ = 0;
  }

  LaserScanSpeckleFilter::~LaserScanSpeckleFilter()
  {
    if (!validator_)
    {
      delete validator_;
    }
  }

  bool LaserScanSpeckleFilter::configure()
  {
    ros::NodeHandle private_nh("~" + getName());
    dyn_server_.reset(new dynamic_reconfigure::Server<r1_laser_filter::SpeckleFilterConfig>(own_mutex_, private_nh));
    dynamic_reconfigure::Server<r1_laser_filter::SpeckleFilterConfig>::CallbackType f;
    f = boost::bind(&r1_laser_filter::LaserScanSpeckleFilter::reconfigureCB, this, _1, _2);
    dyn_server_->setCallback(f);

    getParam("max_range", config_.max_range);
    getParam("max_range_difference", config_.max_range_difference);
    getParam("filter_window", config_.filter_window);
    dyn_server_->updateConfig(config_);
    return true;
  }

  bool LaserScanSpeckleFilter::update(const sensor_msgs::LaserScan &input_scan, sensor_msgs::LaserScan &output_scan)
  {
    output_scan = input_scan;
    std::vector<bool> valid_ranges(output_scan.ranges.size(), false);
    for (size_t idx = 0; idx < output_scan.ranges.size(); ++idx)
    {
      if (output_scan.ranges[idx] > config_.max_range)
      {
        valid_ranges[idx] = true;
      }
      else
      {
        valid_ranges[idx] = validator_->checkWindowValid(output_scan, idx, config_.filter_window, config_.max_range_difference);
      }

      if (!valid_ranges[idx])
      {
        output_scan.ranges[idx] = std::numeric_limits<float>::quiet_NaN();
      }
    }

    return true;
  }

  void LaserScanSpeckleFilter::reconfigureCB(r1_laser_filter::SpeckleFilterConfig &config, uint32_t level)
  {
    config_ = config;

    if (validator_)
    {
      delete validator_;
    }
    validator_ = new r1_laser_filter::DistanceWindowValidatorImproved();
  }
}
