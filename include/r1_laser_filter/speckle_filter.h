#ifndef SPECKLE_FILTER_H
#define SPECKLE_FILTER_H

#include <dynamic_reconfigure/server.h>
#include <filters/filter_base.h>
#include <r1_laser_filter/SpeckleFilterConfig.h>
#include <sensor_msgs/LaserScan.h>

namespace r1_laser_filter
{

  class WindowValidator
  {
  public:
    virtual bool checkWindowValid(const sensor_msgs::LaserScan &scan, size_t idx, size_t window, double max_range_difference) = 0;
  };

  class DistanceWindowValidatorImproved : public WindowValidator
  {
    virtual bool checkWindowValid(const sensor_msgs::LaserScan &scan, size_t idx, size_t window, double max_range_difference)
    {
      int num_valid_neighbors = 0;
      const float &range = scan.ranges[idx];

      if (std::isnan(range))
      {
        return false;
      }

      std::vector<size_t> realNeighborsIndexes;
      for (int i = idx - 1; i >= 0; i--)
      {
        if (!std::isnan(scan.ranges[i]))
        {
          realNeighborsIndexes.push_back(i);
          if (realNeighborsIndexes.size() == (int)window)
          {
            break;
          }
        }
      }
      for (int i = idx + 1; i < scan.ranges.size(); i++)
      {
        if (!std::isnan(scan.ranges[i]))
        {
          realNeighborsIndexes.push_back(i);
          if (realNeighborsIndexes.size() == 2 * ((int)window))
          {
            break;
          }
        }
      }

      for (int i = 0; i < realNeighborsIndexes.size(); i++)
      {
        const float &neighbor_range = scan.ranges[realNeighborsIndexes[i]];
        if (fabs(neighbor_range - range) <= max_range_difference)
        {
          num_valid_neighbors++;
        }
      }

      if (num_valid_neighbors < window)
      {
        return false;
      }
      return true;
    }
  };

  /**
 * @brief This is a filter that removes speckle points in a laser scan based on consecutive ranges
 */
  class LaserScanSpeckleFilter : public filters::FilterBase<sensor_msgs::LaserScan>
  {
  public:
    LaserScanSpeckleFilter();
    ~LaserScanSpeckleFilter();
    bool configure();
    bool update(const sensor_msgs::LaserScan &input_scan, sensor_msgs::LaserScan &output_scan);

  private:
    std::shared_ptr<dynamic_reconfigure::Server<r1_laser_filter::SpeckleFilterConfig>> dyn_server_;
    void reconfigureCB(r1_laser_filter::SpeckleFilterConfig &config, uint32_t level);
    boost::recursive_mutex own_mutex_;

    SpeckleFilterConfig config_ = SpeckleFilterConfig::__getDefault__();
    WindowValidator *validator_;
  };
}
#endif /* speckle_filter.h */
