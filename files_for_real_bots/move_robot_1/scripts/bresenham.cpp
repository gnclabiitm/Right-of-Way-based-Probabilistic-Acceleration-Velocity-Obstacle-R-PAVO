#include "grid_map_demos/IteratorsDemo.hpp"

// ROS
#include <geometry_msgs/PolygonStamped.h>

using namespace std;
using namespace ros;
using namespace grid_map;

namespace grid_map_demos {

IteratorsDemo::OccluLineIterators(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(vector<string>({"type"}))
{
  ROS_INFO("Grid map iterators node started.");
  polygonPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("Occlu_polygon", 1, true);

  publish();
  ros::Duration duration(2.0);
  duration.sleep();

  OccluLineIterator();
}

IteratorsDemo::~IteratorsDemo() {}
void IteratorsDemo::OccluLineIterator(start_x,start_y, end_x,end_y)
{
  ROS_INFO("Running line iterator.");
  map_.clearAll();
  publish();

  Index start(start_x, start_y);
  Index end(end_x, end_y);

  for (grid_map::LineIterator iterator(map_, start, end);
      !iterator.isPastEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}
}