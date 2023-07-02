/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "obstacle_detector/obstacle_extractor.h"
#include "obstacle_detector/utilities/figure_fitting.h"
#include "obstacle_detector/utilities/math_utilities.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;
using namespace obstacle_detector;

ObstacleExtractor::ObstacleExtractor(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  params_srv_ = nh_local_.advertiseService("params", &ObstacleExtractor::updateParams, this);
  initialize();
}

ObstacleExtractor::~ObstacleExtractor() {
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("use_scan");
  nh_local_.deleteParam("use_pcl");
  nh_local_.deleteParam("use_pcl2");

  nh_local_.deleteParam("segments_as_fov");

  nh_local_.deleteParam("use_split_and_merge");
  nh_local_.deleteParam("circles_from_visibles");
  nh_local_.deleteParam("circles_from_foreground");
  nh_local_.deleteParam("discard_converted_segments");
  nh_local_.deleteParam("transform_coordinates");

  nh_local_.deleteParam("min_group_points");

  nh_local_.deleteParam("max_group_distance");
  nh_local_.deleteParam("distance_proportion");
  nh_local_.deleteParam("max_split_distance");
  nh_local_.deleteParam("max_merge_separation");
  nh_local_.deleteParam("max_merge_spread");
  nh_local_.deleteParam("max_circle_radius");
  nh_local_.deleteParam("radius_enlargement");

  nh_local_.deleteParam("min_range");
  nh_local_.deleteParam("max_range");
  nh_local_.deleteParam("min_x_limit");
  nh_local_.deleteParam("max_x_limit");
  nh_local_.deleteParam("min_y_limit");
  nh_local_.deleteParam("max_y_limit");
  nh_local_.deleteParam("NULL_H");

  nh_local_.deleteParam("frame_id");
}

bool ObstacleExtractor::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("use_scan", p_use_scan_, true);
  nh_local_.param<bool>("use_pcl", p_use_pcl_, false);
  nh_local_.param<bool>("use_pcl2", p_use_pcl2_, false);

  nh_local_.param<bool>("segments_as_fov", p_segments_as_fov_, false);

  nh_local_.param<bool>("use_split_and_merge", p_use_split_and_merge_, true);
  nh_local_.param<bool>("circles_from_visibles", p_circles_from_visibles_, true);
  nh_local_.param<bool>("circles_from_visibles", p_circles_from_foreground_, true);
  nh_local_.param<bool>("discard_converted_segments", p_discard_converted_segments_, true);
  nh_local_.param<bool>("transform_coordinates", p_transform_coordinates_, true);

  nh_local_.param<int>("min_group_points", p_min_group_points_, 5);

  nh_local_.param<double>("max_group_distance", p_max_group_distance_, 0.1);
  nh_local_.param<double>("distance_proportion", p_distance_proportion_, 0.00628);
  nh_local_.param<double>("max_split_distance", p_max_split_distance_, 0.2);
  nh_local_.param<double>("max_merge_separation", p_max_merge_separation_, 0.2);
  nh_local_.param<double>("max_merge_spread", p_max_merge_spread_, 0.2);
  nh_local_.param<double>("max_circle_radius", p_max_circle_radius_, 0.6);
  nh_local_.param<double>("radius_enlargement", p_radius_enlargement_, 0.25);

  nh_local_.param<double>("min_range", p_min_range_, 0.1); // The min range for an input point
  nh_local_.param<double>("max_range", p_max_range_, 0.1); // The max range for an input point
  nh_local_.param<double>("min_x_limit", p_min_x_limit_, -10.0);
  nh_local_.param<double>("max_x_limit", p_max_x_limit_,  10.0);
  nh_local_.param<double>("min_y_limit", p_min_y_limit_, -10.0);
  nh_local_.param<double>("max_y_limit", p_max_y_limit_,  10.0);
  nh_local_.param<double>("NULL_H",      p_NULL_H_,      -1.0);

  nh_local_.param<string>("frame_id", p_frame_id_, "map");

  if (p_active_ != prev_active) {
    if (p_active_) {
      if (p_use_scan_)
        scan_sub_ = nh_.subscribe("scan", 10, &ObstacleExtractor::scanCallback, this);
      else if (p_use_pcl_)
        pcl_sub_ = nh_.subscribe("pcl", 10, &ObstacleExtractor::pclCallback, this);
      else if (p_use_pcl2_)
        pcl2_sub_ = nh_.subscribe("pcl2", 10, &ObstacleExtractor::pcl2Callback, this);

      obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("raw_obstacles", 10);
    }
    else {
      // Send empty message
      obstacle_detector::ObstaclesPtr obstacles_msg(new obstacle_detector::Obstacles);
      obstacles_msg->header.frame_id = p_frame_id_;
      obstacles_msg->header.stamp = ros::Time::now();
      obstacles_pub_.publish(obstacles_msg);

      scan_sub_.shutdown();
      pcl_sub_.shutdown();
      pcl2_sub_.shutdown();
      obstacles_pub_.shutdown();
    }
  }

  return true;
}

void ObstacleExtractor::scanCallback(const sensor_msgs::LaserScan::ConstPtr scan_msg) {
  base_frame_id_ = scan_msg->header.frame_id;
  stamp_ = scan_msg->header.stamp;

  double phi = scan_msg->angle_min;

  for (const float r : scan_msg->ranges) {
    if (r >= scan_msg->range_min && r <= scan_msg->range_max)
      input_points_.push_back(Point::fromPoolarCoords(r, phi));

    phi += scan_msg->angle_increment;
  }

  processPoints();
}

void ObstacleExtractor::pclCallback(const sensor_msgs::PointCloud::ConstPtr pcl_msg) {
  base_frame_id_ = pcl_msg->header.frame_id;
  stamp_ = pcl_msg->header.stamp;

  for (const geometry_msgs::Point32& point : pcl_msg->points)
    input_points_.push_back(Point(point.x, point.y));

  processPoints();
}

void ObstacleExtractor::pcl2Callback(const PointCloud2::ConstPtr pcl_msg) {
  // It is assumed that the pcl is ordered as a series of rows read from right to left (increasing y)
  if (processing_) {
    return;
  }

  base_frame_id_ = pcl_conversions::fromPCL(pcl_msg->header).frame_id;
  stamp_ = pcl_conversions::fromPCL(pcl_msg->header).stamp;

  for (const pcl::PointXYZ& pt : pcl_msg->points) {
    if (!(pt.z==p_NULL_H_) && pt.x<p_max_range_ && pt.x>p_min_range_) {
      input_points_.push_back(Point(pt.x, pt.y));
    }
  }

  processPoints();
}

void ObstacleExtractor::processPoints() {
  processing_ = true;
  segments_.clear();
  circles_.clear();

  groupPoints();  // Grouping points simultaneously detects segments
  mergeSegments();

  detectForeground();

  detectCircles();
  mergeCircles();

  publishObstacles();

  input_points_.clear();
  processing_ = false;
}

void ObstacleExtractor::groupPoints() {
  static double sin_dp = sin(2.0 * p_distance_proportion_);

  PointSet point_set;
  point_set.begin = input_points_.begin();
  point_set.end = input_points_.begin();
  point_set.num_points = 1;
  point_set.is_visible = true;

  for (PointIterator point = input_points_.begin()++; point != input_points_.end(); ++point) {
    double range = (*point).length();
    double distance = (*point - *point_set.end).length();

    if (distance < p_max_group_distance_ + range * p_distance_proportion_) {
      point_set.end = point;
      point_set.num_points++;
    }
    else {
      double prev_range = (*point_set.end).length();

      // Heron's equation
      double p = (range + prev_range + distance) / 2.0;
      double S = sqrt(p * (p - range) * (p - prev_range) * (p - distance));
      double sin_d = 2.0 * S / (range * prev_range); // Sine of angle between beams

      // TODO: This condition can be fulfilled if the point are on the opposite sides
      // of the scanner (angle = 180 deg). Needs another check.
      if (abs(sin_d) < sin_dp && range < prev_range)
        point_set.is_visible = false;

      detectSegments(point_set);

      // Begin new point set
      point_set.begin = point;
      point_set.end = point;
      point_set.num_points = 1;
      point_set.is_visible = (abs(sin_d) > sin_dp || range < prev_range);
    }
  }

  detectSegments(point_set); // Check the last point set too!
}

void ObstacleExtractor::detectSegments(const PointSet& point_set) {
  if (point_set.num_points < p_min_group_points_)
    return;

  Segment segment(*point_set.begin, *point_set.end);  // Use Iterative End Point Fit

  if (p_use_split_and_merge_)
    segment = fitSegment(point_set);

  PointIterator set_divider;
  double max_distance = 0.0;
  double distance     = 0.0;

  int split_index = 0; // Natural index of splitting point (counting from 1)
  int point_index = 0; // Natural index of current point in the set

  // Seek the point of division
  for (PointIterator point = point_set.begin; point != point_set.end; ++point) {
    ++point_index;

    if ((distance = segment.distanceTo(*point)) >= max_distance) {
      double r = (*point).length();

      if (distance > p_max_split_distance_ + r * p_distance_proportion_) {
        max_distance = distance;
        set_divider = point;
        split_index = point_index;
      }
    }
  }

  // Split the set only if the sub-groups are not 'small'
  if (max_distance > 0.0 && split_index > p_min_group_points_ && split_index < point_set.num_points - p_min_group_points_) {
    set_divider = input_points_.insert(set_divider, *set_divider);  // Clone the dividing point for each group

    PointSet subset1, subset2;
    subset1.begin = point_set.begin;
    subset1.end = set_divider;
    subset1.num_points = split_index;
    subset1.is_visible = point_set.is_visible;

    subset2.begin = ++set_divider;
    subset2.end = point_set.end;
    subset2.num_points = point_set.num_points - split_index;
    subset2.is_visible = point_set.is_visible;

    detectSegments(subset1);
    detectSegments(subset2);
  } else {  // Add the segment
    if (!p_use_split_and_merge_)
      segment = fitSegment(point_set);

    segments_.push_back(segment);
  }
}

void ObstacleExtractor::mergeSegments() {
  for (auto i = segments_.begin(); i != segments_.end(); ++i) {
    for (auto j = i; j != segments_.end(); ++j) {
      Segment merged_segment;

      if (compareSegments(*i, *j, merged_segment)) {
        auto temp_itr = segments_.insert(i, merged_segment);
        segments_.erase(i);
        segments_.erase(j);
        i = --temp_itr; // Check the new segment against others
        break;
      }
    }
  }
}

bool ObstacleExtractor::compareSegments(const Segment& s1, const Segment& s2, Segment& merged_segment) {
  if (&s1 == &s2)
    return false;

  // Segments must be provided counter-clockwise
  if (s1.first_point.cross(s2.first_point) < 0.0)
    return compareSegments(s2, s1, merged_segment);

  if (checkSegmentsProximity(s1, s2)) {
    vector<PointSet> point_sets;
    point_sets.insert(point_sets.end(), s1.point_sets.begin(), s1.point_sets.end());
    point_sets.insert(point_sets.end(), s2.point_sets.begin(), s2.point_sets.end());

    Segment segment = fitSegment(point_sets);

    if (checkSegmentsCollinearity(segment, s1, s2)) {
      merged_segment = segment;
      return true;
    }
  }

  return false;
}

bool ObstacleExtractor::checkSegmentsProximity(const Segment& s1, const Segment& s2) {
  return (s1.trueDistanceTo(s2.first_point) < p_max_merge_separation_ ||
          s1.trueDistanceTo(s2.last_point)  < p_max_merge_separation_ ||
          s2.trueDistanceTo(s1.first_point) < p_max_merge_separation_ ||
          s2.trueDistanceTo(s1.last_point)  < p_max_merge_separation_);
}

bool ObstacleExtractor::checkSegmentsCollinearity(const Segment& segment, const Segment& s1, const Segment& s2) {
  return (segment.distanceTo(s1.first_point) < p_max_merge_spread_ &&
          segment.distanceTo(s1.last_point)  < p_max_merge_spread_ &&
          segment.distanceTo(s2.first_point) < p_max_merge_spread_ &&
          segment.distanceTo(s2.last_point)  < p_max_merge_spread_);
}

void ObstacleExtractor::detectForeground() {
  // For the first segment
  auto seg = segments_.begin();
  seg->is_foreground = false;
  if (next(seg) == segments_.end()) {
    // Then there is only one segment
    seg->is_foreground = true;
    return;
  }
  if (seg->last_point.length() < next(seg)->first_point.length() ) {
    seg->is_foreground = true;
  }

  // For all the other segments
  for (auto seg = next(segments_.begin()); seg!=prev(segments_.end()); ++seg) {
    seg->is_foreground = false;
    if (seg->last_point.y<0 && next(seg)->first_point.y>0) {
      // Then seg is on the right border of a scan
      if (seg->first_point.length() < prev(seg)->last_point.length()) {
        seg->is_foreground = true;
      }
    } else if (seg->first_point.y>0 && prev(seg)->last_point.y<0) {
      // Then seg is on the left border of a scan
      if (seg->last_point.length() < next(seg)->first_point.length() ) {
        seg->is_foreground = true;
      }
    } else {
      // Then it is a normal segment, not on any border
      if (seg->last_point.length() < next(seg)->first_point.length()
      && seg->first_point.length() < prev(seg)->last_point.length()) {
        seg->is_foreground = true;
      }
    }
  } // end for loop.

  // For the last segment
  seg = prev(segments_.end());
  seg->is_foreground = false;
  if (seg->first_point.length() < prev(seg)->last_point.length()) {
    seg->is_foreground = true;
  }
  return; // end of the function
}

void ObstacleExtractor::detectCircles() {
  for (auto segment = segments_.begin(); segment != segments_.end(); ++segment) {
    if (p_circles_from_visibles_) {
      bool segment_is_visible = true;
      for (const PointSet& ps : segment->point_sets) {
        if (!ps.is_visible) {
          segment_is_visible = false;
          break;
        }
      }
      if (!segment_is_visible)
        continue;
    }

    // If only keep segments from foreground, skip the others
    if (p_circles_from_foreground_ && !segment->is_foreground) {
      continue;
    }

    Circle circle(*segment);
    circle.radius += p_radius_enlargement_;

    if (circle.radius < p_max_circle_radius_) {
      circles_.push_back(circle);

      if (p_discard_converted_segments_) {
        segment = segments_.erase(segment);
        --segment;
      }
    }
  }
}

void ObstacleExtractor::mergeCircles() {
  for (auto i = circles_.begin(); i != circles_.end(); ++i) {
    for (auto j = i; j != circles_.end(); ++j) {
      Circle merged_circle;

      if (compareCircles(*i, *j, merged_circle)) {
        auto temp_itr = circles_.insert(i, merged_circle);
        circles_.erase(i);
        circles_.erase(j);
        i = --temp_itr;
        break;
      }
    }
  }
}

bool ObstacleExtractor::compareCircles(const Circle& c1, const Circle& c2, Circle& merged_circle) {
  if (&c1 == &c2)
    return false;

  // If circle c1 is fully inside c2 - merge and leave as c2
  if (c2.radius - c1.radius >= (c2.center - c1.center).length()) {
    merged_circle = c2;
    return true;
  }

  // If circle c2 is fully inside c1 - merge and leave as c1
  if (c1.radius - c2.radius >= (c2.center - c1.center).length()) {
    merged_circle = c1;
    return true;
  }

  // If circles intersect and are 'small' - merge
  if (c1.radius + c2.radius >= (c2.center - c1.center).length()) {
    Point center = c1.center + (c2.center - c1.center) * c1.radius / (c1.radius + c2.radius);
    double radius = (c1.center - center).length() + c1.radius;

    Circle circle(center, radius);
    circle.radius += max(c1.radius, c2.radius);

    if (circle.radius < p_max_circle_radius_) {
      circle.point_sets.insert(circle.point_sets.end(), c1.point_sets.begin(), c1.point_sets.end());
      circle.point_sets.insert(circle.point_sets.end(), c2.point_sets.begin(), c2.point_sets.end());
      merged_circle = circle;
      return true;
    }
  }

  return false;
}

void ObstacleExtractor::publishObstacles() {
  obstacle_detector::ObstaclesPtr obstacles_msg(new obstacle_detector::Obstacles);
  obstacles_msg->header.stamp = stamp_;
  tf::StampedTransform transform;

  if (p_transform_coordinates_) {
    try {
      tf_listener_.waitForTransform(p_frame_id_, base_frame_id_, stamp_, ros::Duration(0.1));
      tf_listener_.lookupTransform(p_frame_id_, base_frame_id_, stamp_, transform);
    }
    catch (tf::TransformException& ex) {
      ROS_INFO_STREAM(ex.what());
      return;
    }

    for (Circle& c : circles_)
      c.center = transformPoint(c.center, transform);

    obstacles_msg->header.frame_id = p_frame_id_;
  }
  else
    obstacles_msg->header.frame_id = base_frame_id_;

  // If the option is selected, use the segments info to store the fov
  if (p_segments_as_fov_) {
    double d;
    Point point1;
    Point point2;

    // First get the camera point
    Point origin = Point(transform.getOrigin().x(), transform.getOrigin().y());

    // Initiate the two segments delimiting the fov
    SegmentObstacle fseg; // first point's segment
    SegmentObstacle lseg; // last point's segment
    fseg.first_point.x = origin.x;
    fseg.first_point.y = origin.y;
    lseg.first_point.x = origin.x;
    lseg.first_point.y = origin.y;

    // Use the first point of the input_points to give direction and scale it using max_x param
    auto p1 = input_points_.begin();
    point1 = transformPoint(*p1, transform);
    d = (point1-origin).length(); // distance
    point2 = origin + (point1-origin)/d*p_max_x_limit_;
    fseg.last_point.x = point2.x;
    fseg.last_point.y = point2.y;

    // Idem for the last segment, using the last point of inputs
    p1 = prev(input_points_.end());
    point1 = transformPoint(*p1, transform);
    d = (point1-origin).length(); // distance
    point2 = origin + (point1-origin)/d*p_max_x_limit_;
    lseg.last_point.x = point2.x;
    lseg.last_point.y = point2.y;

    obstacles_msg->segments.push_back(fseg);
    obstacles_msg->segments.push_back(lseg);
  } else {
    // Transform segments
    for (Segment& s : segments_) {
      s.first_point = transformPoint(s.first_point, transform);
      s.last_point = transformPoint(s.last_point, transform);
    }
    // Add segments to obstacles messages
    for (const Segment& s : segments_) {
      SegmentObstacle segment;

      segment.first_point.x = s.first_point.x;
      segment.first_point.y = s.first_point.y;
      segment.last_point.x = s.last_point.x;
      segment.last_point.y = s.last_point.y;

      obstacles_msg->segments.push_back(segment);
    }
  }

  for (const Circle& c : circles_) {
    if (c.center.x > p_min_x_limit_ && c.center.x < p_max_x_limit_ &&
        c.center.y > p_min_y_limit_ && c.center.y < p_max_y_limit_) {
        CircleObstacle circle;

        circle.center.x = c.center.x;
        circle.center.y = c.center.y;
        circle.velocity.x = 0.0;
        circle.velocity.y = 0.0;
        circle.radius = c.radius;
        circle.true_radius = c.radius - p_radius_enlargement_;

        obstacles_msg->circles.push_back(circle);
    }
  }

  obstacles_pub_.publish(obstacles_msg);
}
