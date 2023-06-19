//
// Created by caochao on 6/5/19.
//

#include "../include/utils/misc_utils.h"
#include <functional>
#include <queue>

namespace misc_utils_ns
{
/// Function for converting a PointType to a geometry_msgs::msg::Point
/// \param pnt A PointType
/// \return A geometry_msgs::msg::Point
geometry_msgs::msg::Point PCL2GeoMsgPnt(const PCLPointType& pnt)
{
  return GeoMsgPoint(pnt.x, pnt.y, pnt.z);
}

/// Function for converting a geometry_msgs::msg::Point to a PointType
/// \param pnt A geometry_msgs::msg::Point
/// \return A PointType
PCLPointType GeoMsgPnt2PCL(const geometry_msgs::msg::Point& pnt)
{
  PCLPointType point_o;
  point_o.x = (float)pnt.x;
  point_o.y = (float)pnt.y;
  point_o.z = (float)pnt.z;
  return point_o;
}

geometry_msgs::msg::Point GeoMsgPoint(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

PCLPointType PCLPoint(float x, float y, float z)
{
  PCLPointType p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

void LeftRotatePoint(PCLPointType& pnt)
{
  float tmp_z = pnt.z;
  pnt.z = pnt.y;
  pnt.y = pnt.x;
  pnt.x = tmp_z;
}

void RightRotatePoint(PCLPointType& pnt)
{
  float tmp_x = pnt.x;
  pnt.x = pnt.y;
  pnt.y = pnt.z;
  pnt.z = tmp_x;
}

void LeftRotatePoint(geometry_msgs::msg::Point& pnt)
{
  double tmp_z = pnt.z;
  pnt.z = pnt.y;
  pnt.y = pnt.x;
  pnt.x = tmp_z;
}

void RightRotatePoint(geometry_msgs::msg::Point& pnt)
{
  double tmp_x = pnt.x;
  pnt.x = pnt.y;
  pnt.y = pnt.z;
  pnt.z = tmp_x;
}

template <class CloudType>
void KeyposeToMap(CloudType& cloud, const nav_msgs::Odometry::ConstPtr& keypose)
{
  float tx = (float)keypose->pose.pose.position.x;
  float ty = (float)keypose->pose.pose.position.y;
  float tz = (float)keypose->pose.pose.position.z;

  tf::Quaternion tf_q(keypose->pose.pose.orientation.x, keypose->pose.pose.orientation.y,
                      keypose->pose.pose.orientation.z, keypose->pose.pose.orientation.w);
  tf::Matrix3x3 tf_m(tf_q);
  double roll, pitch, yaw;
  tf_m.getRPY(roll, pitch, yaw);

  float sin_roll = (float)sin(roll);
  float cos_roll = (float)cos(roll);
  float sin_pitch = (float)sin(pitch);
  float cos_pitch = (float)cos(pitch);
  float sin_yaw = (float)sin(yaw);
  float cos_yaw = (float)cos(yaw);

  for (auto& point : cloud->points)
  {
    // To map_rot frame
    float x1 = point.x;
    float y1 = point.y;
    float z1 = point.z;

    float x2 = x1;
    float y2 = y1 * cos_roll - z1 * sin_roll;
    float z2 = y1 * sin_roll + z1 * cos_roll;

    float x3 = x2 * cos_pitch + z2 * sin_pitch;
    float y3 = y2;
    float z3 = -x2 * sin_pitch + z2 * cos_pitch;

    float x4 = x3 * cos_yaw - y3 * sin_yaw;
    float y4 = x3 * sin_yaw + y3 * cos_yaw;
    float z4 = z3;

    float x5 = x4 + tx;
    float y5 = y4 + ty;
    float z5 = z4 + tz;

    // To map frame
    point.x = z5;
    point.y = x5;
    point.z = y5;
  }
}

/// Function to compute the distance between two geometry_msgs::msg::Point
/// \param pnt1 The first point
/// \param pnt2 The second point
/// \return Distance between the two points
double PointXYDist(const geometry_msgs::msg::Point& pnt1, const geometry_msgs::msg::Point& pnt2)
{
  return sqrt(pow((pnt1.x - pnt2.x), 2) + pow((pnt1.y - pnt2.y), 2));
}

/// Function to compute the distance between two PointType
/// \param pnt1 The first point
/// \param pnt2 The second point
/// \return Distance between the two points
double PointXYDist(const PCLPointType& pnt1, const PCLPointType& pnt2)
{
  return sqrt(pow((pnt1.x - pnt2.x), 2) + pow((pnt1.y - pnt2.y), 2));
}

/// Function to compute the angle between two vectors in the xy-plane, the z component is omitted
/// \param v1 The first (starting) vector
/// \param v2 The second (end) vector
/// \return Angle from v1 to v2, positive if counterclockwise, negative if clockwise. The output is within [-pi, pi]
double VectorXYAngle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  return atan2(v1.x() * v2.y() - v1.y() * v2.x(), v1.x() * v2.x() + v1.y() * v2.y());
}

/// Function to compute the direction (angle) of a geometry_msgs::msg::Point
/// \param pnt Input point
/// \param robot_pos Robot position
/// \return Direction (angle)
double PointAngle(const geometry_msgs::msg::Point& pnt, const geometry_msgs::msg::Point& robot_pos)
{
  return atan2((pnt.y - robot_pos.y), (pnt.x - robot_pos.x));
}

/// Function to compute the direction (angle) of a PointType
/// \param pnt Intput point
/// \param robot_pos Robot position
/// \return Direction (angle)
double PointAngle(const PCLPointType& pnt, const geometry_msgs::msg::Point& robot_pos)
{
  return atan2((pnt.y - robot_pos.y), (pnt.x - robot_pos.x));
}

/// Function to check if three points are collinear in the xy-plane
/// \param p1 First point
/// \param p2 Second point
/// \param p3 Third point
/// \return Collinear
bool CollinearXY(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2, const geometry_msgs::msg::Point& p3,
                 double threshold)
{
  // https://math.stackexchange.com/questions/405966/if-i-have-three-points-is-there-an-easy-way-to-tell-if-they-are-collinear
  double val = (p2.y - p1.y) * (p3.x - p2.x) - (p3.y - p2.y) * (p2.x - p1.x);
  if (std::abs(val) < threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/// Function to calculate the overlap of two angle intervals [s1, e1] and [s2, e2]. s1(2)->e1(2) counter-clockwise and
/// all angles shoud be in [-pi, pi] \param s1 starting angle of the first interval \param e1 end angle of the second
/// interval \param s2 starting angle of the first interval \param e2 end angle of the second interval \return overlap
/// the overlap of the two intervals. > 0 if overlapped, < 0 otherwise.
double AngleOverlap(double s1, double e1, double s2, double e2)
{
  double overlap = 0.0;
  // TODO: normalize angles to [-pi, pi]
  // The first interval crosses the evil branch
  if (e1 < s1)
  {
    // Recursively compute the overlaps
    double sub_overlap1 = AngleOverlap(s1, M_PI, s2, e2);
    double sub_overlap2 = AngleOverlap(-M_PI, e1, s2, e2);
    // If both sub-overlaps are negative (no overlap) or there is only one positive sub-overlap
    if ((sub_overlap1 < 0 && sub_overlap2 < 0) || sub_overlap1 * sub_overlap2 < 0)
    {
      overlap = std::max(sub_overlap1, sub_overlap2);
    }
    else
    {
      overlap = sub_overlap1 + sub_overlap2;
    }
  }
  else if (e2 < s2)
  {
    // Similar to the case above
    double sub_overlap1 = AngleOverlap(s1, e1, s2, M_PI);
    double sub_overlap2 = AngleOverlap(s1, e1, -M_PI, e2);
    if ((sub_overlap1 < 0 && sub_overlap2 < 0) || sub_overlap1 * sub_overlap2 < 0)
    {
      overlap = std::max(sub_overlap1, sub_overlap2);
    }
    else
    {
      overlap = sub_overlap1 + sub_overlap2;
    }
  }
  else
  {
    if (e1 > e2)
    {
      if (s1 > e2)
      {
        // No overlap
        overlap = e2 - s1;
      }
      else if (s1 > s2)
      {
        overlap = e2 - s1;
      }
      else
      {
        overlap = e2 - s2;
      }
    }
    else
    {
      if (s2 > e1)
      {
        // No overlap
        overlap = e1 - s2;
      }
      else if (s2 > s1)
      {
        overlap = e1 - s2;
      }
      else
      {
        overlap = e1 - s1;
      }
    }
  }
  return overlap;
}

double AngleDiff(double source_angle, double target_angle)
{
  double angle_diff = target_angle - source_angle;
  if (angle_diff > M_PI)
  {
    angle_diff -= 2 * M_PI;
  }
  if (angle_diff < -M_PI)
  {
    angle_diff += 2 * M_PI;
  }
  return angle_diff;
}

/// Function to determine if a point is on a line segment.
/// Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr'
/// \param p End point of line segment pr
/// \param q Point to be examined
/// \param r End point of line segment pr
/// \return If q is on pr
bool PointOnLineSeg(const geometry_msgs::msg::Point& p, const geometry_msgs::msg::Point& q, const geometry_msgs::msg::Point& r)
{
  if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
  {
    return true;
  }
  else
  {
    return false;
  }
}

/// Function to find orientation of ordered triplet (p, q, r).
/// \param p The first point
/// \param q The second point
/// \param r The third point
/// \return 0 --> p, q and r are colinear, 1 --> Clockwise, 2 --> Counterclockwise
int ThreePointOrientation(const geometry_msgs::msg::Point& p, const geometry_msgs::msg::Point& q, const geometry_msgs::msg::Point& r)
{
  // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
  // for details of below formula.
  double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

  if (fabs(val) < 0.1)
    return 0;  // colinear

  return (val > 0) ? 1 : 2;  // clock or counterclock wise
}

/// Function to check if two line segments intersect, returns
/// \param p1 end point of 'p1q1'
/// \param q1 end point of 'p1q1'
/// \param p2 end point of 'p2q2'
/// \param q2 end point of 'p2q2'
/// \return true if line segment 'p1q1' and 'p2q2' intersect, false otherwise
bool LineSegIntersect(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& q1, const geometry_msgs::msg::Point& p2,
                      const geometry_msgs::msg::Point& q2)
{
  // Find the four orientations needed for general and
  // special cases
  int o1 = ThreePointOrientation(p1, q1, p2);
  int o2 = ThreePointOrientation(p1, q1, q2);
  int o3 = ThreePointOrientation(p2, q2, p1);
  int o4 = ThreePointOrientation(p2, q2, q1);

  // General case
  if (o1 != o2 && o3 != o4)
    return true;

  // Special Cases
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1
  if (o1 == 0 && PointOnLineSeg(p1, p2, q1))
    return true;

  // p1, q1 and q2 are colinear and q2 lies on segment p1q1
  if (o2 == 0 && PointOnLineSeg(p1, q2, q1))
    return true;

  // p2, q2 and p1 are colinear and p1 lies on segment p2q2
  if (o3 == 0 && PointOnLineSeg(p2, p1, q2))
    return true;

  // p2, q2 and q1 are colinear and q1 lies on segment p2q2
  if (o4 == 0 && PointOnLineSeg(p2, q1, q2))
    return true;

  return false;  // Doesn't fall in any of the above cases
}

/// Similar to LineSegIntersect(), except adds a "tolerance"
/// such that each line segment is extended at both ends by this distance
/// i.e. this will return true more often than LineSegIntersect()
/// \param p1 end point of 'p1q1'
/// \param q1 end point of 'p1q1'
/// \param p2 end point of 'p2q2'
/// \param q2 end point of 'p2q2'
/// \param tolerance distance to be added at both ends of both lines
/// \return true if line segment 'p1q1' and 'p2q2' intersect, false otherwise
bool LineSegIntersectWithTolerance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& q1,
                                   const geometry_msgs::msg::Point& p2, const geometry_msgs::msg::Point& q2,
                                   const double tolerance)
{
  // Make a copy
  geometry_msgs::msg::Point p1_extend = p1;
  geometry_msgs::msg::Point q1_extend = q1;
  geometry_msgs::msg::Point p2_extend = p2;
  geometry_msgs::msg::Point q2_extend = q2;

  // Extend line segment 1
  double dist1 = PointXYDist(p1, q1);
  if (dist1 == 0)
    return false;  // trivial
  double dir1_x = (q1.x - p1.x) / dist1;
  double dir1_y = (q1.y - p1.y) / dist1;
  p1_extend.x -= tolerance * dir1_x;
  p1_extend.y -= tolerance * dir1_y;
  q1_extend.x += tolerance * dir1_x;
  q1_extend.y += tolerance * dir1_y;

  // Extend line segment 2
  double dist2 = PointXYDist(p2, q2);
  if (dist2 == 0)
    return false;  // trivial
  double dir2_x = (q2.x - p2.x) / dist2;
  double dir2_y = (q2.y - p2.y) / dist2;
  p2_extend.x -= tolerance * dir2_x;
  p2_extend.y -= tolerance * dir2_y;
  q2_extend.x += tolerance * dir2_x;
  q2_extend.y += tolerance * dir2_y;

  // Call the standard function with these extended line segments
  return LineSegIntersect(p1_extend, q1_extend, p2_extend, q2_extend);
}

/// Function to check if a point is inside a polygon
/// \param p point
/// \param polygon polygon
/// \return true if the point is inside the polygon
bool PointInPolygon(const geometry_msgs::msg::Point& point, const geometry_msgs::Polygon& polygon)
{
  int polygon_pnt_num = polygon.points.size();
  if (polygon_pnt_num < 3)
    return false;

  geometry_msgs::msg::Point inf_point;
  inf_point.x = std::numeric_limits<float>::max();
  inf_point.y = point.y;
  int count = 0;
  int cur_idx = 0;
  do
  {
    int next_idx = (cur_idx + 1) % polygon_pnt_num;
    // Check if the line segment from 'point' to 'inf_point' intersects
    // with the line segment from 'polygon[cur_idx]' to 'polygon[next_idx]'
    geometry_msgs::msg::Point polygon_cur_pnt =
        GeoMsgPoint(polygon.points[cur_idx].x, polygon.points[cur_idx].y, polygon.points[cur_idx].z);
    geometry_msgs::msg::Point polygon_next_pnt =
        GeoMsgPoint(polygon.points[next_idx].x, polygon.points[next_idx].y, polygon.points[next_idx].z);
    if (LineSegIntersect(polygon_cur_pnt, polygon_next_pnt, point, inf_point))
    {
      // If the point 'point' is colinear with line segment 'cur-next',
      // then check if it lies on segment. If it lies, return true,
      // otherwise false
      if (ThreePointOrientation(polygon_cur_pnt, point, polygon_next_pnt) == 0)
      {
        return PointOnLineSeg(polygon_cur_pnt, point, polygon_next_pnt);
      }
      count++;
    }
    cur_idx = next_idx;
  } while (cur_idx != 0);

  // return true if count is odd, false othterwise
  return count % 2 == 1;
}

/// Function to get distance from a point to a line segment
/// assumes z components is 0!
/// \param p point
/// \param line_segment_start
/// \param line_segment_end
/// \return distance
double LineSegDistance2D(const geometry_msgs::msg::Point& point, const geometry_msgs::msg::Point& line_segment_start,
                         const geometry_msgs::msg::Point& line_segment_end)
{
  // code adapted from http://geomalgorithms.com/a02-_lines.html

  // vector end to start
  double v_x = line_segment_end.x - line_segment_start.x;
  double v_y = line_segment_end.y - line_segment_start.y;

  // vector start to point
  double w_x = point.x - line_segment_start.x;
  double w_y = point.y - line_segment_start.y;

  // if outside one boundary, get point to point distance
  double c1 = v_x * w_x + v_y * w_y;  // dot product

  if (c1 <= 0)
    return PointXYDist(point, line_segment_start);

  // if outside other boundary, get point to point distance
  double c2 = v_x * v_x + v_y * v_y;  // dot product
  if (c2 <= c1)
    return PointXYDist(point, line_segment_end);

  // otherwise project point and get distance (seems inefficient?)
  double b = c1 / c2;
  geometry_msgs::msg::Point point_projected;
  point_projected.x = line_segment_start.x + b * v_x;
  point_projected.y = line_segment_start.y + b * v_y;
  return PointXYDist(point, point_projected);
}

/// Function to get distance from a point to a line segment
/// \param p point
/// \param line_segment_start
/// \param line_segment_end
/// \return distance
double LineSegDistance3D(const geometry_msgs::msg::Point& point, const geometry_msgs::msg::Point& line_segment_start,
                         const geometry_msgs::msg::Point& line_segment_end)
{
  // code adapted from http://geomalgorithms.com/a02-_lines.html

  // vector end to start
  double v_x = line_segment_end.x - line_segment_start.x;
  double v_y = line_segment_end.y - line_segment_start.y;
  double v_z = line_segment_end.z - line_segment_start.z;

  // vector start to point
  double w_x = point.x - line_segment_start.x;
  double w_y = point.y - line_segment_start.y;
  double w_z = point.z - line_segment_start.z;

  // if outside one boundary, get point to point distance
  double c1 = v_x * w_x + v_y * w_y + v_z * w_z;  // dot product

  if (c1 <= 0)
    return PointXYZDist(point, line_segment_start);

  // if outside other boundary, get point to point distance
  double c2 = v_x * v_x + v_y * v_y + v_z * v_z;  // dot product
  if (c2 <= c1)
    return PointXYZDist(point, line_segment_end);

  // otherwise project point and get distance (seems inefficient?)
  double b = c1 / c2;
  geometry_msgs::msg::Point point_projected;
  point_projected.x = line_segment_start.x + b * v_x;
  point_projected.y = line_segment_start.y + b * v_y;
  point_projected.z = line_segment_start.z + b * v_z;
  return PointXYZDist(point, point_projected);
}

/// Function to get distance from a point to the closest point on boundary of a polygon
/// \param p point
/// \param polygon polygon
/// \return distance
double DistancePoint2DToPolygon(const geometry_msgs::msg::Point& point, const geometry_msgs::Polygon& polygon)
{
  int polygon_pnt_num = polygon.points.size();
  if (polygon_pnt_num < 1)
    return 0;
  if (polygon_pnt_num == 1)
  {
    geometry_msgs::msg::Point poly_point = GeoMsgPoint(polygon.points[0].x, polygon.points[0].y, 0);
    return PointXYDist(point, poly_point);
  }

  double distance_return = INFINITY;
  int cur_idx = 0;

  // iterate through points in polygon
  do
  {
    int next_idx = (cur_idx + 1) % polygon_pnt_num;

    // get point to line segment distance
    geometry_msgs::msg::Point polygon_cur_pnt = GeoMsgPoint(polygon.points[cur_idx].x, polygon.points[cur_idx].y, 0);
    geometry_msgs::msg::Point polygon_next_pnt = GeoMsgPoint(polygon.points[next_idx].x, polygon.points[next_idx].y, 0);
    double distance = LineSegDistance2D(point, polygon_cur_pnt, polygon_next_pnt);
    if (distance < distance_return)
    {
      distance_return = distance;
    }

    cur_idx = next_idx;
  } while (cur_idx != 0);

  return distance_return;
}

void LinInterpPoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double resolution,
                     std::vector<Eigen::Vector3d>& interp_points)
{
  interp_points.clear();
  double point_dist = (p1 - p2).norm();
  if (point_dist < resolution)
  {
    interp_points.push_back(p1);
    interp_points.push_back(p2);
    return;
  }
  int point_num = ceil(point_dist / resolution);
  for (int i = 0; i < point_num; i++)
  {
    Eigen::Vector3d interp_point = (p2 - p1) / point_num * i + p1;
    interp_points.push_back(interp_point);
  }
}

double DegreeToRadian(double degree)
{
  return degree / 180.0 * M_PI;
}
double RadianToDegree(double radian)
{
  return radian * 180.0 / M_PI;
}

void ConcatenatePath(nav_msgs::Path& path1, const nav_msgs::Path& path2, int from_ind, int to_ind)
{
  if (path2.poses.empty())
  {
    return;
  }
  if (from_ind == -1 && to_ind == -1)
  {
    from_ind = 0;
    to_ind = path2.poses.size() - 1;
  }
  MY_ASSERT(from_ind >= 0 && from_ind < path2.poses.size());
  MY_ASSERT(to_ind >= 0 && to_ind < path2.poses.size());
  if (from_ind <= to_ind)
  {
    for (int i = from_ind; i <= to_ind; i++)
    {
      path1.poses.push_back(path2.poses[i]);
    }
  }
  else
  {
    for (int i = from_ind; i >= to_ind; i--)
    {
      path1.poses.push_back(path2.poses[i]);
    }
  }
}

void SetDifference(std::vector<int>& v1, std::vector<int>& v2, std::vector<int>& diff)
{
  std::sort(v1.begin(), v1.end());
  std::sort(v2.begin(), v2.end());
  diff.clear();
  std::set_difference(v1.begin(), v1.end(), v2.begin(), v2.end(), std::inserter(diff, diff.begin()));
}

int Marker::id_ = 0;

int signum(int x)
{
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

double mod(double value, double modulus)
{
  return fmod(fmod(value, modulus) + modulus, modulus);
}

double intbound(double s, double ds)
{
  // Find the smallest positive t such that s+t*ds is an integer.
  if (ds < 0)
  {
    return intbound(-s, -ds);
  }
  else
  {
    s = mod(s, 1);
    // problem is now s+t*ds = 1
    return (1 - s) / ds;
  }
}

bool InRange(const Eigen::Vector3i& sub, const Eigen::Vector3i& max_sub, const Eigen::Vector3i& min_sub)
{
  return sub.x() >= min_sub.x() && sub.x() <= max_sub.x() && sub.y() >= min_sub.y() && sub.y() <= max_sub.y() &&
         sub.z() >= min_sub.z() && sub.z() <= max_sub.z();
}
void RayCast(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub, const Eigen::Vector3i& max_sub,
             const Eigen::Vector3i& min_sub, std::vector<Eigen::Vector3i>& output)
{
  output.clear();
  MY_ASSERT(InRange(start_sub, max_sub, min_sub));
  MY_ASSERT(InRange(end_sub, max_sub, min_sub));

  if (start_sub == end_sub)
  {
    output.push_back(start_sub);
    return;
  }
  Eigen::Vector3i diff_sub = end_sub - start_sub;
  double max_dist = diff_sub.squaredNorm();
  int step_x = signum(diff_sub.x());
  int step_y = signum(diff_sub.y());
  int step_z = signum(diff_sub.z());
  double t_max_x = step_x == 0 ? DBL_MAX : intbound(start_sub.x(), diff_sub.x());
  double t_max_y = step_y == 0 ? DBL_MAX : intbound(start_sub.y(), diff_sub.y());
  double t_max_z = step_z == 0 ? DBL_MAX : intbound(start_sub.z(), diff_sub.z());
  double t_delta_x = step_x == 0 ? DBL_MAX : (double)step_x / (double)diff_sub.x();
  double t_delta_y = step_y == 0 ? DBL_MAX : (double)step_y / (double)diff_sub.y();
  double t_delta_z = step_z == 0 ? DBL_MAX : (double)step_z / (double)diff_sub.z();
  double dist = 0;
  Eigen::Vector3i cur_sub = start_sub;

  while (true)
  {
    MY_ASSERT(InRange(cur_sub, max_sub, min_sub));
    output.push_back(cur_sub);
    dist = (cur_sub - start_sub).squaredNorm();
    if (cur_sub == end_sub || dist > max_dist)
    {
      return;
    }
    if (t_max_x < t_max_y)
    {
      if (t_max_x < t_max_z)
      {
        cur_sub.x() += step_x;
        t_max_x += t_delta_x;
      }
      else
      {
        cur_sub.z() += step_z;
        t_max_z += t_delta_z;
      }
    }
    else
    {
      if (t_max_y < t_max_z)
      {
        cur_sub.y() += step_y;
        t_max_y += t_delta_y;
      }
      else
      {
        cur_sub.z() += step_z;
        t_max_z += t_delta_z;
      }
    }
  }
}

bool InFOV(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position, double vertical_half_fov,
           double range)
{
  Eigen::Vector3d diff = point_position - viewpoint_position;
  double dist = diff.norm();
  if (dist > range)
  {
    return false;
  }
  double theta = acos(diff.z() / dist);
  double vertical_fov_max = M_PI / 2 + vertical_half_fov;
  double vertical_fov_min = M_PI / 2 - vertical_half_fov;
  return theta >= vertical_fov_min && theta <= vertical_fov_max;
}

bool InFOVSimple(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position,
                 double vertical_fov_ratio, double range, double xy_dist_threshold, double z_diff_threshold, bool print)
{
  Eigen::Vector3d diff = point_position - viewpoint_position;
  double xy_dist = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
  if (xy_dist < xy_dist_threshold && std::abs(diff.z()) < z_diff_threshold)
  {
    if (print)
    {
      std::cout << "nearby point approximation" << std::endl;
    }
    return true;
  }
  if (xy_dist > range)
  {
    if (print)
    {
      std::cout << "xy_dist: " << xy_dist << " > range: " << range << std::endl;
    }
    return false;
  }
  if (std::abs(diff.z()) > vertical_fov_ratio * xy_dist)
  {
    if (print)
    {
      std::cout << "diff z: " << std::abs(diff.z()) << " > threshold : " << vertical_fov_ratio << " * " << xy_dist
                << " = " << vertical_fov_ratio * xy_dist << std::endl;
    }
    return false;
  }
  return true;
}

float ApproxAtan(float z)
{
  const float n1 = 0.97239411f;
  const float n2 = -0.19194795f;
  return (n1 + n2 * z * z) * z;
}

float ApproxAtan2(float y, float x)
{
  float ay = std::abs(y), ax = std::abs(x);
  int invert = ay > ax;
  float z = invert ? ax / ay : ay / ax;  // [0,1]
  float th = ApproxAtan(z);              // [0,π/4]
  if (invert)
    th = M_PI_2 - th;  // [0,π/2]
  if (x < 0)
    th = M_PI - th;      // [0,π]
  th = copysign(th, y);  // [-π,π]
  return th;
}

double GetPathLength(const nav_msgs::Path& path)
{
  double path_length = 0.0;
  if (path.poses.size() < 2)
  {
    return path_length;
  }
  for (int i = 0; i < path.poses.size() - 1; i++)
  {
    int cur_pose_idx = i;
    int next_pose_idx = i + 1;
    path_length += misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
        path.poses[cur_pose_idx].pose.position, path.poses[next_pose_idx].pose.position);
  }
  return path_length;
}

double GetPathLength(const std::vector<Eigen::Vector3d>& path)
{
  double path_length = 0.0;
  if (path.size() < 2)
  {
    return path_length;
  }
  for (int i = 0; i < path.size() - 1; i++)
  {
    int cur_idx = i;
    int next_idx = i + 1;
    path_length += (path[cur_idx] - path[next_idx]).norm();
  }
  return path_length;
}

double AStarSearch(const std::vector<std::vector<int>>& graph, const std::vector<std::vector<double>>& node_dist,
                   const std::vector<geometry_msgs::msg::Point>& node_positions, int from_idx, int to_idx, bool get_path,
                   std::vector<int>& path_indices)
{
  MY_ASSERT(graph.size() == node_dist.size());
  MY_ASSERT(graph.size() == node_positions.size());
  MY_ASSERT(misc_utils_ns::InRange<std::vector<int>>(graph, from_idx));
  MY_ASSERT(misc_utils_ns::InRange<std::vector<int>>(graph, to_idx));
  double INF = 9999.0;
  typedef std::pair<double, int> iPair;
  double shortest_dist = 0;
  std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;
  std::vector<double> g(graph.size(), INF);
  std::vector<double> f(graph.size(), INF);
  std::vector<int> prev(graph.size(), -1);
  std::vector<bool> in_pg(graph.size(), false);

  g[from_idx] = 0;
  f[from_idx] = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_positions[from_idx],
                                                                                        node_positions[to_idx]);
  pq.push(std::make_pair(f[from_idx], from_idx));
  in_pg[from_idx] = true;

  while (!pq.empty())
  {
    int u = pq.top().second;
    pq.pop();
    in_pg[u] = false;
    if (u == to_idx)
    {
      shortest_dist = g[u];
      break;
    }

    for (int i = 0; i < graph[u].size(); i++)
    {
      int v = graph[u][i];
      MY_ASSERT(misc_utils_ns::InRange<std::vector<int>>(graph, v));
      double d = node_dist[u][i];
      if (g[v] > g[u] + d)
      {
        prev[v] = u;
        g[v] = g[u] + d;
        f[v] = g[v] + misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_positions[v],
                                                                                              node_positions[to_idx]);
        if (!in_pg[v])
        {
          pq.push(std::make_pair(f[v], v));
          in_pg[v] = true;
        }
      }
    }
  }

  if (get_path)
  {
    path_indices.clear();
    int u = to_idx;
    if (prev[u] != -1 || u == from_idx)
    {
      while (u != -1)
      {
        path_indices.push_back(u);
        u = prev[u];
      }
    }
  }
  std::reverse(path_indices.begin(), path_indices.end());

  return shortest_dist;
}

bool AStarSearchWithMaxPathLength(const std::vector<std::vector<int>>& graph,
                                  const std::vector<std::vector<double>>& node_dist,
                                  const std::vector<geometry_msgs::msg::Point>& node_positions, int from_idx, int to_idx,
                                  bool get_path, std::vector<int>& path_indices, double& shortest_dist,
                                  double max_path_length)
{
  MY_ASSERT(graph.size() == node_dist.size());
  MY_ASSERT(graph.size() == node_positions.size());
  MY_ASSERT(misc_utils_ns::InRange<std::vector<int>>(graph, from_idx));
  MY_ASSERT(misc_utils_ns::InRange<std::vector<int>>(graph, to_idx));
  double INF = 9999.0;
  typedef std::pair<double, int> iPair;
  std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;
  std::vector<double> g(graph.size(), INF);
  std::vector<double> f(graph.size(), INF);
  std::vector<int> prev(graph.size(), -1);
  std::vector<bool> in_pg(graph.size(), false);

  g[from_idx] = 0;
  f[from_idx] = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_positions[from_idx],
                                                                                        node_positions[to_idx]);
  pq.push(std::make_pair(f[from_idx], from_idx));
  in_pg[from_idx] = true;

  bool found_path = false;

  while (!pq.empty())
  {
    int u = pq.top().second;
    pq.pop();
    in_pg[u] = false;
    if (u == to_idx)
    {
      shortest_dist = g[u];
      found_path = true;
      break;
    }
    if (g[u] > max_path_length)
    {
      shortest_dist = g[u];
      found_path = false;
      break;
    }

    for (int i = 0; i < graph[u].size(); i++)
    {
      int v = graph[u][i];
      MY_ASSERT(misc_utils_ns::InRange<std::vector<int>>(graph, v));
      double d = node_dist[u][i];
      if (g[v] > g[u] + d)
      {
        prev[v] = u;
        g[v] = g[u] + d;
        f[v] = g[v] + misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_positions[v],
                                                                                              node_positions[to_idx]);
        if (!in_pg[v])
        {
          pq.push(std::make_pair(f[v], v));
          in_pg[v] = true;
        }
      }
    }
  }

  if (get_path && found_path)
  {
    path_indices.clear();
    int u = to_idx;
    if (prev[u] != -1 || u == from_idx)
    {
      while (u != -1)
      {
        path_indices.push_back(u);
        u = prev[u];
      }
    }
  }
  std::reverse(path_indices.begin(), path_indices.end());

  return found_path;
}

nav_msgs::Path SimplifyPath(const nav_msgs::Path& path)
{
  nav_msgs::Path simplified_path;
  if (path.poses.size() <= 2)
  {
    simplified_path = path;
    return simplified_path;
  }

  simplified_path.poses.push_back(path.poses.front());
  for (int i = 1; i < path.poses.size() - 1; i++)
  {
    geometry_msgs::msg::Point prev_point = path.poses[i - 1].pose.position;
    geometry_msgs::msg::Point cur_point = path.poses[i].pose.position;
    geometry_msgs::msg::Point next_point = path.poses[i + 1].pose.position;
    if (!CollinearXY(prev_point, cur_point, next_point))  // The three points are not colinear in the xy plane
    {
      simplified_path.poses.push_back(path.poses[i]);
    }
  }

  simplified_path.poses.push_back(path.poses.back());

  return simplified_path;
}

nav_msgs::Path DeduplicatePath(const nav_msgs::Path& path, double min_dist)
{
  nav_msgs::Path deduplicated_path;
  if (path.poses.size() <= 2)
  {
    deduplicated_path = path;
    return deduplicated_path;
  }
  deduplicated_path.poses.push_back(path.poses.front());
  geometry_msgs::msg::Point prev_point = path.poses[0].pose.position;
  for (int i = 1; i < path.poses.size() - 1; i++)
  {
    double dist_to_prev = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
        prev_point, path.poses[i].pose.position);
    if (dist_to_prev > min_dist)
    {
      deduplicated_path.poses.push_back(path.poses[i]);
      prev_point = path.poses[i].pose.position;
    }
  }
  double dist_to_prev = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
      prev_point, path.poses.back().pose.position);
  double dist_to_start = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
      path.poses.front().pose.position, path.poses.back().pose.position);
  if (dist_to_prev > min_dist && dist_to_start > min_dist)
  {
    deduplicated_path.poses.push_back(path.poses.back());
  }
  return deduplicated_path;
}

void SampleLineSegments(const std::vector<Eigen::Vector3d>& initial_points, double sample_resol,
                        std::vector<Eigen::Vector3d>& sample_points)

{
  if (initial_points.size() < 2)
  {
    sample_points = initial_points;
    return;
  }
  sample_points.clear();
  int cur_idx = 0;
  int next_idx = 1;
  Eigen::Vector3d cur_point = initial_points[cur_idx];
  Eigen::Vector3d next_point = initial_points[next_idx];
  sample_points.push_back(cur_point);
  double sample_dist = sample_resol;
  while (next_idx < initial_points.size())
  {
    next_point = initial_points[next_idx];
    double dist_to_next = (cur_point - next_point).norm();
    if (dist_to_next > sample_dist)
    {
      Eigen::Vector3d sample_point = (next_point - cur_point).normalized() * sample_dist + cur_point;
      sample_points.push_back(sample_point);
      cur_point = sample_point;
      sample_dist = sample_resol;
    }
    else
    {
      next_idx++;
      cur_point = next_point;
      sample_dist = sample_dist - dist_to_next;
    }
  }
  sample_points.push_back(next_point);
}

void UniquifyIntVector(std::vector<int>& list)
{
  std::sort(list.begin(), list.end());
  std::vector<int>::iterator it;
  it = std::unique(list.begin(), list.end());
  list.resize(std::distance(list.begin(), it));
}

}  // namespace misc_utils_ns

template void misc_utils_ns::KeyposeToMap<pcl::PointCloud<pcl::PointXYZ>::Ptr>(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const nav_msgs::Odometry::ConstPtr& keypose);
template void misc_utils_ns::KeyposeToMap<pcl::PointCloud<pcl::PointXYZI>::Ptr>(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const nav_msgs::Odometry::ConstPtr& keypose);
template void misc_utils_ns::KeyposeToMap<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>(
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud, const nav_msgs::Odometry::ConstPtr& keypose);
template void misc_utils_ns::KeyposeToMap<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, const nav_msgs::Odometry::ConstPtr& keypose);
