#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include "visualization_msgs/Marker.h"
#include <random>
#include <backward.hpp>
namespace backward
{
backward::SignalHandling sh;
}

using namespace std;
double height;
double vicon_time, height_time, last_vicon_time, image_time;
Eigen::Vector3d velocity, position, last_position, velocity_gt;
Eigen::Quaterniond q;
ros::Publisher pub_vel, pub_vel_gt;
cv::Mat tmp, image;
double fx, fy, cx, cy;
cv::Mat cameraMatrix, distCoeffs;
cv::Size imageSize;

/* ---------- optical flow ---------- */
#define CORNER_FEATURE 1
#define UNIFORM_FEATURE 2
cv::Mat prev_gray;
vector<cv::Point2f> uniform_pts, prev_pts;
bool init = false;
const int feature_type = UNIFORM_FEATURE;

Eigen::Vector3d last_velocity;
double last_height;
double last_image_time, last_height_time;

ros::Publisher gt_pub, lkt_pub;
nav_msgs::Path gt_path, lkt_path;
Eigen::Vector3d lkt_pos;

ros::Publisher opti_tf_pub_;

Eigen::Vector2f calcTranslationRANSAC(const vector<pair<cv::Point2f, cv::Point2f>>& pairs,
                                      vector<pair<cv::Point2f, cv::Point2f>>& inliers);
Eigen::Vector2f opticalFlowUniform(cv::Mat image)
{
  /* ========== optical flow ========== */
  cv::Mat gray, draw;
  image.copyTo(gray);
  cv::cvtColor(image, draw, cv::COLOR_GRAY2BGR);
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
  cv::Size sub_pix_win_size(10, 10), win_size(21, 21);
  Eigen::Vector2f translation;
  translation.setZero();

  /* ---------- initialize ---------- */
  if (!init)
  {
    srand(ros::Time::now().toSec());
    /*  init with uniform feature */
    const float delta = 50.0f;
    for (float x = delta; x < imageSize.width; x += delta)
      for (float y = delta; y < imageSize.height; y += delta)
      {
        uniform_pts.push_back(cv::Point2f(x, y));
      }
    init = true;
  }
  else
  {
    /* ---------- opencv calc optical flow ---------- */
    vector<uchar> status;
    vector<float> err;
    vector<cv::Point2f> next_pts;
    vector<pair<cv::Point2f, cv::Point2f>> pairs, inliers;

    cv::calcOpticalFlowPyrLK(prev_gray, gray, uniform_pts, next_pts, status, err, cv::Size(21, 21), 3);

    for (int i = 0; i < next_pts.size(); i++)
    {
      if (status[i] == 0)
        continue;
      pairs.push_back(make_pair(uniform_pts[i], next_pts[i]));
    }

    /* ---------- RANSAC ---------- */
    translation = calcTranslationRANSAC(pairs, inliers);

    /* ---------- draw result ---------- */
    // for (int i = 0; i < next_pts.size(); i++)
    // {
    //   cv::circle(draw, uniform_pts[i], 2, cv::Scalar(0, 255, 0), -1, 8);
    //   // if (status[i] == 0) continue;
    //   // cv::circle(draw, next_pts[i], 2, cv::Scalar(0, 0, 255), -1, 8);
    //   // cv::line(draw, next_pts[i], uniform_pts[i], cv::Scalar(0, 0, 255), 2, 8);
    // }

    // for (int i = 0; i < inliers.size(); i++)
    // {
    //   cv::circle(draw, inliers[i].second, 2, cv::Scalar(0, 0, 255), -1, 8);
    //   cv::line(draw, inliers[i].first, inliers[i].second, cv::Scalar(0, 0, 255), 2, 8);
    // }

    // ROS_INFO_STREAM("inliers size: " << inliers.size());
  }

  // std::swap(prev_gray, gray);
  prev_gray = gray;
  // cv::imshow("LK", draw);

  return translation;
}

Eigen::Vector2f calcTranslationRANSAC(const vector<pair<cv::Point2f, cv::Point2f>>& pairs,
                                      vector<pair<cv::Point2f, cv::Point2f>>& inliers)
{
  const int iter_num = 20, sample_num = 1, total_num = pairs.size();

  const double thresh = 0.1;

  vector<pair<cv::Point2f, cv::Point2f>> samples(3), temp_inliers, best_inliers;

  for (int loop = 0; loop < iter_num; ++loop)
  {
    /* ---------- sample data ---------- */
    for (int i = 0; i < sample_num; i++)
    {
      int idx = rand() % total_num;
      samples[i] = pairs[idx];

      // cout << "sample:\n"
      //      << samples[i].first.x << ", " << samples[i].first.y << "\n"
      //      << samples[i].second.x << ", " << samples[i].second.y << "\n";
    }

    /* ---------- minimum least square error translation (avg in this case) ---------- */
    Eigen::Vector2f t_2d(0, 0);
    for (int i = 0; i < sample_num; i++)
    {
      float dx = samples[i].second.x - samples[i].first.x;
      float dy = samples[i].second.y - samples[i].first.y;
      t_2d(0) += dx;
      t_2d(1) += dy;
    }
    t_2d /= double(sample_num);
    // cout << "t: " << t_2d.transpose() << endl;

    /* ---------- find inliers from all data ---------- */
    temp_inliers.clear();
    for (int i = 0; i < total_num; i++)
    {
      double dx = pairs[i].second.x - pairs[i].first.x;
      double dy = pairs[i].second.y - pairs[i].first.y;

      double error = pow(t_2d(0) - dx, 2) + pow(t_2d(1) - dy, 2);

      // cout << "error:" << error << ", dx dy: " << dx << ", " << dy << endl;

      if (error < thresh)
        temp_inliers.push_back(pairs[i]);
    }

    /* ---------- update best model ---------- */
    if (temp_inliers.size() > best_inliers.size())
    {
      std::swap(best_inliers, temp_inliers);
    }
  }

  /* ---------- fit the best inliers ---------- */
  Eigen::Vector2f t_2d(0, 0);
  for (int i = 0; i < best_inliers.size(); i++)
  {
    float dx = best_inliers[i].second.x - best_inliers[i].first.x;
    float dy = best_inliers[i].second.y - best_inliers[i].first.y;
    t_2d(0) += dx;
    t_2d(1) += dy;
  }
  t_2d /= double(best_inliers.size());

  // cout << "inlier num: " << best_inliers.size() << endl;
  // cout << "translation pixel: " << t_2d.transpose() << endl;

  std::swap(inliers, best_inliers);

  return t_2d;
}

/* ---------- --- ---------- */

void visualizeVelocity(Eigen::Vector3d position, Eigen::Vector3d velocity, int id, Eigen::Vector3d color,
                       ros::Publisher pub_vel)
{
  double scale = 10;
  visualization_msgs::Marker m;
  m.header.frame_id = "world";
  m.id = id;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::MODIFY;
  m.scale.x = 0.2;
  m.scale.y = 0.5;
  m.scale.z = 0;
  m.pose.position.x = 0;
  m.pose.position.y = 0;
  m.pose.position.z = 0;
  m.pose.orientation.w = 1;
  m.pose.orientation.x = 0;
  m.pose.orientation.y = 0;
  m.pose.orientation.z = 0;
  m.color.a = 1.0;
  m.color.r = color.x();
  m.color.g = color.y();
  m.color.b = color.z();
  m.points.clear();
  geometry_msgs::Point point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();
  m.points.push_back(point);
  point.x = position.x() + velocity.x() * scale;
  point.y = position.y() + velocity.y() * scale;
  point.z = position.z() + velocity.z() * scale;
  m.points.push_back(point);
  pub_vel.publish(m);
}

void heightCallback(const sensor_msgs::Range::ConstPtr& height_msg)
{
  height = height_msg->range;
  height_time = height_msg->header.stamp.toSec();
}

void viconCallback(const nav_msgs::Odometry::ConstPtr& vicon_msg)
{
  position.x() = vicon_msg->pose.pose.position.x;
  position.y() = vicon_msg->pose.pose.position.y;
  position.z() = vicon_msg->pose.pose.position.z;

  velocity_gt(0) = vicon_msg->twist.twist.linear.x;
  velocity_gt(1) = vicon_msg->twist.twist.linear.y;
  velocity_gt(2) = vicon_msg->twist.twist.linear.z;

  q = Eigen::Quaterniond(vicon_msg->pose.pose.orientation.w, vicon_msg->pose.pose.orientation.x,
                         vicon_msg->pose.pose.orientation.y, vicon_msg->pose.pose.orientation.z);
  vicon_time = vicon_msg->header.stamp.toSec();

  if (!init)
  {
    lkt_pos = position;
  }
}

void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{


  // cout << "optical flow stamp: " << image_msg->header.stamp << endl;

  image_time = image_msg->header.stamp.toSec();
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
  // cv_ptr->image.copyTo(tmp);

  cv::undistort(cv_ptr->image, image, cameraMatrix, distCoeffs);


  // cv::equalizeHist(tmp1, image);

  // cv::imshow("optical_flow", image);
  // cv::waitKey(1);

  // TODO: 1. Calculate velocity by LK Optical Flow Algorithm
  // TODO: You can use height as the z value and q(from VICON) as the orientation.
  // TODO: For this part, you can assume the UAV is flying slowly,
  // TODO: which means height changes slowly and q seldom changes.
  velocity = Eigen::Vector3d(0, 0, 0);

  /* ---------- optical flow ---------- */
  Eigen::Vector2f duv;
  duv = opticalFlowUniform(image);



  /* ---------- convert to drone velocity ---------- */
  const double alpha = 0.5;
  Eigen::Vector3d temp_velocity;
  if (init)
  {
    Eigen::Vector3d v_camera;
    double dt = (image_time - last_image_time);

    v_camera(0) = -(duv(0) * height) / (fx * dt);
    v_camera(1) = -(duv(1) * height) / (fy * dt);

    temp_velocity(0) = v_camera(1);
    temp_velocity(1) = v_camera(0);

    velocity = (1.0 - alpha) * last_velocity + alpha * temp_velocity;
    lkt_pos += velocity * dt;
  }

  last_velocity = velocity;
  last_image_time = image_time;
  last_height_time = height_time;
  last_height = height;

  /* ---------- publish optical flow and tf ---------- */
  geometry_msgs::PointStamped vel_height;

  vel_height.header.frame_id = "world";
  vel_height.header.stamp = image_msg->header.stamp;

  vel_height.point.x = velocity(0);
  vel_height.point.y = velocity(1);
  vel_height.point.z = height;

  opti_tf_pub_.publish(vel_height);

  /* ---------- visualize odom ---------- */
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = position(0);
  pose.pose.position.y = position(1);
  pose.pose.position.z = position(2);
  gt_path.poses.push_back(pose);
  gt_pub.publish(gt_path);

  pose.pose.position.x = lkt_pos(0);
  pose.pose.position.y = lkt_pos(1);
  pose.pose.position.z = lkt_pos(2);
  lkt_path.poses.push_back(pose);
  lkt_pub.publish(lkt_path);

  // // Visualize in RViz
  visualizeVelocity(position, velocity, 0, Eigen::Vector3d(1, 0, 0), pub_vel);
  visualizeVelocity(position, velocity_gt, 0, Eigen::Vector3d(0, 1, 0), pub_vel_gt);

  last_position = position;
  last_vicon_time = vicon_time;

  // TODO: 2. Analyze the RMS Error here
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "opticalflow_node");
  ros::NodeHandle node;

  cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  fx = cameraMatrix.at<double>(0, 0) = 362.565;
  fy = cameraMatrix.at<double>(1, 1) = 363.082;
  cx = cameraMatrix.at<double>(0, 2) = 365.486;
  cy = cameraMatrix.at<double>(1, 2) = 234.889;

  distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
  distCoeffs.at<double>(0, 0) = -0.278765;
  distCoeffs.at<double>(1, 0) = 0.0694761;
  distCoeffs.at<double>(2, 0) = -2.86553e-05;
  distCoeffs.at<double>(3, 0) = 0.000242845;

  imageSize.height = 480;
  imageSize.width = 752;

  ros::Subscriber sub_height = node.subscribe("/tfmini_ros_node/TFmini", 10, heightCallback);
  ros::Subscriber sub_image = node.subscribe("/camera/image_raw", 10, imageCallback);
  ros::Subscriber sub_vicon = node.subscribe("/uwb_vicon_odom", 10, viconCallback);
  pub_vel = node.advertise<visualization_msgs::Marker>("/optical_flow/velocity", 1, true);
  pub_vel_gt = node.advertise<visualization_msgs::Marker>("/optical_flow/velocity_gt", 1, true);

  // opti_tf_pub_ = node.advertise<nav_msgs::Odometry>("/ekf/opti_tf_odom", 10, true);
  opti_tf_pub_ = node.advertise<geometry_msgs::PointStamped>("/ekf/opti_tf_odom", 10, true);

  /* ----------  ---------- */
  last_velocity = Eigen::Vector3d(0, 0, 0);

  gt_pub = node.advertise<nav_msgs::Path>("/gt_path", 10);
  lkt_pub = node.advertise<nav_msgs::Path>("/lkt_path", 10);

  gt_path.header.frame_id = "world";
  lkt_path.header.frame_id = "world";

  ros::spin();
  return 0;
}
