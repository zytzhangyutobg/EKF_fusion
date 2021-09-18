#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "visualization_msgs/Marker.h"

using namespace std;
using namespace Eigen;

ros::Publisher odom_pub;
ros::Publisher ref_pub;
ros::Publisher error_pub;
ros::Publisher path_pub;
nav_msgs::Path path;
ros::Publisher pathr_pub;
nav_msgs::Path pathr;
ros::Publisher pub_vel_imu;
ros::Publisher pub_vel_opt;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
MatrixXd Rt_opt = MatrixXd::Identity(3,3);
MatrixXd x = MatrixXd::Zero(15, 1);
MatrixXd cov = MatrixXd::Identity(15, 15);
MatrixXd x_ = MatrixXd::Zero(15,100);
MatrixXd t = MatrixXd::Zero(1,100);
MatrixXd dt_ = MatrixXd::Zero(1,100);
MatrixXd ang_v_=MatrixXd::Zero(3,100);
MatrixXd lin_a_=MatrixXd::Zero(3,100);
Vector3d velocity_gt, position;
std::vector<ros::Time> t_(100);
std::vector<ros::Time> dtt(100);
//std::vector<sensor_msgs::Imu::> v;
ros::Time cur_t;
ros::Time last_t;
ros::Time rosT;
ros::Time vicon_time;
ros::Time s_time;
bool cam_ready = false;
bool optf_ready = false;
bool first_imu_frame = true;
nav_msgs::Odometry odom_ekf;

Quaterniond q;
float g=-9.8;
int N=0;  
int n=0;  
int N_max=0;
double T;
Eigen::Vector3d error_(0,0,0);
Eigen::Vector3d ave_error(0,0,0);

Eigen::Vector3d R_2_q(const Eigen::Matrix3d& R)
{
	Eigen::Vector3d q;
	double roll = asin(R(2, 1));
	double pitch = atan2(-R(2, 0) / cos(roll), R(2, 2) / cos(roll));
	double yaw = atan2(-R(0, 1) / cos(roll), R(1, 1) / cos(roll));
	q(0) = roll;
	q(1) = pitch;
	q(2) = yaw;

	return q;
}

Eigen::Matrix3d q_2_R(const Eigen::Vector3d& q)
{
	Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3, 3);
	double roll = q(0), pitch = q(1), yaw = q(2);
	R(0, 0) = cos(yaw) * cos(pitch) - sin(roll) * sin(pitch) * sin(yaw);
	R(0, 1) = -cos(roll) * sin(yaw);
	R(0, 2) = cos(yaw) * sin(pitch) + cos(pitch) * sin(roll) * sin(yaw);
	R(1, 0) = cos(pitch) * sin(yaw) + cos(yaw) * sin(roll) * sin(pitch);
	R(1, 1) = cos(roll) * cos(yaw);
	R(1, 2) = sin(yaw) * sin(pitch) - cos(yaw) * cos(pitch) * sin(roll);
	R(2, 0) = -cos(roll) * sin(pitch);
	R(2, 1) = sin(roll);
	R(2, 2) = cos(roll) * cos(pitch);

	return R;
}

void system_pub(ros::Time stamp, const sensor_msgs::Imu::ConstPtr &msg)
{
	Vector3d q_o;
	q_o(0) = x(3, 0);
	q_o(1) = x(4, 0);
	q_o(2) = x(5, 0);
	Matrix3d Ro(3, 3);
	Ro = q_2_R(q_o);
	Quaterniond q_mw(Ro);
	odom_ekf.header.stamp = stamp;
	odom_ekf.header.frame_id = "world";
	odom_ekf.pose.pose.position.x = x(0, 0);
	odom_ekf.pose.pose.position.y = x(1, 0);
	odom_ekf.pose.pose.position.z = x(2, 0);
	odom_ekf.twist.twist.linear.x = x(6, 0);
	odom_ekf.twist.twist.linear.y = x(7, 0);
	odom_ekf.twist.twist.linear.z = x(8, 0);
	odom_ekf.pose.pose.orientation.w = q_mw.w();
	odom_ekf.pose.pose.orientation.x = q_mw.x();
	odom_ekf.pose.pose.orientation.y = q_mw.y();
	odom_ekf.pose.pose.orientation.z = q_mw.z();
	//cout << "p_ekf" << endl << x(0, 0) << ', ' << x(1, 0) << ', ' << x(2, 0) << endl;
	odom_pub.publish(odom_ekf);
	
	path.header.stamp=msg->header.stamp;
    path.header.frame_id="world";
	geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = x(0, 0);
    this_pose_stamped.pose.position.y = x(1, 0);
	this_pose_stamped.pose.position.z = x(2, 0);
	this_pose_stamped.pose.orientation.x = q_mw.x();
	this_pose_stamped.pose.orientation.y = q_mw.y();
	this_pose_stamped.pose.orientation.z = q_mw.z();
	this_pose_stamped.pose.orientation.w = q_mw.w();
	this_pose_stamped.header.stamp=msg->header.stamp;
	this_pose_stamped.header.frame_id="world";
	path.poses.push_back(this_pose_stamped);
	path_pub.publish(path);
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
  vicon_time = vicon_msg->header.stamp;
}

void sys_pub(ros::Time stamp)
{
	Vector3d q_o;
	q_o(0) = x(3, 0);
	q_o(1) = x(4, 0);
	q_o(2) = x(5, 0);
	Matrix3d Ro(3, 3);
	Ro = q_2_R(q_o);
	Quaterniond q_mw(Ro);
	odom_ekf.header.stamp = stamp;
	odom_ekf.header.frame_id = "world";
	odom_ekf.pose.pose.position.x = x(0, 0);
	odom_ekf.pose.pose.position.y = x(1, 0);
	odom_ekf.pose.pose.position.z = x(2, 0);
	odom_ekf.twist.twist.linear.x = x(6, 0);
	odom_ekf.twist.twist.linear.y = x(7, 0);
	odom_ekf.twist.twist.linear.z = x(8, 0);
	odom_ekf.pose.pose.orientation.w = q_mw.w();
	odom_ekf.pose.pose.orientation.x = q_mw.x();
	odom_ekf.pose.pose.orientation.y = q_mw.y();
	odom_ekf.pose.pose.orientation.z = q_mw.z();
	//cout << "p_ekf" << endl << x(0, 0) << ', ' << x(1, 0) << ', ' << x(2, 0) << endl;
	odom_pub.publish(odom_ekf);
	
	nav_msgs::Odometry error;
	error_(0) += abs(position.x()-x(0, 0));
	error_(1) += abs(position.y()-x(1, 0));
	error_(2) += abs(position.z()-x(2, 0));
	double delta_t = odom_ekf.header.stamp.toSec() - s_time.toSec();
	ave_error(0) = error_(0) / delta_t;
	ave_error(1) = error_(1) / delta_t;
	ave_error(2) = error_(2) / delta_t;
	error.header.stamp = stamp;
	error.header.frame_id = "world";
	error.pose.pose.position.x = ave_error(0);
	error.pose.pose.position.y = ave_error(1);
	error.pose.pose.position.z = ave_error(2);
	error_pub.publish(error);

	path.header.stamp=stamp;
    path.header.frame_id="world";
	geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = x(0, 0);
    this_pose_stamped.pose.position.y = x(1, 0);
	this_pose_stamped.pose.position.z = x(2, 0);
	this_pose_stamped.pose.orientation.x = q_mw.x();
	this_pose_stamped.pose.orientation.y = q_mw.y();
	this_pose_stamped.pose.orientation.z = q_mw.z();
	this_pose_stamped.pose.orientation.w = q_mw.w();
	this_pose_stamped.header.stamp=stamp;
	this_pose_stamped.header.frame_id="world";
	path.poses.push_back(this_pose_stamped);
	path_pub.publish(path);
}

void visualizeVelocity(Eigen::Vector3d position, Eigen::Vector3d velocity,
                       int id, Eigen::Vector3d color, ros::Publisher pub_vel) {
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

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //t(0,N)=msg->header.stamp.toSec();
    //your code for propagation
	
	Eigen::MatrixXd At = Eigen::MatrixXd::Zero(15, 15);
	Eigen::MatrixXd Ut = Eigen::MatrixXd::Zero(15, 12);
	Eigen::MatrixXd Ft = Eigen::MatrixXd::Zero(15, 15);
	Eigen::MatrixXd Vt = Eigen::MatrixXd::Zero(15, 12);

	if(first_imu_frame)
	{
		first_imu_frame = false;
		cur_t = msg->header.stamp;
		s_time = cur_t;
		//system_pub(msg->header.stamp, msg);
	}
	else
	{
		t(0,N)=msg->header.stamp.toSec();
		double dT = msg->header.stamp.toSec()-cur_t.toSec();
		dt_(0,N)=dT;
		ang_v_(0,N)=msg->angular_velocity.x;
		ang_v_(1,N)=msg->angular_velocity.y;
		ang_v_(2,N)=msg->angular_velocity.z;
		lin_a_(0,N)=msg->linear_acceleration.x;
		lin_a_(1,N)=msg->linear_acceleration.y;
		lin_a_(2,N)=msg->linear_acceleration.z;
		cur_t = msg->header.stamp;
		t_.push_back(cur_t);
		//dtt.push_back(msg->header.stamp-cur_t);
		N++;
		//system_pub(msg->header.stamp, msg);
		cout<<"imu pro"<<endl;
		//visualizeVelocity(Vector3d(x(0,0),x(1,0),x(2,0)), Vector3d(x(6,0),x(7,0),x(8,0)), 0, Eigen::Vector3d(0, 0, 1), pub_vel_imu);
	}

	if(cam_ready)
	{
		double dT=dt_(0,n);
	    //if(dT>1||!cam_ready)
	    //	dT = 0;
		//cout<<"dT: "<<dT<<endl;
		MatrixXd At = MatrixXd::Zero(15,15);
		MatrixXd Ut = MatrixXd::Zero(15,12);
		MatrixXd Ft = MatrixXd::Zero(15,15);
		MatrixXd Vt = MatrixXd::Zero(15,12);
		MatrixXd f = MatrixXd::Zero(15,1);
		Vector3d ang_v(ang_v_(0,n), ang_v_(1,n), ang_v_(2,n));
		Vector3d lin_a(lin_a_(0,n), lin_a_(1,n), lin_a_(2,n));
		Vector3d x4(x(9,0), x(10,0), x(11,0));
		Vector3d x5(x(12,0), x(13,0), x(14,0));
	    Matrix3d G_inv_dot, R_dot, G, R;
	    Vector3d g(0, 0, -9.8);
	    

		At.block<3,3>(0,6) = Matrix3d::Identity(3,3);
		
	    G_inv_dot<<
			0, ang_v(2)*cos(x(4,0)) - ang_v(0)*sin(x(4,0)), 0,
	    	ang_v(0)*sin(x(4,0)) - ang_v(2)*cos(x(4,0)) - (ang_v(2)*cos(x(4,0))*sin(x(3,0))*sin(x(3,0)))/(cos(x(3,0))*cos(x(3,0))) + (ang_v(0)*sin(x(3,0))*sin(x(3,0))*sin(x(4,0)))/(cos(x(3,0))*cos(x(3,0))), (ang_v(0)*cos(x(4,0))*sin(x(3,0)))/cos(x(3,0)) + (ang_v(2)*sin(x(3,0))*sin(x(4,0)))/cos(x(3,0)), 0,
	    	(ang_v(2)*cos(x(4,0))*sin(x(3,0)))/(cos(x(3,0))*cos(x(3,0))) - (ang_v(0)*sin(x(3,0))*sin(x(4,0)))/(cos(x(3,0))*cos(x(3,0))), -(ang_v(0)*cos(x(4,0)))/cos(x(3,0)) - (ang_v(2)*sin(x(4,0)))/cos(x(3,0)), 0;
	    At.block<3,3>(3,3) = G_inv_dot;
	    
	    R_dot<<
	        sin(x(5,0))*((lin_a(1,0)-x(13,0))*sin(x(3,0)) + (lin_a(2,0)-x(14,0))*cos(x(3,0))*cos(x(4,0)) - (lin_a(0,0)-x(12,0))*cos(x(3,0))*sin(x(4,0))), (lin_a(2,0)-x(14,0))*(cos(x(5,0))*cos(x(4,0)) - sin(x(3,0))*sin(x(5,0))*sin(x(4,0))) - (lin_a(0,0)-x(12,0))*(cos(x(5,0))*sin(x(4,0)) + cos(x(4,0))*sin(x(3,0))*sin(x(5,0))),  -(lin_a(0,0)-x(12,0))*(cos(x(4,0))*sin(x(5,0)) + cos(x(5,0))*sin(x(3,0))*sin(x(4,0))) - (lin_a(2,0)-x(14,0))*(sin(x(5,0))*sin(x(4,0)) - cos(x(5,0))*cos(x(4,0))*sin(x(3,0))) - (lin_a(1,0)-x(13,0))*cos(x(3,0))*cos(x(5,0)),
	        -cos(x(5,0))*((lin_a(1,0)-x(13,0))*sin(x(3,0)) + (lin_a(2,0)-x(14,0))*cos(x(3,0))*cos(x(4,0)) - (lin_a(0,0)-x(12,0))*cos(x(3,0))*sin(x(4,0))), (lin_a(2,0)-x(14,0))*(cos(x(4,0))*sin(x(5,0)) + cos(x(5,0))*sin(x(3,0))*sin(x(4,0))) - (lin_a(0,0)-x(12,0))*(sin(x(5,0))*sin(x(4,0)) - cos(x(5,0))*cos(x(4,0))*sin(x(3,0))),   (lin_a(0,0)-x(12,0))*(cos(x(5,0))*cos(x(4,0)) - sin(x(3,0))*sin(x(5,0))*sin(x(4,0))) + (lin_a(2,0)-x(14,0))*(cos(x(5,0))*sin(x(4,0)) + cos(x(4,0))*sin(x(3,0))*sin(x(5,0))) - (lin_a(1,0)-x(13,0))*cos(x(3,0))*sin(x(5,0)),
	        (lin_a(1,0)-x(13,0))*cos(x(3,0)) - (lin_a(2,0)-x(14,0))*cos(x(4,0))*sin(x(3,0)) + (lin_a(0,0)-x(12,0))*sin(x(3,0))*sin(x(4,0)),           -cos(x(3,0))*((lin_a(0,0)-x(12,0))*cos(x(4,0)) + (lin_a(2,0)-x(14,0))*sin(x(4,0))),                                                                            0;
	    At.block<3,3>(6,3) = R_dot;
		G<<
			cos(x(4,0)), 0, -cos(x(3,0))*sin(x(4,0)),
	        0, 1, sin(x(3,0)),
	        sin(x(4,0)), 0, cos(x(3,0))*cos(x(4,0));
	    At.block<3,3>(3,9) = -G.inverse();
	    
	    R<<
	    	cos(x(4,0))*cos(x(5,0)) - sin(x(3,0))*sin(x(4,0))*sin(x(5,0)), -cos(x(3,0))*sin(x(5,0)), cos(x(5,0))*sin(x(4,0)) + cos(x(4,0))*sin(x(3,0))*sin(x(5,0)),
			cos(x(4,0))*sin(x(5,0)) + cos(x(5,0))*sin(x(3,0))*sin(x(4,0)),  cos(x(3,0))*cos(x(5,0)), sin(x(4,0))*sin(x(5,0)) - cos(x(4,0))*cos(x(5,0))*sin(x(3,0)),
		   -cos(x(3,0))*sin(x(4,0)), sin(x(3,0)), cos(x(3,0))*cos(x(4,0));
		At.block<3,3>(6,12) = -R;

	    Ut.block<3,3>(3,0) = -G.inverse();
	    Ut.block<3,3>(6,3) = -R;
	    Ut.block<6,6>(9,6) = MatrixXd::Identity(6,6);

	    Ft = MatrixXd::Identity(15,15) + dT*At;
	    Vt = dT*Ut;
	    cov = Ft*cov*Ft.transpose()+Vt*Q*Vt.transpose();

		f.block<3,1>(0,0) = x.block<3,1>(6,0);
		f.block<3,1>(3,0) = G.inverse()*(ang_v-x4);
		f.block<3,1>(6,0) = g + R*(lin_a-x5);
		x += f*dT;

		//ros::Time tt=t_[n];
		//T+=dT;
		
		rosT+=msg->header.stamp-cur_t;
		sys_pub(rosT);

		visualizeVelocity(Vector3d(x(0,0),x(1,0),x(2,0)), Vector3d(x(6,0),x(7,0),x(8,0)), 0, Eigen::Vector3d(0, 0, 1), pub_vel_imu);
		n++;
	    if(n>N_max)
	    {
	    	n=0;
	    }
	}
	
}

Eigen::Matrix3d Rcam;
Eigen::Matrix3d Rwt;
//Rotation from the camera frame to the IMU frame //Ric

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{	
	
	T=msg->header.stamp.toSec();
	rosT=msg->header.stamp;
	if(!cam_ready)
	{
		cam_ready = true;
		cout << "CAM READY!" <<endl;
		VectorXd Zt = VectorXd::Zero(6);
		VectorXd zt_g = VectorXd::Zero(6);
		Vector3d Tcw, Twi, Tic, q_wi, Twt;
		Quaterniond q_cw;
		Matrix3d Rwi, Rcw;
		//ros::Time Time_update = msg->header.stamp;
		//relative transformations between frames
		//world in the camera frame
		Tcw(0) = msg->pose.pose.position.x;
		Tcw(1) = msg->pose.pose.position.y;
		Tcw(2) = msg->pose.pose.position.z;
		q_cw.w() = msg->pose.pose.orientation.w;
		q_cw.x() = msg->pose.pose.orientation.x;
		q_cw.y() = msg->pose.pose.orientation.y;
		q_cw.z() = msg->pose.pose.orientation.z;
		Rcw = q_cw.toRotationMatrix();
		//relative transformation between camera and IMU
		Tic << -0.1, 0, -0.03;
		//Ric = Rcam
		//from IMU frame to world frame
		Rwi = Rcw.transpose() * Rcam.transpose(); //Rwi = Rwc * Rci
		Twi = -Rcw.transpose() * (Rcam.transpose() * Tic + Tcw);
		Rwt << 0,1,0,
               1,0,0,
               0,0,-1;
		Twt << -0.55, -0.3, 0;
		Twi = -Rwi.transpose() * (Rwt.transpose() * Twt + Twi);
		//Twi(0) = -Twi(0);
		Twi(2) = -Twi(2);
		Rwi = Rwt * Rwi;
		q_wi = R_2_q(Rwi);
		Zt << Twi, q_wi;

		x(0, 0) = Twi(0);
		x(1, 0) = Twi(1);
		x(2, 0) = Twi(2);
		x(3, 0) = q_wi(0);
		x(4, 0) = q_wi(1);
		x(5, 0) = q_wi(2);
	
	}
	else
	{
		double t0=msg->header.stamp.toSec();
		double dt;
		int min_n=0;
		double x_min=100;
		for(int i=0;i<N;i++)
		{
			dt=fabs(t(0,i)-t0);
			if(dt<x_min)
			{
				min_n=i;
				x_min=dt;
			}
		}
		n=min_n;

		///get Linear observation model Zt:
		VectorXd Zt = VectorXd::Zero(6);
		VectorXd zt_g = VectorXd::Zero(6);
		Vector3d Tcw, Twi, Tic, q_wi, Twt;
		Quaterniond q_cw;
		Matrix3d Rwi, Rcw;
		//ros::Time Time_update = msg->header.stamp;
		//relative transformations between frames
		//world in the camera frame
		Tcw(0) = msg->pose.pose.position.x;
		Tcw(1) = msg->pose.pose.position.y;
		Tcw(2) = msg->pose.pose.position.z;
		q_cw.w() = msg->pose.pose.orientation.w;
		q_cw.x() = msg->pose.pose.orientation.x;
		q_cw.y() = msg->pose.pose.orientation.y;
		q_cw.z() = msg->pose.pose.orientation.z;
		Rcw = q_cw.toRotationMatrix();
		//relative transformation between camera and IMU
		Tic << -0.1, 0, -0.03;
		//Ric = Rcam
		//from IMU frame to world frame
		Rwi = Rcw.transpose() * Rcam.transpose(); //Rwi = Rwc * Rci
		Twi = -Rcw.transpose() * (Rcam.transpose() * Tic + Tcw);
		Rwt << 0,1,0,
               1,0,0,
               0,0,-1;
		Twt << -0.55, -0.3, 0;
		Twi = -Rwi.transpose() * (Rwt.transpose() * Twt + Twi);
		//Twi(0) = -Twi(0);
		Twi(2) = -Twi(2);
		Rwi = Rwt * Rwi;
		q_wi = R_2_q(Rwi);
		Zt << Twi, q_wi;

		nav_msgs::Odometry cam_odom_ekf;
		Eigen::Quaterniond Q_ref(Rwi);
		cam_odom_ekf.header.stamp = msg->header.stamp;
		cam_odom_ekf.header.frame_id = "world";
		cam_odom_ekf.pose.pose.position.x = Twi(0);
		cam_odom_ekf.pose.pose.position.y = Twi(1);
		cam_odom_ekf.pose.pose.position.z = Twi(2);
		cam_odom_ekf.pose.pose.orientation.w = Q_ref.w();
		cam_odom_ekf.pose.pose.orientation.x = Q_ref.x();
		cam_odom_ekf.pose.pose.orientation.y = Q_ref.y();
		cam_odom_ekf.pose.pose.orientation.z = Q_ref.z();
		
		//Zt - g(mean,0)
		zt_g << Zt(0, 0) - x(0, 0), Zt(1, 0) - x(1, 0), Zt(2, 0) - x(2, 0), Zt(3, 0) - x(3, 0), Zt(4, 0) - x(4, 0), Zt(5, 0) - x(5, 0);
		//Euler angles should be in a range that makes function 'asin' vaild
		for (int i = 3; i <= 5; i++) {
			if (zt_g(i, 0) > M_PI)
				zt_g(i, 0) -= 2 * M_PI;
			else if (zt_g(i, 0) < -M_PI)
				zt_g(i, 0) += 2 * M_PI;
		}
		///Linearization parameters and update
		MatrixXd Ct(6, 15);
		Ct << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			  0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			  0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			  0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			  0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			  0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
		MatrixXd Kt(15, 6);
		MatrixXd Wt = MatrixXd::Identity(6, 6);
		//update the Kt
		Kt = cov * Ct.transpose()*(Ct* cov *Ct.transpose() + Wt * Rt * Wt.transpose()).inverse();
		//update the pose
		x += Kt * zt_g;
		for (int j = 3; j <= 5; j++) {
			if (x(j, 0) > M_PI)
				x(j, 0) -= 2 * M_PI;
			else if (x(j, 0) < -M_PI)
				x(j, 0) += 2 * M_PI;
		}
		cout<<"x"<<x(0,0)<<","<<x(1,0)<<","<<x(2,0)<<endl;
		cout<<"cam"<<Twi(0)<<","<<Twi(1)<<","<<Twi(2)<<endl;
		//update covariance
		cov -= Kt * Ct * cov;
		
		cam_odom_ekf.twist.twist.linear.x = zt_g(3);
	    cam_odom_ekf.twist.twist.linear.y = zt_g(4);
	    cam_odom_ekf.twist.twist.linear.z = zt_g(5);
		
		ref_pub.publish(cam_odom_ekf);
		
		pathr.header.stamp=msg->header.stamp;
		pathr.header.frame_id="world";
		geometry_msgs::PoseStamped this_pose_stampedr;
		this_pose_stampedr.pose.position.x = Twi(0);
		this_pose_stampedr.pose.position.y = Twi(1);
		this_pose_stampedr.pose.position.z = Twi(2);
		this_pose_stampedr.pose.orientation.x = Q_ref.x();
		this_pose_stampedr.pose.orientation.y = Q_ref.y();
		this_pose_stampedr.pose.orientation.z = Q_ref.z();
		this_pose_stampedr.pose.orientation.w = Q_ref.w();
		this_pose_stampedr.header.stamp=msg->header.stamp;
		this_pose_stampedr.header.frame_id="world";
		pathr.poses.push_back(this_pose_stampedr);
		pathr_pub.publish(pathr);
		//cam_ready = true;
	}
	sys_pub(msg->header.stamp);
	cout << "OF tag_update!" << endl;
	//rosT=msg->header.stamp;
    //your code for update
    //For part 1
    // camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, -1, 0,
    //                                       0, 0, -1;

    // For part 2 & 3
    // camera position in the IMU frame = (0.1, 0, 0.03)
    // camera orientaion in the IMU frame = Quaternion(0, 0, 0, 1); w x y z, respectively
    //                     RotationMatrix << -1, 0, 0,
    //                                       0, -1, 0,
    //                                       0, 0, 1;
}

void opti_tf_odom_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    T=msg->header.stamp.toSec();
	rosT=msg->header.stamp;
    if (cam_ready)
		optf_ready = true;
	if(optf_ready){
		double t0=msg->header.stamp.toSec();
		double dt;
		int min_n=0;
		double x_min=100;
		for(int i=0;i<N;i++)
		{
			dt=fabs(t(0,i)-t0);
			if(dt<x_min)
			{
				min_n=i;
				x_min=dt;
			}
		}
		n=min_n;
		Vector3d Tic = Vector3d(-0.1, 0, -0.03);
		
	    Vector3d Twi(x(0,0),x(1,0),x(2,0));
	    Vector3d Vw = Vector3d(x(6,0), x(7,0), x(8,0));

	    double phi =x(3,0);
	    double theta=x(4,0);
	    double psi=x(5,0);
	    double vx=x(6,0);
	    double vy=x(7,0);
	    double vz=x(8,0);

	    Matrix3d Rwi;
		Vector3d q_wi = Vector3d(phi,theta,psi);
	    Rwi = q_2_R(q_wi);
		Vector3d Vc = Rcam.transpose() * Rwi.transpose() * Vw;
		Vector3d Tc = -Rcam.transpose()*Rwi.transpose()*Twi - Rcam.transpose()*Tic;

	    MatrixXd Ct = MatrixXd::Zero(3,15); //z,vx,vy
		/**/
	    //d vx vy dx3
	    Ct.block<2,3>(1,6) <<cos(phi)*sin(psi),                                   -cos(phi)*cos(psi),            -sin(phi),
	                    sin(phi)*sin(psi)*sin(theta) - cos(psi)*cos(theta), - cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta),  cos(phi)*sin(theta);
		// d vx vy dx2
	    Ct.block<2,3>(1,3) <<vy*cos(psi)*sin(phi) - vz*cos(phi) - vx*sin(phi)*sin(psi),                                                                                                                                          0,                                                                       vx*cos(phi)*cos(psi) + vy*cos(phi)*sin(psi),
	                         vx*cos(phi)*sin(psi)*sin(theta) - vy*cos(phi)*cos(psi)*sin(theta) - vz*sin(phi)*sin(theta), vx*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) + vy*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + vz*cos(phi)*cos(theta), vx*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - vy*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta));
	    //z
	    Ct.block<1,3>(0,3) << x(0,0)*cos(phi)*cos(theta)*sin(psi) - x(1,0)*cos(phi)*cos(psi)*cos(theta) - x(2,0)*cos(theta)*sin(phi),   x(0,0)*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + x(1,0)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - x(2,0)*cos(phi)*sin(theta), x(1,0)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - x(0,0)*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi));

	    Ct.block<1,3>(0,0)<< cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),  cos(phi)*cos(theta);

	    MatrixXd Wt = MatrixXd::Identity(3,3);
	    MatrixXd Kt = cov*Ct.transpose()*(Ct*cov*Ct.transpose()+Wt*Rt_opt*Wt.transpose()).inverse();

	    Vector3d zt_g((msg-> point.z-Tc(2)),
					  (msg-> point.x-Vc(0)),
					  (msg-> point.y-Vc(1)));
		cout << "-----------------------"<<endl<<msg-> point.z<<","<<Tc(2)<<endl;
		cout << "-----------------------"<<endl<<msg-> point.x<<","<<Vc(0)<<endl;
		cout << "-----------------------"<<endl<<msg-> point.y<<","<<Vc(1)<<endl;

	    x += Kt * zt_g;
	    cov -= Kt*Ct*cov;

		Vector3d velocity = Vector3d(msg->point.x, msg->point.y, msg->point.z);
	    visualizeVelocity(Vector3d(x(0,0),x(1,0),x(2,0)), velocity, 0, Eigen::Vector3d(1, 0, 0), pub_vel_opt);
		cout << "OF update!" << endl;
	}
	
	sys_pub(msg->header.stamp);
	N_max=N;
	N=0;
	t_.clear();
	dtt.clear();
	

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
	ros::Time::init();
	cur_t = ros::Time::now();
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
	ros::Subscriber s3 = n.subscribe("opti_tf_odom", 1000, opti_tf_odom_callback);
	ros::Subscriber sub_vicon = n.subscribe("/uwb_vicon_odom", 10, viconCallback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
	ref_pub = n.advertise<nav_msgs::Odometry>("cam_odom_ekf", 100);
	error_pub = n.advertise<nav_msgs::Odometry>("error", 100);
	path_pub = n.advertise<nav_msgs::Path>("path",1, true);
	pathr_pub = n.advertise<nav_msgs::Path>("pathr",1, true);
    pub_vel_imu = n.advertise<visualization_msgs::Marker>("ekf_imu_velocity", 1, true);
    pub_vel_opt = n.advertise<visualization_msgs::Marker>("ekf_opt_velocity", 1, true);
    Rcam << 0,-1,0,
            -1,0,0,
            0,0,-1;
    //cout << "R_cam--" << endl << Rcam << endl;
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // You should also tune these parameters
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3) = 0.5 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.5 * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);
	Rt_opt = 0.5 * Rt_opt;
    ros::spin();
}