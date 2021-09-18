#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#define _USE_MATH_DEFINES
#include <math.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace std;
using namespace Eigen;

ros::Publisher odom_pub;
ros::Publisher ref_pub;
ros::Publisher path_pub;
nav_msgs::Path path;
ros::Publisher pathr_pub;
nav_msgs::Path pathr;

MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
MatrixXd x = MatrixXd::Zero(15, 1);
MatrixXd cov = MatrixXd::Identity(15, 15);


ros::Time cur_t;
ros::Time last_t;
bool cam_ready = 0;
bool first_imu_frame = true;
nav_msgs::Odometry odom_ekf;

float g=9.8;

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

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //your code for propagation
	Eigen::MatrixXd At = Eigen::MatrixXd::Zero(15, 15);
	Eigen::MatrixXd Ut = Eigen::MatrixXd::Zero(15, 12);
	Eigen::MatrixXd Ft = Eigen::MatrixXd::Zero(15, 15);
	Eigen::MatrixXd Vt = Eigen::MatrixXd::Zero(15, 12);

	if(cam_ready)
	{
		if(first_imu_frame)
		{
			first_imu_frame = false;
			cur_t = msg->header.stamp;
			system_pub(msg->header.stamp, msg);
		}
		else
		{
			double dT = msg->header.stamp.toSec()-cur_t.toSec();
		    //if(dT>1||!cam_ready)
		    //	dT = 0;
			//cout<<"dT: "<<dT<<endl;
			MatrixXd At = MatrixXd::Zero(15,15);
			MatrixXd Ut = MatrixXd::Zero(15,12);
			MatrixXd Ft = MatrixXd::Zero(15,15);
			MatrixXd Vt = MatrixXd::Zero(15,12);
			MatrixXd f = MatrixXd::Zero(15,1);
			Vector3d ang_v(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
			Vector3d lin_a(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
			Vector3d x4(x(9,0), x(10,0), x(11,0));
			Vector3d x5(x(12,0), x(13,0), x(14,0));
		    Matrix3d G_inv_dot, R_dot, G, R;
		    Vector3d g(0, 0, 9.8);

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

			cur_t = msg->header.stamp;

			system_pub(msg->header.stamp, msg);
		}
	}
}

//Rotation from the camera frame to the IMU frame //Ric
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	if(!cam_ready)
	{
		cam_ready = true;
		VectorXd Zt = VectorXd::Zero(6);
		VectorXd zt_g = VectorXd::Zero(6);
		Vector3d Tcw, Twi, Tic, q_wi;
		Quaterniond q_cw;
		Matrix3d Rwi, Rcw;
		ros::Time Time_update = msg->header.stamp;
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
		Tic << 0.05, 0.05, 0;
		//Ric = Rcam
		//from IMU frame to world frame
		Rwi = Rcw.transpose() * Rcam.transpose(); //Rwi = Rwc * Rci
		Twi = -Rcw.transpose() * (Rcam.transpose() * Tic + Tcw);
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
		///get Linear observation model Zt:
		VectorXd Zt = VectorXd::Zero(6);
		VectorXd zt_g = VectorXd::Zero(6);
		Vector3d Tcw, Twi, Tic, q_wi;
		Quaterniond q_cw;
		Matrix3d Rwi, Rcw;
		ros::Time Time_update = msg->header.stamp;
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
		Tic << 0.05, 0.05, 0;
		//Ric = Rcam
		//from IMU frame to world frame
		Rwi = Rcw.transpose() * Rcam.transpose(); //Rwi = Rwc * Rci
		Twi = -Rcw.transpose() * (Rcam.transpose() * Tic + Tcw);
		q_wi = R_2_q(Rwi);
		Zt << Twi, q_wi;
		// cout << "Zt: " << Zt(0) << " " << Zt(1) << " " << Zt(2) << " " << Zt(3) << " " << Zt(4) << " " << Zt(5) << endl;

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

		Vector3d qwi;
		
		//Zt - g(mean,0)
		zt_g << Zt(0, 0) - x(0, 0), Zt(1, 0) - x(1, 0), Zt(2, 0) - x(2, 0), Zt(3, 0) - x(3, 0), Zt(4, 0) - x(4, 0), Zt(5, 0) - x(5, 0);
		//Euler angles should be in a range that makes function 'asin' vaild
		for (int i = 3; i <= 5; i++) {
			if (zt_g(i, 0) > 6)
				zt_g(i, 0) -= 2 * M_PI;
			else if (zt_g(i, 0) < -6)
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
	}
	//publish topic massage
	
	
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
	ros::Time::init();
	cur_t = ros::Time::now();
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
	ref_pub = n.advertise<nav_msgs::Odometry>("cam_odom_ekf", 100);
	path_pub = n.advertise<nav_msgs::Path>("path",1, true);
	pathr_pub = n.advertise<nav_msgs::Path>("pathr",1, true);
    Rcam = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // You should also tune these parameters
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3) = 0.5 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.5 * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    ros::spin();
}
