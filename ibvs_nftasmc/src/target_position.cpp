#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>

Eigen::Vector3f quadPosition;
geometry_msgs::Vector3 linear_error;
float pos_x;
float pos_y;
float xp;
float yp;
float xpp;
float ypp;
float pos_z = 0;
float yaw;
float yawRate;
float yawAccel;
float t;
float step = 0.01;
float arg;
float x_traj;
float y_traj;

void calculate_linear_error(float pos_x, float pos_y, float zD, Eigen::Vector3f quadPosition)
{
	linear_error.x = pos_x - quadPosition(0);
	linear_error.y = pos_y - quadPosition(1);
	linear_error.z = -zD - quadPosition(2);

}

void quad_positionCallback(const geometry_msgs::Vector3::ConstPtr& quadPos)
{
	quadPosition(0) = quadPos->x;
	quadPosition(1) = quadPos->y;
	quadPosition(2) = quadPos->z;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tgt_pos");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Publisher tgt_pos_pub = nh.advertise<geometry_msgs::Vector3>("tgt_position",100);
	ros::Publisher tgt_yaw_pub = nh.advertise<std_msgs::Float64>("tgt_yaw",100);
	ros::Publisher tgt_vel_pub = nh.advertise<geometry_msgs::Vector3>("tgt_velocity",100);
	ros::Publisher tgt_yaw_rate_pub = nh.advertise<std_msgs::Float64>("tgt_yaw_rate",100);
	ros::Publisher tgt_accel_pub = nh.advertise<geometry_msgs::Vector3>("tgt_acceleration",100);
	ros::Publisher tgt_yaw_acceleration_pub = nh.advertise<std_msgs::Float64>("tgt_yaw_acceleration",100);
	ros::Publisher linear_error_pub = nh.advertise<geometry_msgs::Vector3>("linear_error",100);

	ros::Subscriber quad_pos = nh.subscribe("quad_position", 100, &quad_positionCallback);

	geometry_msgs::Vector3 tgt_position;
	std_msgs::Float64 tgt_yaw;
	
	geometry_msgs::Vector3 tgt_vel;
	std_msgs::Float64 tgt_yaw_vel;
	std_msgs::Float64 tgt_psi_rate;

	geometry_msgs::Vector3 tgt_accel;
	std_msgs::Float64 tgt_yaw_accel;	
	
	int i = 0;
	int sim_time = 100/step; //Seconds / step

	pos_x = 0;
	pos_y = 0;
	xp = 0;
	yp = 0;
	yaw = 0;
	yawRate = 0;
	tgt_position.x = pos_x;
	tgt_position.y = pos_y;
	tgt_position.z = pos_z;
	tgt_yaw.data = yaw;
	tgt_vel.x = xp;
	tgt_vel.y = yp;
	tgt_vel.z = 0;
	tgt_psi_rate.data = yawRate;
	calculate_linear_error(pos_x, pos_y, 2.5, quadPosition);
	ros::Duration(0.05).sleep();
	tgt_pos_pub.publish(tgt_position);
	tgt_yaw_pub.publish(tgt_yaw);
	tgt_vel_pub.publish(tgt_vel);
	tgt_yaw_rate_pub.publish(tgt_psi_rate);
	linear_error_pub.publish(linear_error);
	ros::Duration(2).sleep();
	tgt_pos_pub.publish(tgt_position);
	tgt_yaw_pub.publish(tgt_yaw);
	tgt_vel_pub.publish(tgt_vel);
	tgt_yaw_rate_pub.publish(tgt_psi_rate);
	linear_error_pub.publish(linear_error);

	ros::Duration(0.1).sleep();
	while(ros::ok())
	{
		t = i*step;
		
		// Circle Trajectory (wide)
		
		xp = 0.6 * cos(0.3*t);
		yp = 0.6 * sin(0.3*t);
		yawRate = 0.3;

		pos_x = pos_x + xp*step;
		pos_y = pos_y + yp*step;
		yaw = 0.3*t;		
		

		/*
		arg = 0.5*t;
		pos_x = cos(arg);
		pos_y = sin(arg);
		
		xp = -0.5*sin(arg);
		yp = 0.5*cos(arg);
		xpp = 0.5*0.5*cos(arg);
		ypp = -0.5*0.5*sin(arg);
		
		yaw = arg;
		yawRate = 0.5;
		yawAccel = 0;
		*/

		/*
		pos_x = -3*cos(0.1*3.141592*t) +3;
		pos_y = 3*sin(0.1*3.141592*t) + 0;
			
		xp = 3*0.1*3.141592*sin(0.005*3.141592*t);
		yp = 3*0.1*3.141592*cos(0.005*3.141592*t);

		yaw = atan(yp/xp);
		yawRate = 0.005*3.141592;
		yawAccel = 0;
		*/

		/*
		pos_x = 0.1*3.141592*t;
		pos_y = 0.1*3.141592*t;
			
		xp = 0.1*3.141592;
		yp = 0.1*3.141592;

		yaw = 0;
		yawRate = 0;
		yawAccel = 0;
		*/
		
		/*
		pos_x = cos(0.2*t);
		pos_y = sin(0.2*t);

		xp = -0.2*sin(0.2*t);
		yp = 0.2*cos(0.2*t);

		yaw = 0;
		yawRate = 0;
		yawAccel = 0;
		*/
		
		// Static Position
		/*
		pos_x = 0;
		pos_y = 0;
			
		xp = 0;
		yp = 0;

		xpp = 0;
		ypp = 0;
			
		yaw = 0;
		yawRate = 0;
		yawAccel = 0;
		*/

		// Assignt parameters to trajectories
		tgt_position.x = pos_x;
		tgt_position.y = pos_y;
		tgt_position.z = pos_z;
		tgt_yaw.data = yaw;

		tgt_vel.x = xp;
		tgt_vel.y = yp;
		tgt_vel.z = 0;
		tgt_psi_rate.data = yawRate;

		tgt_accel.x = xpp;
		tgt_accel.y = ypp;
		tgt_accel.z = 0;
		tgt_yaw_accel.data = 0;

		calculate_linear_error(pos_x, pos_y, 2.5, quadPosition);

		tgt_pos_pub.publish(tgt_position);
		tgt_yaw_pub.publish(tgt_yaw);
		
		tgt_vel_pub.publish(tgt_vel);
		tgt_yaw_rate_pub.publish(tgt_psi_rate);

		tgt_accel_pub.publish(tgt_accel);
		
		linear_error_pub.publish(linear_error);

		i = i+1;

		std::cout << linear_error << std::endl;

		ros::spinOnce();
		loop_rate.sleep();
		
		
	}
	return 0;	
}
