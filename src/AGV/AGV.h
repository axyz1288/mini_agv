#pragma once
#include "./MotorUnion/MotorUnion.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>

class AGV: public MotorUnion
{
// Control
public:	
	/*
	@ target_x, 
	@ target_y, 
	@ target_oz
	*/
	void Move(const float target_x, const float target_y, const float target_oz, int target_velocity=default_velocity);
	/*
	@ Distance
	@ Velocity
	*/
	void MoveForward(const float &distance = 0.0f, const int &velocity = default_velocity);
	/*
	@ Distance
	@ Velocity
	*/
	void MoveBackward(const float &distance = 0.0f, const int &velocity = default_velocity);
	/*
	@ Direction (angle)
	@ Velocity
	*/
	void MoveLeft(const float &direction = 0.0f, const int &velocity = 100);
	/*
	@ Direction (angle)
	@ Velocity
	*/
	void MoveRight(const float &direction = 0.0f, const int &velocity = 100);
	/*
	*/
	void Stop();
	/*
	@ Direction (angle)
	@ Velocity
	*/
	void RotateConveyor(const float &direction = 0.0f, const int &velocity = default_velocity);
	/*
	@ Velocity
	*/
	void Put(const int &velocity = default_velocity);



// Ros
private:
	void InitialRos();
	void SubAction();
	void SubPos();
	void ActionCallBack(const std_msgs::Float32MultiArray &msg);
	void PosCallBack(const geometry_msgs::PoseStamped &msg);
public:
	void CheckData();
	int GetAction();



// Properties
public:
    static AGV *getAGV(const string &node_name, const string &agent_name);
    ~AGV() { inst_ = nullptr; };
	static const int default_velocity;
private:
	AGV(const string &node_name, const string &agent_name);
	static AGV *inst_;

	const unsigned char wheel_L;
	const unsigned char wheel_R;
	const unsigned char Conveyor_R;
	const unsigned char Conveyor;
	const float wheel_radius;
	const float VEL2METER_MS;
	const float ACCEL2METER_MS2;
	const float kWheelBase_2;
	const float kAxle_2; // 輪距
	const int max_velocity;

	float x, y, oz = 0; 
	float threshold = 0.1;
	
	vector<vector<float>> action = {};
	const int idx;
	const string agent_name;
	ros::NodeHandle n;
	ros::Subscriber sub_a;
	ros::Subscriber sub_pos;
	thread thread_sub_a;
	thread thread_sub_pos;
};