#pragma once
#include "./MotorUnion/MotorUnion.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <thread>

class Car: public MotorUnion
{
// Control
public:	
	/*
	@ Wheel_L_Velocity, 
	@ Wheel_R_Velocity, 
	@ Distance
	*/
	void Move(int velocity_L, int velocity_R, float distance = 0.0f);
    /*
	@ Distance
	@ Direction (angle)
	@ Velocity
	*/
	void Move(const float &distance = 0.0f, const float &direction = 0.0f, const int &velocity = default_velocity);
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
	void ActionCallBack(const std_msgs::Float32MultiArray &msg);
public:
	void CheckData();
	int GetAction();



// Properties
public:
    static Car *getCar(const string &node_name, const string &agent_name);
    ~Car() { inst_ = nullptr; };
	static const int default_velocity;
private:
	Car(const string &node_name, const string &agent_name);
	static Car *inst_;

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
	
	vector<vector<float>> action = {};
	const int idx;
	const string agent_name;
	ros::NodeHandle n;
	ros::Subscriber sub_a;
	thread thread_sub_a;
};