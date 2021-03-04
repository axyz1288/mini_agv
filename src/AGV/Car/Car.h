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
	void Move(const int &velocity_L, const int &velocity_R, const float &distance = 0.0f);
    /*
	@ Direction (rad)
	@ Distance
	@ Velocity
	*/
	void MoveDirection(const float &direction, const float &distance = 0.0f, const int &velocity = default_velocity);
	/*
	@ Distance
	@ Speed
	*/
	virtual void MoveForward(const float &distance = 0.0f, const int &speed = default_velocity);
	/*
	@ Distance
	@ Speed
	*/
	virtual void MoveBackward(const float &distance = 0.0f, const int &speed = default_velocity);
	/*
	@ Direction (rad)
	@ Speed
	*/
	virtual void Rotate(const float &direction = 0.0f, const int &speed = 100);
	/*
	@ Direction (rad)
	@ Speed
	*/
	virtual void RotateLeft(const float &direction = 0.0f, const int &speed = 100);
	/*
	@ Direction (rad)
	@ Speed
	*/
	virtual void RotateRight(const float &direction = 0.0f, const int &speed = 100);
	/*
	*/
	virtual void Stop();
	/*
	@ Direction (rad)
	*/
	virtual void RotateConveyor(const float &direction);
	/*
	@ Velocity
	*/
	virtual void Put(const int &velocity = default_velocity);



// Ros
private:
	void InitialRos();
	void Sub();
	void ActionCallBack(const std_msgs::Float32MultiArray &msg);
	void RewardCallBack(const std_msgs::Float32MultiArray &msg);
public:
	virtual void CheckData();
	virtual const int GetAction();



// Properties
public:
    static Car *getCar(const string &node_name, const string &env_name, const string &agent_name);
    ~Car();
	static const int default_velocity;
protected:
	Car(const string &node_name, const string &env_name, const string &agent_name);
	static Car *inst_;
	const string node_name;
	const string env_name;
	const string agent_name;
	const int idx;
	const std_msgs::Float32MultiArray info;
	const int num_agent;
	ros::NodeHandle n;
private:
	ros::Subscriber sub_a;
	ros::Subscriber sub_r;
	thread thread_sub;
	bool delete_thread_sub = false;
	vector<vector<float>> action;
	vector<vector<float>> reward;

protected:
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
};