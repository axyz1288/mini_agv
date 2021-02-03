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
	@ Velocity
	*/
	virtual void MoveForward(const float &distance = 0.0f, const int &velocity = default_velocity);
	/*
	@ Distance
	@ Velocity
	*/
	virtual void MoveBackward(const float &distance = 0.0f, const int &velocity = default_velocity);
	/*
	@ Direction (rad)
	@ Velocity
	*/
	virtual void Rotate(const float &direction = 0.0f, const int &velocity = 100);
	/*
	@ Direction (rad)
	@ Velocity
	*/
	virtual void RotateLeft(const float &direction = 0.0f, const int &velocity = 100);
	/*
	@ Direction (rad)
	@ Velocity
	*/
	virtual void RotateRight(const float &direction = 0.0f, const int &velocity = 100);
	/*
	*/
	virtual void Stop();
	/*
	@ Direction (rad)
	@ Velocity
	*/
	virtual void RotateConveyor(const float &direction);
	/*
	@ Velocity
	*/
	virtual void Put(const int &velocity = default_velocity);



// Ros
private:
	void InitialRos();
	void SubAction();
	void ActionCallBack(const std_msgs::Float32MultiArray &msg);
public:
	void CheckData();
	const int GetAction();



// Properties
public:
    static Car *getCar(const string &node_name, const string &agent_name);
    ~Car();
	static const int default_velocity;
protected:
	Car(const string &node_name, const string &agent_name);
	static Car *inst_;
protected:
	vector<vector<float>> action = {};
	const int idx;
	const string agent_name;
	ros::NodeHandle n;
	ros::Subscriber sub_a;
	thread thread_sub_a;
	bool delete_thread_a = false;

private:
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