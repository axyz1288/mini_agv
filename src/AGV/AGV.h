#pragma once
#include "./Car/Car.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <thread>

class AGV : protected Car
{
// Control
public:
	/*
	@ target_x,
	@ target_y,
	@ target_oz,
	@ target_velocity
	*/
	void Move(const float target_x, const float target_y, const float target_oz, const int &target_velocity = Car::default_velocity);
	/*
	@ target_x,
	@ target_y,
	@ target_velocity
	*/
	void Move(const float target_x, const float target_y, const int &target_velocity = Car::default_velocity);
	/*
	@ Distance
	@ Velocity
	*/
	virtual void MoveForward(const float distance = 0.0f, const int &velocity = Car::default_velocity);
	/*
	@ Distance
	@ Velocity
	*/
	virtual void MoveBackward(const float distance = 0.0f, const int &velocity = Car::default_velocity);
	/*
	@ target_oz (rad)
	@ target_velocity
	*/
	virtual void Rotate(float target_oz, const int &target_velocity = Car::default_velocity);
	/*
	@ Direction (rad)
	@ Velocity
	*/
	virtual void RotateLeft(const float direction = 0.0f, const int &velocity = 100);
	/*
	@ Direction (rad)
	@ Velocity
	*/
	virtual void RotateRight(const float direction = 0.0f, const int &velocity = 100);
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
	void SubPos();
	void PosCallBack(const nav_msgs::Odometry &msg);
public:
	void CheckData();
	const int GetAction();


// Properties
public:
	static AGV *getAGV(const string &node_name, const string &agent_name);
	~AGV();

private:
	AGV(const string &node_name, const string &agent_name);
	static AGV *inst_;

	ros::Subscriber sub_pos;
	thread thread_sub_pos;
	bool delete_thread_pos = false;


	float x, y, oz;
	const float threshold;
	const float Kp;
	const float Ki;
	const float Kd;
	const float Koz;
	const float dt;

	bool thread_break = false;
};