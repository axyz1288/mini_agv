#pragma once
#include "./Car/Car.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <thread>
#include <slamware_ros_sdk/SetMapUpdateRequest.h>
#include <slamware_ros_sdk/SyncSetStcm.h>
#include <geometry_msgs/Pose.h>
#include <boost/filesystem/fstream.hpp>

class AGV : protected Car
{
// Control
public:
	/*
	@ target_x,
	@ target_y,
	@ target_oz,
	@ speed
	*/
	void Move(const float target_x, const float target_y, const float target_oz, const int &speed = Car::default_velocity);
	/*
	@ target_x,
	@ target_y,
	@ speed
	*/
	void Move(const float target_x, const float target_y, const int &speed = Car::default_velocity);
	/*
	@ target_x,
	@ target_y,
	@ speed
	*/
	void MoveDirection(const float target_x, const float target_y, const int &speed = Car::default_velocity);
	/*
	@ distance
	@ speed
	*/
	virtual void MoveForward(const float distance = 0.0f, const int &speed = Car::default_velocity);
	/*
	@ distance
	@ speed
	*/
	virtual void MoveBackward(const float distance = 0.0f, const int &speed = Car::default_velocity);
	/*
	@ distance
	@ speed
	*/
	void MoveLeft(const float distance = 0.0f, const int &speed = Car::default_velocity);
	/*
	@ distance
	@ speed
	*/
	void MoveRight(const float distance = 0.0f, const int &speed = Car::default_velocity);
	/*
	@ target_oz (rad)
	@ speed
	*/
	virtual void Rotate(float target_oz, const int &speed = Car::default_velocity);
	/*
	@ direction (rad)
	@ speed
	*/
	virtual void RotateLeft(const float direction = 0.0f, const int &speed = 100);
	/*
	@ direction (rad)
	@ speed
	*/
	virtual void RotateRight(const float direction = 0.0f, const int &speed = 100);
	/*
	@ direction (rad)
	@ speed
	*/
	void Selfturn(const float direction = 0.0f, const int &speed = 100);
	/*
	*/
	virtual void Stop();
	/*
	@ direction (rad)
	*/
	virtual void RotateConveyor(const float &direction);
	/*
	@ velocity
	*/
	virtual void Put(const int &velocity = default_velocity);

// Get
	virtual const int GetAction();
	const float GetNextX();
	const float GetNextY();
	const float GetNowX();
	const float GetNowY();

// Ros
private:
	void InitialMap();
	void InitialRos();
	void Sub();
	void PosCallBack(const nav_msgs::Odometry &msg);
	void NowStateCallBack(const std_msgs::Float32MultiArray &msg);
	void NextStateCallBack(const std_msgs::Float32MultiArray &msg);
public:
	virtual void CheckData();
	virtual void ClearData();
	virtual void PubDone();

// Properties
public:
	static AGV *getAGV(const string &node_name, const string &env_name, const string &agent_name);
	~AGV();

private:
	AGV(const string &node_name, const string &env_name, const string &agent_name);
	static AGV *inst_;

	ros::Subscriber sub_pos;
	ros::Subscriber sub_now_state;
	ros::Subscriber sub_next_state;
	thread thread_sub;
	bool delete_thread_sub = false;

	vector<vector<float>> now_state, next_state;

	float x, y, oz;
	float next_x, next_y;
	const float map_w, map_h;
	const float map_unit;
	const float threshold;
	const float Kp;
	const float Ki;
	const float Kd;
	const float Koz;
	const float dt;

	bool move_break = false;
};