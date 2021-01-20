#include "AGV.h"

const int AGV::default_velocity = 150;
AGV *AGV::inst_ = nullptr;
AGV *AGV::getAGV(const string &node_name, const string &agent_name)
{
    if (inst_ == nullptr)
        inst_ = new AGV(node_name, agent_name);
    return inst_;
}

AGV::AGV(const string &node_name, const string &agent_name)
    : MotorUnion({0, 1, 2, 3}, {"Mx106", "Mx106", "Mx106", "Mx106"}),
      wheel_L(0),
      wheel_R(1),
      Conveyor_R(2),
      Conveyor(3),
      wheel_radius(0.075),
      VEL2METER_MS(GetMotor_Scale2RPM(wheel_L) * (1.667 * 1e-5) * 2 * M_PI * wheel_radius),
      ACCEL2METER_MS2(GetMotor_Scale2RPMM(wheel_L) * pow(1.667 * 1e-5, 2) * 2 * M_PI * wheel_radius),
      kAxle_2(0.130),
      kWheelBase_2(0.075),
      max_velocity(210),
      idx(atoi(&node_name[3])),
      agent_name(agent_name)
{
    SetMotor_CenterScale(Conveyor_R, 2080);
    SetAllMotorsAccel(10);
    SetMotor_Operating_Mode(wheel_L, 1);
    SetMotor_Operating_Mode(wheel_R, 1);
    SetMotor_Operating_Mode(Conveyor, 1);
    SetMotor_Velocity(Conveyor_R, 50);
    SetAllMotorsTorqueEnable(true);

    InitialRos();
}



/* 
Control
*/
void AGV::Move(const float target_x, const float target_y, const float target_oz, int target_velocity)
{
    SetMotor_TorqueEnable(wheel_L, true);
    SetMotor_TorqueEnable(wheel_R, true);

    float delta_x, delta_y, delta_oz = 0;
    float sum_delta_x, sum_delta_y, sum_delta_oz = 0;
    float diff_delta_x, diff_delta_y, diff_delta_oz = 0;
    float Kp = 1;
    float Ki = 0.1;
    float Kd = 0.005;
    do
    {
        diff_delta_x = delta_x - (target_x - x);
        diff_delta_y = delta_y - (target_y - y);
        diff_delta_oz = delta_oz - (target_oz - oz);

        delta_x = target_x - x;
        delta_y = target_y - y;
        delta_oz = target_oz - oz;

        sum_delta_x += delta_x;
        sum_delta_y += delta_y;
        sum_delta_oz += delta_oz;

        target_velocity *= (Kp * (delta_x + delta_y) + Ki * (sum_delta_x + sum_delta_y) + Kd * (diff_delta_x + diff_delta_y));
        int diff_velocity = target_velocity * (Kp * delta_oz + Ki * sum_delta_oz + Kd * diff_delta_oz);
        int velocity_L = target_velocity + diff_velocity;
        int velocity_R = target_velocity - diff_velocity;

        // velocity clamp
        if(abs(velocity_L) > max_velocity)
            velocity_L = copysignf(max_velocity, velocity_L);
        if(abs(velocity_R) > max_velocity)
            velocity_R = copysignf(max_velocity, velocity_R);

        SetMotor_Velocity(wheel_L, velocity_L);
        SetMotor_Velocity(wheel_R, velocity_R);
    } while (delta_x < threshold || delta_y < threshold || delta_oz < threshold);
}

void AGV::MoveForward(const float &distance, const int &velocity)
{
    if (oz >= 0 and oz < 1)
        Move(x + distance, y, oz);
    else if (oz >= 1 and oz < 2)
        Move(x, y + distance, oz);
    else if (oz >= 2 and oz < 3)
        Move(x - distance, y, oz);
    else
        Move(x, y - distance, oz);
}

void AGV::MoveBackward(const float &distance, const int &velocity)
{
    if (oz >= 0 and oz < 1)
        Move(x - distance, y, oz);
    else if (oz >= 1 and oz < 2)
        Move(x, y - distance, oz);
    else if (oz >= 2 and oz < 3)
        Move(x + distance, y, oz);
    else
        Move(x, y + distance, oz);
}

void AGV::MoveLeft(const float &direction, const int &velocity)
{
    Move(x, y, oz + direction);
}

void AGV::MoveRight(const float &direction, const int &velocity)
{
    Move(x, y, oz - direction);
}

void AGV::Stop()
{
    SetMotor_Velocity(wheel_L, 0);
    SetMotor_Velocity(wheel_R, 0);
}

void AGV::RotateConveyor(const float &direction, const int &velocity)
{
    if (abs(velocity) > max_velocity)
        SetMotor_Velocity(Conveyor_R, copysignf(max_velocity, velocity));
    SetMotor_Angle(Conveyor_R, direction);
}

void AGV::Put(const int &velocity)
{
    SetMotor_Velocity(Conveyor, velocity);
    this_thread::sleep_for(std::chrono::seconds(3));
    SetMotor_Velocity(Conveyor, 0);
}



/* 
Ros
*/
void AGV::InitialRos()
{
    thread_sub_a = thread(&AGV::SubAction, this);
    thread_sub_pos = thread(&AGV::SubPos, this);
}

void AGV::SubAction()
{
    sub_a = n.subscribe('/' + agent_name + "/action", 0, &AGV::ActionCallBack, this);
    ros::spin();
}

void AGV::SubPos()
{
    sub_pos = n.subscribe("/orb_slam2_rgbd/pose", 0, &AGV::PosCallBack, this);
    ros::spin();
}

void AGV::ActionCallBack(const std_msgs::Float32MultiArray &msg)
{
    action.push_back(msg.data);
}

void AGV::PosCallBack(const geometry_msgs::PoseStamped &msg)
{
    x = msg.pose.position.x;
    y = msg.pose.position.y;
    oz = msg.pose.orientation.z;
}

void AGV::CheckData()
{
    if (action.size()) action.erase(action.begin(), action.begin() + 6);
    while (action.size() < idx+1)
        this_thread::sleep_for(std::chrono::microseconds(100));
}

int AGV::GetAction()
{
    return int(action.at(idx).at(0));
}