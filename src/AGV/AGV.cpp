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
      agent_name(agent_name),
      Kp(1.2),
      Ki(0.5),
      Kd(0.1),
      Koz(0.1),
      dt(0.01)
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
void AGV::Move(const float target_x, const float target_y, const float target_oz, const int &target_velocity)
{
    // do
    // {
    //     Move(target_x, target_y, target_velocity);
    //     Move(target_oz, target_velocity);
    //     if (thread_break)
    //         break;
    // } while (abs(target_x - x) > threshold || abs(target_y - y) > threshold || abs(target_oz - oz));
    Move(target_x, target_y, target_velocity);
    Move(target_oz, target_velocity);
    cout << target_x - x << "   " << target_y - y << "   " << target_oz - oz << endl;
}

void AGV::Move(const float target_x, const float target_y, const int &target_velocity)
{
    float err_x, err_y;
    float sum_err_x, sum_err_y;
    float diff_err_x, diff_err_y;
    err_x = err_y = 0;
    sum_err_x = sum_err_y = 0;
    diff_err_x = diff_err_y = 0;

    do
    {
        diff_err_x = ((target_x - x) - err_x) / dt;
        diff_err_y = ((target_y - y) - err_y) / dt;
        err_x = target_x - x;
        err_y = target_y - y;
        sum_err_x += err_x * dt;
        sum_err_y += err_y * dt;

        const double velocity_x = target_velocity * (Kp * err_x + Ki * sum_err_x + Kd * diff_err_x);
        const double velocity_y = target_velocity * (Kp * err_y + Ki * sum_err_y + Kd * diff_err_y);
        const double velocity_oz = atan2(velocity_y, velocity_x);

        int velocity = abs(velocity_y) / sin(velocity_oz);
        if (sin(velocity_oz) == 0)
            velocity = velocity_x;

        cout << "XY Error: " << err_x << "   " << err_y << endl;
        cout << "XY Velocity: " << velocity << endl;

        // velocity clamp
        if (abs(velocity) > max_velocity)
            velocity = copysignf(0, velocity);
        if (thread_break)
            break;

        SetMotor_Velocity(wheel_L, velocity);
        SetMotor_Velocity(wheel_R, -velocity);
        this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    } while (abs(err_x) > threshold || abs(err_y) > threshold);
    Stop();
}

void AGV::Move(const float target_oz, const int &target_velocity)
{
    float err_oz = 0;
    float sum_err_oz = 0;
    float diff_err_oz = 0;

    do
    {
        diff_err_oz = ((target_oz - oz) - err_oz) / dt;
        err_oz = target_oz - oz;
        sum_err_oz += err_oz * dt;
        int diff_velocity = Koz * target_velocity * (Kp * err_oz + Ki * sum_err_oz + Kd * diff_err_oz);

        cout << "Oz Error: " << err_oz << "   " << target_oz << "   " << oz << endl;
        cout << "Oz Velocity: " << diff_velocity << endl;

        // velocity clamp
        if (abs(diff_velocity) > 0.5 * max_velocity)
            diff_velocity = copysignf(0, diff_velocity);
        if (thread_break)
            break;

        SetMotor_Velocity(wheel_L, -diff_velocity);
        SetMotor_Velocity(wheel_R, -diff_velocity);
        this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    } while (abs(err_oz) > threshold);
    Stop();
}

void AGV::MoveForward(const float &distance, const int &velocity)
{
    if (abs(oz) >= M_PI_4 and abs(oz) < 3 * M_PI_4)
        Move(x, y + distance, oz, abs(velocity));
    else
        Move(x + distance, y, oz, abs(velocity));
}

void AGV::MoveBackward(const float &distance, const int &velocity)
{
    MoveForward(-distance, velocity);
}

void AGV::MoveLeft(const float &direction, const int &velocity)
{
    Move(x, y, oz + direction, abs(velocity));
}

void AGV::MoveRight(const float &direction, const int &velocity)
{
    MoveLeft(-direction, velocity);
}

void AGV::Stop()
{
    thread_break = true;
    SetMotor_Velocity(wheel_L, 0);
    SetMotor_Velocity(wheel_R, 0);
    this_thread::sleep_for(std::chrono::milliseconds(100));
    thread_break = false;
}

void AGV::RotateConveyor(const float &direction)
{
    SetMotor_Angle(Conveyor_R, direction);
}

void AGV::Put(const int &velocity)
{
    SetMotor_Velocity(Conveyor, -velocity);
    this_thread::sleep_for(std::chrono::seconds(3));
    SetMotor_Velocity(Conveyor, 0);
}

/* 
Ros
*/
void AGV::InitialRos()
{
    thread_sub = thread(&AGV::Sub, this);
}

void AGV::Sub()
{
    sub_a = n.subscribe('/' + agent_name + "/action", 0, &AGV::ActionCallBack, this);
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

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    oz = yaw;
}

void AGV::CheckData()
{
    if (action.size())
        action.erase(action.begin(), action.begin() + 6);
    while (action.size() < idx + 1)
        this_thread::sleep_for(std::chrono::microseconds(100));
}

int AGV::GetAction()
{
    return int(action.at(idx).at(0));
}