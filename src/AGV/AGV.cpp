#include "AGV.h"

AGV *AGV::inst_ = nullptr;
AGV *AGV::getAGV(const string &node_name, const string &agent_name)
{
    if (inst_ == nullptr)
        inst_ = new AGV(node_name, agent_name);
    return inst_;
}

AGV::AGV(const string &node_name, const string &agent_name)
    : Car(node_name, agent_name),
      Kp(1.2),
      Ki(0.5),
      Kd(0.01),
      Koz(1),
      dt(0.01),
      threshold(0.02)
{
    InitialRos();
}

AGV::~AGV()
{
    inst_ = nullptr;
    delete_thread_pos = true;
    thread_sub_pos.join();
}

/* 
Control
*/
void AGV::Move(const float target_x, const float target_y, const float target_oz, const int &target_velocity)
{
    Move(target_x, target_y, target_velocity);
    Rotate(target_oz, target_velocity);
    cout << "Finished" << endl;
}

void AGV::Move(const float target_x, const float target_y, const int &target_velocity)
{   
    float err_x, err_y, err_oz;
    float sum_err_x, sum_err_y, sum_err_oz;
    float diff_err_x, diff_err_y, diff_err_oz;
    sum_err_x = sum_err_y = sum_err_oz = 0;
    diff_err_x = diff_err_y = diff_err_oz = 0;

    err_x = target_x - x;
    err_y = target_y - y;
    float distance = sqrt(pow(err_x, 2) + pow(err_y, 2));
    float target_oz = atan2(err_y, err_x);
    Rotate(target_oz + copysignf(0.015, target_oz), target_velocity);
    Car::Move(target_velocity, -target_velocity, 0.3 * distance);

    do
    {
        diff_err_x = ((target_x - x) - err_x) / dt;
        diff_err_y = ((target_y - y) - err_y) / dt;
        diff_err_oz = ((target_oz - oz) - err_oz) / dt;
        err_x = target_x - x;
        err_y = target_y - y;
        distance = sqrt(pow(err_x, 2) + pow(err_y, 2));
        target_oz = atan2(err_y, err_x);
        err_oz = target_oz - oz;
        if (abs(err_oz) > 3 * M_PI_2)
            err_oz = copysignf(2 * M_PI - abs(err_oz), -err_oz);
        sum_err_x += err_x * dt;
        sum_err_y += err_y * dt;
        sum_err_oz += err_oz * dt;
        
        const float velocity_x = target_velocity * (Kp * err_x + Ki * sum_err_x + Kd * diff_err_x);
        const float velocity_y = target_velocity * (Kp * err_y + Ki * sum_err_y + Kd * diff_err_y);
        const float velocity_oz = (Kp * err_oz + Ki * sum_err_oz + Kd * diff_err_oz);
        int velocity = sqrt(pow(velocity_x, 2) + pow(velocity_y, 2));
        velocity = copysignf(velocity, velocity_x * cos(oz) * velocity_y * sin(oz));

        cout << "Error: " << err_x << "   " << err_y << "   " << err_oz << endl;
        cout << "Velocity: " << velocity << endl;

        if (thread_break)
            break;
        
        Car::MoveDirection(velocity_oz, 0.0f, velocity);
        this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    } while (abs(err_x) > threshold || abs(err_y) > threshold);
    Stop();
}

void AGV::MoveForward(const float distance, const int &velocity)
{
    Move(x + distance * cos(oz), y + distance * sin(oz), oz, abs(velocity));
}

void AGV::MoveBackward(const float distance, const int &velocity)
{
    MoveForward(-distance, velocity);
}

void AGV::Rotate(float target_oz, const int &target_velocity)
{
    float direction = target_oz - oz;
    if (abs(direction) > 3 * M_PI_2)
        direction = copysignf(2 * M_PI - abs(direction), -direction);
    Car::Rotate(direction, target_velocity);

    float err_oz = 0;
    float sum_err_oz = 0;
    float diff_err_oz = 0;

    do
    {
        diff_err_oz = ((target_oz - oz) - err_oz) / dt;
        err_oz = target_oz - oz;
        if (abs(err_oz) > 3 * M_PI_2)
            target_oz = copysignf(2 * M_PI - abs(target_oz), -target_oz);
        sum_err_oz += err_oz * dt;
        int diff_velocity = Koz * abs(target_velocity) * (Kp * err_oz + Ki * sum_err_oz + Kd * diff_err_oz);

        cout << "Oz Error: " << err_oz << endl;
        cout << "Oz Velocity: " << diff_velocity << endl;
        
        if (thread_break)
            break;

        Car::Move(-diff_velocity, -diff_velocity, 0.0f);
        this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    } while (abs(err_oz) > 0.2 * threshold);
    Stop();
}

void AGV::RotateLeft(const float direction, const int &velocity)
{
    this->Rotate(oz + direction, abs(velocity));
}

void AGV::RotateRight(const float direction, const int &velocity)
{
    this->Rotate(oz - direction, -abs(velocity));
}

void AGV::Stop()
{
    thread_break = true;
    Car::Stop();
    this_thread::sleep_for(std::chrono::milliseconds(100));
    thread_break = false;
}

void AGV::RotateConveyor(const float &direction)
{
    Car::RotateConveyor(direction);
}

void AGV::Put(const int &velocity)
{
    Car::Put(velocity);
}



/* 
Ros
*/
void AGV::InitialRos()
{
    thread_sub_pos = thread(&AGV::SubPos, this);
}

void AGV::SubPos()
{
    sub_pos = n.subscribe("/slamware_ros_sdk_server_node/odom", 0, &AGV::PosCallBack, this);
    while (ros::ok && !delete_thread_pos)
    {
        ros::spinOnce();
        this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


void AGV::PosCallBack(const nav_msgs::Odometry &msg)
{
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    oz = yaw;
}

void AGV::CheckData()
{
    Car::CheckData();
}

const int AGV::GetAction()
{
    return Car::GetAction();
}