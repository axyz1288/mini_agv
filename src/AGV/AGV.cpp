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
      Koz(0.8),
      dt(0.03),
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
    // do
    // {
    //     Move(target_x, target_y, target_velocity);
    //     Move(target_oz, target_velocity);
    //     if (thread_break)
    //         break;
    // } while (abs(target_x - x) > threshold || abs(target_y - y) > threshold || abs(target_oz - oz));
    Move(target_x, target_y, target_velocity);
    Rotate(target_oz, target_velocity);
    cout << target_x - x << "   " << target_y - y << "   " << target_oz - oz << endl;
}

void AGV::Move(const float target_x, const float target_y, const int &target_velocity)
{   
    float err_x, err_y, err_oz;
    float sum_err_x, sum_err_y;
    float diff_err_x, diff_err_y;
    sum_err_x = sum_err_y = 0;
    diff_err_x = diff_err_y = 0;

    err_x = target_x - x;
    err_y = target_y - y;
    err_oz = atan2(err_y, err_x);
    int velocity = copysignf(target_velocity, err_oz);
    const float distance = 0.5 * sqrt(pow(err_x, 2) + pow(err_y, 2));
    // this->Rotate(err_oz, target_velocity);
    Car::Move(velocity, -velocity, distance);

    do
    {
        this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
        diff_err_x = ((target_x - x) - err_x) / dt;
        diff_err_y = ((target_y - y) - err_y) / dt;
        err_x = target_x - x;
        err_y = target_y - y;
        err_oz = atan2(err_y, err_x) - oz;
        sum_err_x += err_x * dt;
        sum_err_y += err_y * dt;
        
        const double velocity_x = target_velocity * (Kp * err_x + Ki * sum_err_x + Kd * diff_err_x);
        const double velocity_y = target_velocity * (Kp * err_y + Ki * sum_err_y + Kd * diff_err_y);
        velocity = copysignf(sqrt(pow(velocity_x, 2) + pow(velocity_y, 2)), atan2(err_y, err_x));

        cout << "XY Error: " << err_x << "   " << err_y << endl;
        cout << "XY Velocity: " << velocity << endl;

        if (thread_break)
            break;

        Car::MoveDirection(err_oz, 0.0f, velocity); 
    } while (abs(err_x) > threshold || abs(err_y) > threshold);
    Stop();
}

void AGV::MoveForward(const float distance, const int &velocity)
{
    if (oz >= -M_PI_4 and oz < M_PI_4)
        Move(x + distance, y, oz, velocity);
    else if (oz >= M_PI_4 and oz < 3 * M_PI_4)
        Move(x, y + distance, oz, velocity);
    else if (oz >= -3 * M_PI_4 and oz < -M_PI_4)
        Move(x, y - distance, oz, velocity);
    else
        Move(x - distance, y, oz, velocity);
}

void AGV::MoveBackward(const float distance, const int &velocity)
{
    MoveForward(-distance, velocity);
}

void AGV::Rotate(float target_oz, const int &target_velocity)
{
    float direction = abs(target_oz - oz);
    if (abs(direction) > 3 * M_PI_2)
        direction = copysignf(2 * M_PI - abs(direction), -direction);
    Car::Rotate(direction, target_velocity);

    float err_oz = 0;
    float sum_err_oz = 0;
    float diff_err_oz = 0;

    do
    {
        this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
        diff_err_oz = (abs(target_oz - oz) - err_oz) / dt;
        err_oz = abs(target_oz - oz);
        if (abs(err_oz) > 3 * M_PI_2)
            target_oz = copysignf(2 * M_PI - abs(target_oz), -target_oz);
        sum_err_oz += err_oz * dt;
        int diff_velocity = Koz * target_velocity * (Kp * err_oz + Ki * sum_err_oz + Kd * diff_err_oz);

        cout << "Oz Error: " << err_oz << "   " << target_oz << "   " << oz << endl;
        cout << "Oz Velocity: " << diff_velocity << endl;
        
        if (thread_break)
            break;

        Car::Move(-diff_velocity, -diff_velocity, 0.0f);
    } while (abs(err_oz) > threshold);
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
    sub_pos = n.subscribe("/orb_slam2_rgbd/pose", 0, &AGV::PosCallBack, this);
    while (ros::ok && !delete_thread_pos)
    {
        ros::spinOnce();
        this_thread::sleep_for(std::chrono::milliseconds(1));
    }
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