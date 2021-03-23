#include "Car.h"

const int Car::default_velocity = 150;
Car *Car::inst_ = nullptr;
Car *Car::getCar(const string &node_name, const string &env_name, const string &agent_name)
{
    if (inst_ == nullptr)
        inst_ = new Car(node_name, env_name, agent_name);
    return inst_;
}

Car::Car(const string &node_name, const string &env_name, const string &agent_name)
    : MotorUnion({0, 1, 2, 3}, {"Mx106", "Mx106", "Mx106", "Mx106"}),
      wheel_L(0),
      wheel_R(1),
      Conveyor_R(2),
      Conveyor(3),
      wheel_radius(0.075),
      VEL2METER_MS(GetMotor_Scale2RPM(wheel_L) * (1.667 * 1e-5) * 2 * M_PI * wheel_radius),
      ACCEL2METER_MS2(GetMotor_Scale2RPMM(wheel_L) * pow(1.667 * 1e-5, 2) * 2 * M_PI * wheel_radius),
      kAxle_2(0.130),
      kWheelBase_2(0.1),
      max_velocity(210),
      node_name(node_name),
      env_name(env_name),
      agent_name(agent_name),
      idx(atoi(&node_name[3])),
      info(*(ros::topic::waitForMessage<std_msgs::Float32MultiArray>('/' + env_name + "/info", ros::Duration(600)))),
      num_agent(int(info.data.at(0)))
{
    SetAllMotorsAccel(10);
    SetMotor_Operating_Mode(wheel_L, 1);
    SetMotor_Operating_Mode(wheel_R, 1);
    SetMotor_Operating_Mode(Conveyor, 1);
    SetMotor_Velocity(Conveyor_R, 80);
    SetMotor_Angle(Conveyor_R, 0);
    SetAllMotorsTorqueEnable(true);

    InitialRos();
}

Car::~Car()
{
    inst_ = nullptr;
    delete_thread_sub = true;
    delete_thread_emstop = true;
    thread_sub.join();
    thread_emstop.join();
}

/* 
Control
*/
void Car::Move(const int &velocity_L, const int &velocity_R, const float &distance)
{
    float tmp_velocity_L = velocity_L;
    float tmp_velocity_R = velocity_R;

    // velocity clamp
    if (fabs(velocity_L) > max_velocity)
        tmp_velocity_L = copysignf(max_velocity, velocity_L);
    if (fabs(velocity_R) > max_velocity)
        tmp_velocity_R = copysignf(max_velocity, velocity_R);

    if (distance == 0)
    {
        if (!GetMotor_TorqueEnable(wheel_L))
            SetMotor_TorqueEnable(wheel_L, true);
        if (!GetMotor_TorqueEnable(wheel_R))
            SetMotor_TorqueEnable(wheel_R, true);
        SetMotor_Velocity(wheel_L, tmp_velocity_L);
        SetMotor_Velocity(wheel_R, tmp_velocity_R);
    }
    else
    {
        const int accel = abs(GetMotor_Accel(wheel_L));
        const int velocity = (abs(velocity_L) + abs(velocity_R)) / 2;

        const int present_velocity_L = GetMotor_PresentVelocity(wheel_L);
        const int present_velocity_R = GetMotor_PresentVelocity(wheel_R);
        const int present_velocity = (abs(present_velocity_L) + abs(present_velocity_R)) / 2;

        float rising_time = abs(velocity - present_velocity) * VEL2METER_MS / (accel * ACCEL2METER_MS2);
        float steady_time = (distance - 2 * rising_time * (present_velocity * VEL2METER_MS + accel * ACCEL2METER_MS2 * rising_time / 2)) /
                            (velocity * VEL2METER_MS);
        if (steady_time < 0)
        {
            rising_time = (-present_velocity * VEL2METER_MS +
                           sqrt(pow(present_velocity * VEL2METER_MS, 2) + 4 * accel * ACCEL2METER_MS2 * abs(distance))) /
                          (2 * accel * ACCEL2METER_MS2);
            steady_time = 0;
            const int accel_L = copysignf(accel, velocity_L - present_velocity_L);
            const int accel_R = copysignf(accel, velocity_R - present_velocity_R);
            tmp_velocity_L = copysignf((present_velocity_L * VEL2METER_MS + accel_L * ACCEL2METER_MS2 * rising_time) / VEL2METER_MS, velocity_L);
            tmp_velocity_R = copysignf((present_velocity_R * VEL2METER_MS + accel_R * ACCEL2METER_MS2 * rising_time) / VEL2METER_MS, velocity_R);
        }
        if (!GetMotor_TorqueEnable(wheel_L))
            SetMotor_TorqueEnable(wheel_L, true);
        if (!GetMotor_TorqueEnable(wheel_R))
            SetMotor_TorqueEnable(wheel_R, true);
        SetMotor_Velocity(wheel_L, tmp_velocity_L);
        SetMotor_Velocity(wheel_R, tmp_velocity_R);
        WaitAllMotorsArrival(steady_time + rising_time);
        SetMotor_Velocity(wheel_L, 0);
        SetMotor_Velocity(wheel_R, 0);
        WaitAllMotorsArrival(rising_time);
    }
}

void Car::MoveDirection(const float &direction, const float &distance, const int &velocity)
{
    // Calculate Ackermann steering model:
    // this radius is circle center to center of back wheel
    const float radius = kWheelBase_2 / tan(direction + 1e-10);
    const float left_radius = radius - kAxle_2;
    const float right_radius = radius + kAxle_2;
    float left_velocity = left_radius / radius * velocity;
    float right_velocity = right_radius / radius * velocity;

    Move(int(left_velocity), -int(right_velocity), distance);
}

void Car::MoveForward(const float &distance, const int &speed)
{
    Move(abs(speed), -abs(speed), distance);
}

void Car::MoveBackward(const float &distance, const int &speed)
{
    Move(-abs(speed), abs(speed), distance);
}

void Car::Rotate(const float &direction, const int &speed)
{
    const float distance = kAxle_2 * abs(direction);
    const int velocity = copysignf(abs(speed), -direction); 
    Move(velocity, velocity, distance);
}

void Car::RotateLeft(const float &direction, const int &speed)
{
    Rotate(abs(direction), speed);
}

void Car::RotateRight(const float &direction, const int &speed)
{
    Rotate(-abs(direction), speed);
}

void Car::Stop()
{
    is_stop = true;
    SetMotor_Velocity(wheel_L, 0);
    SetMotor_Velocity(wheel_R, 0);
    SetMotor_TorqueEnable(wheel_L, false);
    SetMotor_TorqueEnable(wheel_R, false);
    this_thread::sleep_for(std::chrono::milliseconds(200));
}

void Car::RotateConveyor(const float &direction)
{
    SetMotor_Angle(Conveyor_R, direction * Rad2Angle);
}

void Car::Put(const int &velocity)
{
    SetMotor_Velocity(Conveyor, -velocity);
    this_thread::sleep_for(std::chrono::seconds(5));
    SetMotor_Velocity(Conveyor, 0);
}

/* 
Ros
*/
void Car::InitialRos()
{
    sub_a = n.subscribe('/' + agent_name + "/action", 1000, &Car::ActionCallBack, this);
    sub_r = n.subscribe('/' + env_name + "/reward", 1000, &Car::RewardCallBack, this);
    sub_done = n.subscribe('/' + env_name + "/done", 1000, &Car::DoneCallBack, this);
    pub_done = n.advertise<std_msgs::Float32MultiArray>('/' + node_name + "/done", 1000);
    thread_sub = thread(&Car::Sub, this);
    thread_emstop = thread(&Car::EmergencyStop, this);
}

void Car::Sub()
{
    while (ros::ok && !delete_thread_sub)
    {
        ros::spinOnce();
        this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Car::ActionCallBack(const std_msgs::Float32MultiArray &msg)
{
    action.push_back(msg.data);
}

void Car::RewardCallBack(const std_msgs::Float32MultiArray &msg)
{
    reward.push_back(msg.data);
}

void Car::DoneCallBack(const std_msgs::Float32MultiArray &msg)
{
    done.push_back(msg.data);
}

void Car::EmergencyStop()
{
    while (!delete_thread_emstop)
    {
        done = {};
        while (done.size() < num_agent)
            this_thread::sleep_for(std::chrono::milliseconds(50));
        if(GetDone() == true)
        {
            Stop();
            exit(0);
        }
    }
}

void Car::CheckData()
{
    while (action.size() < num_agent || reward.size() < num_agent)
        this_thread::sleep_for(std::chrono::milliseconds(1));

    if (reward.at(idx).at(0) < 0)
        action[idx][0] = 0; 
}

void Car::ClearData()
{
    for(int i = 0; i < num_agent; i++)
    {
        action.erase(action.begin());
        reward.erase(reward.begin());
    }
}

const int Car::GetAction()
{
    return int(action.at(idx).at(0));
}

const bool Car::GetDone()
{
    return bool(done.at(idx).at(0));
}

void Car::PubDone()
{
    std_msgs::Float32MultiArray msg;
    msg.data = {1};
    while(pub_done.getNumSubscribers() == 0)
        this_thread::sleep_for(std::chrono::milliseconds(50));
    pub_done.publish(msg);
    ros::spinOnce();
}