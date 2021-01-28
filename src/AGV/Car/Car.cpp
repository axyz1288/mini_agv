#include "Car.h"

const int Car::default_velocity = 150;
Car *Car::inst_ = nullptr;
Car *Car::getCar(const string &node_name, const string &agent_name)
{
    if (inst_ == nullptr)
        inst_ = new Car(node_name, agent_name);
    return inst_;
}

Car::Car(const string &node_name, const string &agent_name)
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

Car::~Car()
{
    inst_ = nullptr;
    delete_thread_a = true;
    thread_sub_a.join();
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
    float tmp_angle = direction;
    int tmp_velocity = velocity;

    // Calculate Ackermann steering model:
    // this radius is circle center to center of back wheel
    const float radius = kWheelBase_2 / tan(tmp_angle + 1e-10);

    const float left_radius = radius - kAxle_2;
    const float right_radius = radius + kAxle_2;
    float left_velocity = left_radius / radius * tmp_velocity;
    float right_velocity = right_radius / radius * tmp_velocity;

    Move(int(left_velocity), -int(right_velocity), distance);
}

void Car::MoveForward(const float &distance, const int &velocity)
{
    Move(abs(velocity), -abs(velocity), distance);
}

void Car::MoveBackward(const float &distance, const int &velocity)
{
    Move(-abs(velocity), abs(velocity), distance);
}

void Car::Rotate(const float &direction, const int &velocity)
{
    float distance = kAxle_2 * direction;
    Move(-velocity, -velocity, abs(distance));
}

void Car::RotateLeft(const float &direction, const int &velocity)
{
    Rotate(abs(direction), abs(velocity));
}

void Car::RotateRight(const float &direction, const int &velocity)
{
    Rotate(abs(direction), -abs(velocity));
}

void Car::Stop()
{
    SetMotor_Velocity(wheel_L, 0);
    SetMotor_Velocity(wheel_R, 0);
    SetMotor_TorqueEnable(wheel_L, false);
    SetMotor_TorqueEnable(wheel_R, false);
}

void Car::RotateConveyor(const float &direction)
{
    SetMotor_Angle(Conveyor_R, direction);
}

void Car::Put(const int &velocity)
{
    SetMotor_Velocity(Conveyor, -velocity);
    this_thread::sleep_for(std::chrono::seconds(3));
    SetMotor_Velocity(Conveyor, 0);
}

/* 
Ros
*/
void Car::InitialRos()
{
    thread_sub_a = thread(&Car::SubAction, this);
}

void Car::SubAction()
{
    sub_a = n.subscribe('/' + agent_name + "/action", 0, &Car::ActionCallBack, this);
    while (ros::ok && !delete_thread_a)
    {
        ros::spinOnce();
        this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Car::ActionCallBack(const std_msgs::Float32MultiArray &msg)
{
    action.push_back(msg.data);
}

void Car::CheckData()
{
    action = {};
    while (action.size() < 6)
        this_thread::sleep_for(std::chrono::milliseconds(1));
}

int Car::GetAction()
{
    return int(action.at(idx).at(0));
}