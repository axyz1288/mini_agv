#include "AGV.h"

AGV *AGV::inst_ = nullptr;
AGV *AGV::getAGV(const string &node_name, const string &env_name, const string &agent_name)
{
    if (inst_ == nullptr)
        inst_ = new AGV(node_name, env_name, agent_name);
    return inst_;
}

AGV::AGV(const string &node_name, const string &env_name, const string &agent_name)
    : Car(node_name, env_name, agent_name),
      map_w(int(info.data.at(-2 + info.data.size()))),
      map_h(int(info.data.at(-1 + info.data.size()))),
      map_unit(0.5),
      map_x_shift(0.2),
      map_y_shift(0.1),
      Kp(4),
      Ki(0.5),
      Kd(1),
      Koz(0.5),
      Kp_oz(1),
      Ki_oz(0),
      Kd_oz(15),
      dt(0.001),
      threshold(0.01)
{
    InitialRos();
    InitialMap();
}

AGV::~AGV()
{
    inst_ = nullptr;
    delete_thread_sub = true;
    thread_sub.join();
}

/* 
Control
*/
void AGV::Move(const float target_x, const float target_y, const float target_oz, const int &speed)
{
    Move(target_x, target_y, speed);
    Rotate(target_oz, speed);
}

void AGV::Move(const float target_x, const float target_y, const int &speed)
{
    const float abs_err_oz = atan2(target_y - y, target_x - x);
    if (abs(target_x - x) > map_unit / 2 || abs(target_y - y) > map_unit / 2)
    {
        Selfturn(abs_err_oz - oz, speed);
        // if (target_x == 0 - map_x_shift && target_y == 0.5 - map_y_shift)
        MoveDirection(target_x, target_y, speed);
        // else
        //     MoveDirection(target_x, target_y, speed);
    }
    PubDone();
}

void AGV::MoveDirection(const float target_x, const float target_y, const int &speed)
{   
    float abs_err_x, abs_err_y, abs_oz;
    float err_x, err_y, err_oz, err_oz2;
    float sum_err_x, sum_err_y, sum_err_oz;
    float diff_err_x, diff_err_y, diff_err_oz;
    abs_err_x = abs_err_y = abs_oz = 0;
    err_x = err_y = err_oz = err_oz2 = 0;
    sum_err_x = sum_err_y = 0;
    diff_err_x = diff_err_y  = 0;

    abs_err_x = target_x - x;
    abs_err_y = target_y - y;
    float distance = sqrt(pow(abs_err_x, 2) + pow(abs_err_y, 2));
    const float target_oz = atan2(abs_err_y, abs_err_x);

    do
    {
        abs_err_x = target_x - x;
        abs_err_y = target_y - y;
        abs_oz = target_oz - oz;
        distance = sqrt(pow(abs_err_x, 2) + pow(abs_err_y, 2));
        err_oz = atan2(abs_err_y, abs_err_x) - oz;
        // calculate error x, y
        diff_err_x = ((distance * cos(err_oz)) - err_x) / dt;
        diff_err_y = ((distance * sin(err_oz)) - err_y) / dt;
        err_x = distance * cos(err_oz);
        err_y = distance * sin(err_oz);
        sum_err_x += err_x * dt;
        sum_err_y += err_y * dt;
        // calculate error oz
        diff_err_oz = (err_oz - err_oz2)/dt;
        sum_err_oz += err_oz * dt; 
        err_oz2 = err_oz;

        const float direction = Kp_oz * err_oz + Ki_oz * sum_err_oz + Kd_oz * diff_err_oz;
		const float velocity_x = abs(speed) * (Kp * err_x + Ki * sum_err_x + Kd * diff_err_x);
    	const float velocity_y = abs(speed) * (Kp * err_y + Ki * sum_err_y + Kd * diff_err_y);
        const int velocity = copysignf(sqrt(pow(velocity_x, 2) + pow(velocity_y, 2)), velocity_x);

        if (move_break)
            break;
		if (abs_oz > 3/2 * M_PI)
			abs_oz = copysignf(2 * M_PI - abs(abs_oz), -abs_oz);
        Car::MoveDirection(direction, 0.0f, velocity);
        this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    } while (abs(err_x) > threshold || abs(err_y) > threshold);
    cout << x << "   " << y << "\n" << flush;
    Stop();
}

void AGV::MoveForward(const float distance, const int &speed)
{
    const float target_oz = oz;
    Move(x + distance * cos(target_oz), y + distance * sin(target_oz), target_oz, speed);
}

void AGV::MoveBackward(const float distance, const int &speed)
{
    /*
    This place cannot use MoveForward(-distance), because it will face to target position forcibly. 
    */
    const float target_oz = oz;
    // Rotate opposite direction first.
    const float abs_err_oz = atan2(distance * sin(target_oz), distance * cos(target_oz));
    Rotate(abs_err_oz, speed);
    MoveDirection(x - distance * cos(target_oz), y - distance * sin(target_oz), speed);
    Rotate(target_oz);
}

void AGV::MoveLeft(const float distance, const int &speed)
{
    const float target_oz = oz + M_PI_2;
    Move(x + distance * cos(target_oz), y + distance * sin(target_oz), target_oz, speed);
}

void AGV::MoveRight(const float distance, const int &speed)
{
    const float target_oz = oz - M_PI_2;
    Move(x + distance * cos(target_oz), y + distance * sin(target_oz), target_oz, speed);
}

void AGV::Rotate(float target_oz, const int &speed)
{
    float direction = target_oz - oz;
    if (abs(direction) > 3/2 * M_PI)
        direction = copysignf(2 * M_PI - abs(direction), -direction);
    Car::Rotate(direction, speed);

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
        int diff_velocity = Koz * abs(speed) * (Kp * err_oz + Ki * sum_err_oz + Kd * diff_err_oz);
        
        if (move_break)
            break;

        Car::Move(-diff_velocity, -diff_velocity, 0.0f);
        this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    } while (abs(err_oz) > threshold);
    cout << oz << "   " << flush;
    Stop();
}

void AGV::RotateLeft(const float direction, const int &speed)
{
    Rotate(oz + abs(direction), speed);
}

void AGV::RotateRight(const float direction, const int &speed)
{
    Rotate(oz - abs(direction), speed);
}

void AGV::Selfturn(const float direction, const int &speed)
{
	const float target_x = x;
	const float target_y = y;
    const float target_oz = oz + direction;

    Car::MoveForward(kWheelBase_2, speed);
    Rotate(oz + direction, speed);
    Car::MoveBackward(kWheelBase_2, speed);
    Stop();
}

void AGV::Stop()
{
    move_break = true;
    Car::Stop();
    this_thread::sleep_for(std::chrono::milliseconds(100));
    move_break = false;
}

void AGV::RotateConveyor(const float &direction)
{
    Car::RotateConveyor(direction);
}

void AGV::Put(const int &velocity)
{
    RotateConveyor(-oz);
    this_thread::sleep_for(std::chrono::milliseconds(2000));
    Car::Put(velocity);
    RotateConveyor(0);
    PubDone();
}

void AGV::Pick()
{
    std_msgs::Int8 msg;
    msg.data = 2;
    while(pub_product.getNumSubscribers() < num_agent + 2) //Env + Yuan control
        this_thread::sleep_for(std::chrono::milliseconds(50));
    pub_product.publish(msg);
    RotateConveyor(copysignf(M_PI, oz) - oz);
    
    while(is_product != 3)
        this_thread::sleep_for(std::chrono::milliseconds(500));
    // this_thread::sleep_for(std::chrono::milliseconds(5000));
    RotateConveyor(0);
    PubDone();
}

/* 
Ros
*/
void AGV::InitialMap()
{
    /* Disable map update */
    slamware_ros_sdk::SetMapUpdateRequest msg;
    msg.enabled = false;
    ros::Publisher mapUpdatePub = n.advertise<slamware_ros_sdk::SetMapUpdateRequest>('/' + node_name + "/slam/set_map_update", 1);
    mapUpdatePub.publish(msg);
    ros::spinOnce();

    /* Initial Map */
    // copy from slamware_ros_sdk/slamware_ros_sdk_client.cpp
    slamware_ros_sdk::SyncSetStcm srvMsg;
    boost::filesystem::ifstream ifs("/home/aiRobots/mini_agv/src/map.stcm", (std::ios_base::in | std::ios_base::binary | std::ios_base::ate));
    if (!ifs.is_open())
        printf("Failed to open file\n");
    const auto szDat = ifs.tellg();
    if (boost::filesystem::ifstream::pos_type(-1) == szDat)
        printf("Failed to get file size.\n");
    ifs.seekg(0);
    srvMsg.request.raw_stcm.resize(szDat);
    ifs.read((char*)srvMsg.request.raw_stcm.data(), szDat);
    if (ifs.gcount() != szDat)
        printf("Failed to read file data.\n");
    ros::ServiceClient client = n.serviceClient<slamware_ros_sdk::SyncSetStcm>('/' + node_name + "/slam/sync_set_stcm");
    client.waitForExistence();
    if (client.call(srvMsg))
        printf("Succeeded in calling syncSetStcm.\n");
    else
        printf("Failed to call syncSetStcm.\n");

    /* Relocalize */
    while (now_state.size() < num_agent)
        this_thread::sleep_for(std::chrono::milliseconds(1));
    const float now_x = now_state.at(idx).at(0) * map_w * map_unit - map_x_shift;
    const float now_y = now_state.at(idx).at(1) * map_h * map_unit - map_y_shift;
    x = y = -1; // Initialize with a unreachable point
    geometry_msgs::Pose pose;
    tf::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;
    quat_tf.setRPY(0, 0, 0);
    tf::quaternionTFToMsg(quat_tf, quat_msg);
    pose.position.x = now_x;
    pose.position.y = now_y;
    pose.position.z = 0;
    pose.orientation.x = quat_msg.x;
    pose.orientation.y = quat_msg.y;
    pose.orientation.z = quat_msg.z;
    pose.orientation.w = quat_msg.w;
    ros::Publisher pub_pose = n.advertise<geometry_msgs::Pose>('/' + node_name + "/slam/set_pose", 0);
    while(x != now_x || y != now_y || oz != 0)
    {
        pub_pose.publish(pose);
        ros::spinOnce();
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    Car::Rotate(2 * M_PI, 20);
}

void AGV::InitialRos()
{
    sub_pos = n.subscribe('/' + node_name + "/slam/odom", 1000, &AGV::PosCallBack, this);
    sub_now_state = n.subscribe('/' + env_name + "/now_state", 1000, &AGV::NowStateCallBack, this);
    sub_next_state = n.subscribe('/' + env_name + "/next_state", 1000, &AGV::NextStateCallBack, this);
    sub_product = n.subscribe("/six_arm/product", 1000, &AGV::ProductCallBack, this);
    pub_product = n.advertise<std_msgs::Int8>("/six_arm/product", 1000);
    thread_sub = thread(&AGV::Sub, this);
}

void AGV::Sub()
{
    while (ros::ok && !delete_thread_sub)
    {
        ros::spinOnce();
        this_thread::sleep_for(std::chrono::milliseconds(50));
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

void AGV::NowStateCallBack(const std_msgs::Float32MultiArray &msg)
{
    now_state.push_back(msg.data);
}

void AGV::NextStateCallBack(const std_msgs::Float32MultiArray &msg)
{
    next_state.push_back(msg.data);
}

void AGV::ProductCallBack(const std_msgs::Int8 &msg)
{
    is_product = msg.data;
}

void AGV::CheckData()
{
    Car::CheckData();
    while (now_state.size() < num_agent || next_state.size() < num_agent)
        this_thread::sleep_for(std::chrono::milliseconds(1));
    
    next_x = next_state.at(idx).at(0) * map_w * map_unit - map_x_shift;
    next_y = next_state.at(idx).at(1) * map_h * map_unit - map_y_shift;
}

void AGV::ClearData()
{
    Car::ClearData();
    for(int i = 0; i < num_agent; i++)
    {
        now_state.erase(now_state.begin());
        next_state.erase(next_state.begin());
    }
}

void AGV::PubDone()
{
    Car::PubDone();
}

const int AGV::GetAction()
{
    return Car::GetAction();
}

const float AGV::GetNextX()
{
    return next_x;
}

const float AGV::GetNextY()
{
    return next_y;
}

const float AGV::GetNowX()
{
    return x;
}

const float AGV::GetNowY()
{
    return y;
}