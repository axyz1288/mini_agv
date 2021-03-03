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
      map_unit(0.6),
      Kp(1.5),
      Ki(0.5),
      Kd(0.06),
      Koz(1),
      dt(0.01),
      threshold(0.02)
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
    Rotate(abs_err_oz, speed);
    MoveDirection(target_x, target_y, speed);
}

void AGV::MoveDirection(const float target_x, const float target_y, const int &speed)
{   
    float abs_err_x, abs_err_y, abs_oz;
    float err_x, err_y, err_oz;
    float sum_err_x, sum_err_y, sum_err_oz;
    float diff_err_x, diff_err_y, diff_err_oz;
    abs_err_x = abs_err_y = abs_oz = 0;
    err_x = err_y = err_oz = 0;
    sum_err_x = sum_err_y = 0;
    diff_err_x = diff_err_y  = 0;

    abs_err_x = target_x - x;
    abs_err_y = target_y - y;
    float distance = sqrt(pow(abs_err_x, 2) + pow(abs_err_y, 2));
    const float target_oz = atan2(abs_err_y, abs_err_x);
    Car::Rotate(rand()/RAND_MAX * 0.016 - 0.08, speed);

    do
    {
        abs_err_x = target_x - x;
        abs_err_y = target_y - y;
        abs_oz = target_oz - oz;
        distance = sqrt(pow(abs_err_x, 2) + pow(abs_err_y, 2));
        err_oz = atan2(abs_err_y, abs_err_x) - oz;
        diff_err_x = ((distance * cos(err_oz)) - err_x) / dt;
        diff_err_y = ((distance * sin(err_oz)) - err_y) / dt;
        err_x = distance * cos(err_oz);
        err_y = distance * sin(err_oz);
        sum_err_x += err_x * dt;
        sum_err_y += err_y * dt;

		const float velocity_x = abs(speed) * (Kp * err_x + Ki * sum_err_x + Kd * diff_err_x);
    	const float velocity_y = abs(speed) * (Kp * err_y + Ki * sum_err_y + Kd * diff_err_y);
        const int velocity = copysignf(sqrt(pow(velocity_x, 2) + pow(velocity_y, 2)), velocity_x);
		
        cout << "Error: " << err_x << "   " << err_y << "   " << err_oz << "   " << abs_oz << endl;
        cout << "Velocity: " << velocity << endl;

        if (move_break)
            break;
		if (abs_oz > 3/2 * M_PI)
			abs_oz = copysignf(2 * M_PI - abs(abs_oz), -abs_oz);
        Car::MoveDirection(2 * abs_oz, 0.0f, velocity);
        this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    } while (abs(err_x) > threshold || abs(err_y) > 2 * threshold);
    Stop();
    cout << "Finished" << endl;
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

        cout << "Oz Error: " << err_oz << endl;
        cout << "Oz Velocity: " << diff_velocity << endl;
        
        if (move_break)
            break;

        Car::Move(-diff_velocity, -diff_velocity, 0.0f);
        this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    } while (abs(err_oz) > 0.2 * threshold);
    Stop();
    cout << "Finished" << endl;
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

    Car::MoveBackward(kWheelBase_2, speed);
    Rotate(oz + direction, speed);
    Car::MoveForward(kWheelBase_2, speed);
    Stop();
    cout << target_x - x << "   " << target_y - y << "   " << target_oz - oz << endl;
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
    const float direction = oz * Rad2Angle;
    RotateConveyor(-direction);
    this_thread::sleep_for(std::chrono::milliseconds(1500));
    Car::Put(velocity);
    RotateConveyor(0);
}



/* 
Ros
*/
void AGV::InitialMap()
{
    /* Disable map update */
    slamware_ros_sdk::SetMapUpdateRequest msg;
    msg.enabled = false;
    ros::Publisher mapUpdatePub = n.advertise<slamware_ros_sdk::SetMapUpdateRequest>("/slamware_ros_sdk_server_node/set_map_update", 1);
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
    ros::ServiceClient client = n.serviceClient<slamware_ros_sdk::SyncSetStcm>("/slamware_ros_sdk_server_node/sync_set_stcm");
    client.waitForExistence();
    if (client.call(srvMsg))
        printf("Succeeded in calling syncSetStcm.\n");
    else
        printf("Failed to call syncSetStcm.\n");

    /* Relocalize */
    while (now_state.size() < num_agent)
        this_thread::sleep_for(std::chrono::milliseconds(1));
    const float now_x = now_state.at(idx).at(0) * map_unit;
    const float now_y = now_state.at(idx).at(1) * map_unit;
    sub_now_state.shutdown();
    x = y = -1; // Initialize with a unreachable point
    geometry_msgs::Pose pose;
    pose.position.x = now_x;
    pose.position.y = now_y;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    ros::Publisher pub_pose = n.advertise<geometry_msgs::Pose>("/slamware_ros_sdk_server_node/set_pose", 0);
    while(x != now_x || y != now_y)
    {
        pub_pose.publish(pose);
        ros::spinOnce();
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void AGV::InitialRos()
{
    thread_sub = thread(&AGV::Sub, this);
    pub_done = n.advertise<std_msgs::Bool>('/' + node_name + "/done", 1000);
}

void AGV::PubDone()
{
    std_msgs::Bool msg;
    msg.data = true;
    pub_done.publish(msg);
}

void AGV::Sub()
{
    sub_pos = n.subscribe("/slamware_ros_sdk_server_node/odom", 1000, &AGV::PosCallBack, this);
    sub_now_state = n.subscribe('/' + env_name + "/now_state", 1000, &AGV::NowStateCallBack, this);
    sub_next_state = n.subscribe('/' + env_name + "/next_state", 1000, &AGV::NextStateCallBack, this);
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

void AGV::CheckData()
{
    next_state = {};
    Car::CheckData();
    while (next_state.size() < num_agent)
        this_thread::sleep_for(std::chrono::milliseconds(1));
    next_x = next_state.at(idx).at(0) * map_unit;
    next_y = next_state.at(idx).at(1) * map_unit;
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