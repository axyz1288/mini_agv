#include "./AGV/AGV.h"
#include <ros/ros.h>

const string Ip2NodeName(const string ip)
{
    if(ip == "192.168.50.249")
        return "agv0";
    else if(ip == "192.168.50.20")
        return "agv1";
    else if(ip == "192.168.50.55")
        return "agv2";
    else if(ip == "192.168.50.")
        return "agv3";
    else if(ip == "192.168.50.")
        return "agv4";
    else if(ip == "192.168.50.")
        return "agv5";
    else
        return "0";
}

int main(int argc, char *argv[])
{
    const string nodeName = Ip2NodeName(argv[1]);
    ros::init(argc, argv, nodeName);
    AGV *agv = AGV::getAGV(nodeName, argv[2], argv[3]);
    while(ros::ok)
    {
        agv->CheckData();
        if(agv->GetAction() == 0 || agv->GetAction() == 5)
            agv->Stop();
        else if(agv->GetAction() == 1 || agv->GetAction() == 2 || agv->GetAction() == 3 || agv->GetAction() == 4)
        {
            /* 
            callfunc is a pointer to a member. 
            It means that it points to an void* member variable that is declared in the class AGV.
            */
            void (AGV::*callfunc)(const float, const float, const int &) = &AGV::Move;
            thread* forward = new thread(callfunc, agv, agv->GetNextX(), agv->GetNextY(), 100);
        }
        else if(agv->GetAction() == 6)
            agv->Put();
        else
        {
            cout << "Out of action space" << endl;
            break;
        }
    }
    return 0;
}