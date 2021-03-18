#include "./AGV/AGV.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, argv[1]);
    AGV *agv = AGV::getAGV(argv[1], argv[2], argv[3]);
    while(ros::ok)
    {
        cout << "check data\n" << flush;
        agv->CheckData();
        cout << agv->GetAction() << '\n' << flush;
        if(agv->GetAction() == 0)
            agv->PubDone();
        else if(agv->GetAction() == 1 || agv->GetAction() == 2 || agv->GetAction() == 3 || agv->GetAction() == 4)
            agv->Move(agv->GetNextX(), agv->GetNextY(), 50);
        else if(agv->GetAction() == 5)
            agv->Pick();
        else if(agv->GetAction() == 6)
            agv->Put();
        else
            cout << "Out of action space\n" << flush;
        agv->ClearData();
    }
    return 0;
}