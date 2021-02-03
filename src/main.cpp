#include "./AGV/AGV.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, argv[1]);
    AGV *car = AGV::getAGV(argv[1], argv[2]);
    while(ros::ok)
    {
        int type;
        cin >> type;
        // car->CheckData();
        if(type == 0 || type == 5)
            car->Stop();
        else if(type == 1)
            thread* forward = new thread(&AGV::MoveForward, car, 0.5f, 100);
        else if(type == 2)
            thread* backward = new thread(&AGV::MoveBackward, car, 0.5f, 100);
        else if(type == 3)
            thread* left = new thread(&AGV::RotateLeft, car, M_PI_2, 100);
        else if(type == 4)
            thread* right = new thread(&AGV::RotateRight, car, M_PI_2, 100);
        else if(type == 6)
            car->Put();
        else
        {
            cout << "Out of action space" << endl;
            break;
        }
    }
    return 0;
}