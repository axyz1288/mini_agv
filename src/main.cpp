#include "./Car/Car.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, argv[1]);
    Car *car = Car::getCar(argv[1], argv[2]);
    while(ros::ok)
    {
        car->CheckData();
        cout << car->GetAction() << endl;
        // if(car->GetAction() == 0 || car->GetAction() == 5)
        //     car->Stop();
        // else if(car->GetAction() == 1)
        //     car->MoveForward(0.5);
        // else if(car->GetAction() == 2)
        //     car->MoveBackward(0.5);
        // else if(car->GetAction() == 3)
        //     car->MoveLeft(0.1);
        // else if(car->GetAction() == 4)
        //     car->MoveRight(0.1);
        // else if(car->GetAction() == 6)
        //     car->Put();
        // else
        // {
        //     cout << "Out of action space" << endl;
        //     return -1;
        // }
    }
    return 0;
}