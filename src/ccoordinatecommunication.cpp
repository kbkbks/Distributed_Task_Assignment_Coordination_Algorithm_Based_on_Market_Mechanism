#include "ccoordinatecommunication.h"

/*
 * 构造函数
 */
ccoordinatecommunication::ccoordinatecommunication(crobot * Robot)
{
    // for (int i = 0; i < ROBOTNUM; ++i)
    // {
    //     CoorStatus[i] = true;
    // }
    switch (Robot->sendRobotNum())
    {
    case 0:
        CoorStatus = {true, true};
        break;
    
    case 1:
    case 2:
    case 3:
    case 4:
        CoorStatus = {true, true, true};
        break;

    case 5:
        CoorStatus = {true, true};
        break;

    default:
        break;
    }
}

/*
 * 机器人协调通信
 */
void ccoordinatecommunication::enterCoordinate()
{
    while(1)
    {
        //读协调状态
        

        bool Status = checkCoorStatus();
        if (Status == true)
        //自身可协调，邻接机器人可协调，满足协调三规则
        {

        }

    }
}

/*
 * 状态检测
 */
bool ccoordinatecommunication::checkCoorStatus()
{
    bool StatusGathered = false;
    for(auto iter = CoorStatus.begin(); iter != CoorStatus.end(); ++iter)
    {
        if (*iter == false)
        {
            StatusGathered = false;
            break;
        }
        else
        {
            StatusGathered = true;
        }
    }

    return StatusGathered;
}