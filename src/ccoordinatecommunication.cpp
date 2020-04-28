#include "ccoordinatecommunication.h"

/*
 * 构造函数
 */
ccoordinatecommunication::ccoordinatecommunication(crobot * Robot) : CurrentRobot(*Robot)
{
    switch (CurrentRobot.sendRobotNum())
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
        muCoorStatus.lock();
        readCoorStatus();

        //判断协调条件
        bool Status = checkCoorStatus();
        if (Status == true)
        //自身可协调，邻接机器人可协调，满足协调三规则
        {
            //锁协调状态
            for (auto iter = CoorStatus.begin(); iter != CoorStatus.end(); ++iter)
            {
                *iter = false;
            }
            writeCoorStatus();
            muCoorStatus.unlock();

            //读CoorTEQ

            //协调算法
            cout << "机器人" << CurrentRobot.sendRobotNum() << "协调算法" << endl;
            //写CoorTEQ

            //恢复协调状态
            muCoorStatus.lock();
            for (auto iter = CoorStatus.begin(); iter != CoorStatus.end(); ++iter)
            {
                *iter = true;
            }
            writeCoorStatus();
            muCoorStatus.unlock();

            break;
        }
        else
        {
            muCoorStatus.unlock();
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

/*
 * 读协调状态，下标0为当前机器人协调状态，下标1和2按字典序存放邻接机器人协调状态
 */
void ccoordinatecommunication::readCoorStatus()
{
    switch (CurrentRobot.sendRobotNum())
    {
    case 0:
        CoorStatus[0] = GlobalCoorStatus[0];
        CoorStatus[1] = GlobalCoorStatus[1];
        break;
    
    case 1:
        CoorStatus[0] = GlobalCoorStatus[1];
        CoorStatus[1] = GlobalCoorStatus[0];
        CoorStatus[2] = GlobalCoorStatus[2];
        break;
    
    case 2:
        CoorStatus[0] = GlobalCoorStatus[2];
        CoorStatus[1] = GlobalCoorStatus[1];
        CoorStatus[2] = GlobalCoorStatus[3];
        break;

    case 3:
        CoorStatus[0] = GlobalCoorStatus[3];
        CoorStatus[1] = GlobalCoorStatus[2];
        CoorStatus[2] = GlobalCoorStatus[4];
        break;

    case 4:
        CoorStatus[0] = GlobalCoorStatus[4];
        CoorStatus[1] = GlobalCoorStatus[3];
        CoorStatus[2] = GlobalCoorStatus[5];
        break;

    case 5:
        CoorStatus[0] = GlobalCoorStatus[5];
        CoorStatus[1] = GlobalCoorStatus[4];
        break;

    default:
        break;
    }
}

/*
 * 写协调状态
 */
void ccoordinatecommunication::writeCoorStatus()
{
    switch (CurrentRobot.sendRobotNum())
    {
    case 0:
        GlobalCoorStatus[0] = CoorStatus[0];
        GlobalCoorStatus[1] = CoorStatus[1];
        break;
    
    case 1:
        GlobalCoorStatus[1] = CoorStatus[0];
        GlobalCoorStatus[0] = CoorStatus[1];
        GlobalCoorStatus[2] = CoorStatus[2];
        break;

    case 2:
        GlobalCoorStatus[2] = CoorStatus[0];
        GlobalCoorStatus[1] = CoorStatus[1];
        GlobalCoorStatus[3] = CoorStatus[2];
        break;

    case 3:
        GlobalCoorStatus[3] = CoorStatus[0];
        GlobalCoorStatus[2] = CoorStatus[1];
        GlobalCoorStatus[4] = CoorStatus[2];
        break;

    case 4:
        GlobalCoorStatus[4] = CoorStatus[0];
        GlobalCoorStatus[3] = CoorStatus[1];
        GlobalCoorStatus[5] = CoorStatus[2];
        break;

    case 5:
        GlobalCoorStatus[5] = CoorStatus[0];
        GlobalCoorStatus[4] = CoorStatus[1];
        break;

    default:
        break;
    }
}