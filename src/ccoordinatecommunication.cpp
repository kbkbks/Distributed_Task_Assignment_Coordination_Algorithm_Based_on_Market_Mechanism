#include "ccoordinatecommunication.h"

/*
 * 构造函数
 */
ccoordinatecommunication::ccoordinatecommunication(crobot * Robot) : CurrentRobot(Robot)
{
    switch (CurrentRobot->sendRobotNum())
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

            //读CoorTEQ和CurrentTEQ
            readTEQ();

            //协调算法
            cout << "机器人" << CurrentRobot->sendRobotNum() << "协调算法" << endl;

            //写CurrentTEQ和CoorTEQ
            writeTEQ();

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
    switch (CurrentRobot->sendRobotNum())
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
    switch (CurrentRobot->sendRobotNum())
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

/*
 * 读CoorTEQ
 */
void ccoordinatecommunication::readTEQ()
{
    switch (CurrentRobot->sendRobotNum())
    {
    case 0:
    {
        //robot0向robot1进行协调，向robot[1]读数据
        unique_lock<mutex> lck1_0(TEQrw1_0);
        CurrentRobot->updateTaskExecutionQueue(GlobalTEQ1_0, 0);   //读函数
        CurrentRobot->updateNewCoorTEQ(GlobalNewCoorTEQ1_0);
        break;
    }

    case 1:
    {
        //robot1向robot0和robot2进行协调，向robot[0]和robot[2]读数据
        unique_lock<mutex> lck0_1(TEQrw0_1);    
        unique_lock<mutex> lck2_1(TEQrw2_1);
        CurrentRobot->updateTaskExecutionQueue(GlobalTEQ0_1, 0);   //读函数
        CurrentRobot->updateTaskExecutionQueue(GlobalTEQ2_1, 1);   //读函数
    }

    case 2:
    {
        //robot2向robot[1]和robot[3]进行协调，向robot[1]和robot[3]读数据
        unique_lock<mutex> lck1_2(TEQrw1_2);
        unique_lock<mutex> lck3_2(TEQrw3_2);
        CurrentRobot->updateTaskExecutionQueue(GlobalTEQ1_2, 0);   //读函数
        CurrentRobot->updateTaskExecutionQueue(GlobalTEQ3_2, 1);   //读函数
        break;
    }

    case 3:
    {
        //robot3向robot[2]和robot[4]进行协调，向robot[2]和robot[4]读数据
        unique_lock<mutex> lck2_3(TEQrw2_3);
        unique_lock<mutex> lck4_3(TEQrw4_3);
        CurrentRobot->updateTaskExecutionQueue(GlobalTEQ2_3, 0);   //读函数
        CurrentRobot->updateTaskExecutionQueue(GlobalTEQ4_3, 1);   //读函数
        break;
    }

    case 4:
    {
        //robot4向robot[3]和robot[5]进行协调，向robot[3]和robot[5]读数据
        unique_lock<mutex> lck3_4(TEQrw3_4);
        unique_lock<mutex> lck5_4(TEQrw5_4);
        CurrentRobot->updateTaskExecutionQueue(GlobalTEQ3_4, 0);   //读函数
        CurrentRobot->updateTaskExecutionQueue(GlobalTEQ5_4, 1);   //读函数
        break;
    }

    case 5:
    {
        //robot5向robot[4]进行协调，向robot[4]读数据
        unique_lock<mutex> lck4_5(TEQrw4_5);
        CurrentRobot->updateTaskExecutionQueue(GlobalTEQ4_5, 0);   //读函数
        break;
    }

    default:
        break;
    }

}

/*
 * 写CoorTEQ
 */
void ccoordinatecommunication::writeTEQ()
{
    switch (CurrentRobot->sendRobotNum())
    {
    case 0:
    {
        unique_lock<mutex> lck1_0(TEQrw0_1); 
        GlobalTEQ0_1 = CurrentRobot->setTaskExecutionQueue();  //写函数
        GlobalNewCoorTEQ0_1 = CurrentRobot->setNewCoorTEQ(0); //写函数
        break;
    }

    case 1:
    {
        //robot[1]向robot[0]和robot[2]写数据，robot[0]和robot[2]进行协调
        unique_lock<mutex> lck1_0(TEQrw1_0);
        unique_lock<mutex> lck1_2(TEQrw1_2);
        GlobalTEQ1_0 = CurrentRobot->setTaskExecutionQueue();  //写函数
        GlobalTEQ1_2 = CurrentRobot->setTaskExecutionQueue();  //写函数
        GlobalNewCoorTEQ1_0 = CurrentRobot->setNewCoorTEQ(0);  //写函数
        GlobalNewCoorTEQ1_2 = CurrentRobot->setNewCoorTEQ(1);  //写函数
        break;
    }

    case 2:
    {
        //robot[2]向robot[1]和robot[3]写数据，robot[1]和robot[3]进行协调
        unique_lock<mutex> lck2_1(TEQrw2_1);
        unique_lock<mutex> lck2_3(TEQrw2_3);
        GlobalTEQ2_1 = CurrentRobot->setTaskExecutionQueue();  //写函数
        GlobalTEQ2_3 = CurrentRobot->setTaskExecutionQueue();  //写函数
        GlobalNewCoorTEQ2_1 = CurrentRobot->setNewCoorTEQ(0);  //写函数
        GlobalNewCoorTEQ2_3 = CurrentRobot->setNewCoorTEQ(1);  //写函数
        break;
    }

    case 3:
    {
        //robot3向robot[2]和robot[4]写数据，robot[2]和robot[4]进行协调
        unique_lock<mutex> lck3_2(TEQrw3_2);
        unique_lock<mutex> lck3_4(TEQrw3_4);
        GlobalTEQ3_2 = CurrentRobot->setTaskExecutionQueue();  //写函数
        GlobalTEQ3_4 = CurrentRobot->setTaskExecutionQueue();  //写函数
        GlobalNewCoorTEQ3_2 = CurrentRobot->setNewCoorTEQ(0);  //写函数
        GlobalNewCoorTEQ3_4 = CurrentRobot->setNewCoorTEQ(1);  //写函数
        break;
    }

    case 4:
    {
        //robot4向robot[3]和robot[5]写数据，robot[3]和robot[5]进行协调
        unique_lock<mutex> lck4_3(TEQrw4_3);
        unique_lock<mutex> lck4_5(TEQrw4_5);
        GlobalTEQ4_3 = CurrentRobot->setTaskExecutionQueue();  //写函数
        GlobalTEQ4_5 = CurrentRobot->setTaskExecutionQueue();  //写函数
        GlobalNewCoorTEQ4_3 = CurrentRobot->setNewCoorTEQ(0);  //写函数
        GlobalNewCoorTEQ4_5 = CurrentRobot->setNewCoorTEQ(1);  //写函数
        break;
    }

    case 5:
    {
        //robot5向robot[4]写数据，robot[4]进行协调
        unique_lock<mutex> lck5_4(TEQrw5_4);
        GlobalTEQ5_4 = CurrentRobot->setTaskExecutionQueue();  //写函数
        GlobalNewCoorTEQ5_4 = CurrentRobot->setNewCoorTEQ(0);  //写函数
        break;    
    }
    
    default:
        break;
    }
}