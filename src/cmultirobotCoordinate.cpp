#include "cmultirobotCoordinate.h"

/*
 * 构造函数
 */
cmultirobotCoordinate::cmultirobotCoordinate(vector<TaskTemplate> tmpCoorTEQ, vector<TaskTemplate> currentQueue, int coorLength, int robot_no) : 
CoorTEQ(tmpCoorTEQ), CurrentQueue(currentQueue), CoorLength(coorLength), Robot_No(robot_no)
{
    CoordinateTask = -1;
    MaxTaskCoorValue = 0;
}

/*
 * 任务协调
 */
void cmultirobotCoordinate::taskCoordinate()
{
    MaxTaskCoorValue = calTaskCoorUtility(CoorTEQ) + calTaskCoorCurrentUtility(CurrentQueue);
    for(int i = 1; i <= CoorLength; ++i)
    {
        vector<TaskTemplate> tmpCoorTEQ = CoorTEQ;
        vector<TaskTemplate> tmpCurrentTEQ = CurrentQueue;
        TaskTemplate tmpTask = *(tmpCoorTEQ.end() - 1 - i);
        *(tmpCoorTEQ.end() - 1 - i) = *(CurrentQueue.end() - 1);
        *(tmpCurrentTEQ.end() - 1) = tmpTask;
        float TaskCoorValue = calTaskCoorUtility(tmpCoorTEQ) + calTaskCoorCurrentUtility(tmpCurrentTEQ);
        if (TaskCoorValue > MaxTaskCoorValue)
        {
            MaxTaskCoorValue = TaskCoorValue;
            CoordinateTask = i;
        }
    }

    if (CoordinateTask != -1)
    {
        cout << "机器人:" << Robot_No << "协调的任务序号:" << CoordinateTask << endl;         
    }
    else
    {
        cout << "机器人:" << Robot_No << "不进行协调" << endl;
    }
       
}

/*
 * 计算机器人任务协调效用（协调机器人）
 */
float cmultirobotCoordinate::calTaskCoorUtility(vector<TaskTemplate> tmpCoorTEQ)
{
    vector<TaskTemplate> tmp = tmpCoorTEQ;
    float CoorValue1 = 0;
    for(int i = 1; i <= CoorLength; ++i)
    {
        CoorValue1 += sqrt(pow((tmp.end() - 1 - i) -> EndPoint[0] - (tmp.end() - 1 - i) -> BeginPoint[0], 2) +
                pow((tmp.end() - 1 - i) -> EndPoint[1] - (tmp.end() - 1 - i) -> BeginPoint[1], 2)) +
                sqrt(pow((tmp.end() - 1 - i) -> EndPoint[0] - (tmp.end() - 1 - (i - 1)) -> BeginPoint[0], 2) +
                pow((tmp.end() - 1 - i) -> EndPoint[1] - (tmp.end() - 1 - (i - 1)) -> BeginPoint[1], 2));        
    }

    float CoorValue2 = sqrt(pow((tmp.end() - 1 - (CoorLength + 1)) -> EndPoint[0] - (tmp.end() - 1 - CoorLength) -> BeginPoint[0], 2) +
                pow((tmp.end() - 1 - (CoorLength + 1)) -> EndPoint[1] - (tmp.end() - 1 - CoorLength) -> BeginPoint[1], 2));

    float result = 1 / (CoorValue1 + CoorValue2);

    return result;
}

/*
 * 计算机器人任务协调效用(当前机器人)
 */
float cmultirobotCoordinate::calTaskCoorCurrentUtility(vector<TaskTemplate> tmpCoorCurrentTEQ)
{
    vector<TaskTemplate> tmp = tmpCoorCurrentTEQ;
    float CoorValue = sqrt(pow((tmp.end() - 2) -> EndPoint[0] - (tmp.end() - 1) -> BeginPoint[0], 2) + 
            pow((tmp.end() - 2) -> EndPoint[1] - (tmp.end() - 1) -> BeginPoint[1], 2)) + 
            sqrt(pow((tmp.end() - 1) -> EndPoint[0] - (tmp.end() - 1) -> BeginPoint[0], 2) +
            pow((tmp.end() - 1) -> EndPoint[1] - (tmp.end() - 1) -> BeginPoint[1], 2));

    float result = 1 / CoorValue;
    return result;
}