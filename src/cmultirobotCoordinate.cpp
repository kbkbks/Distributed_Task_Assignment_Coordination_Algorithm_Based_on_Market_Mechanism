#include "cmultirobotCoordinate.h"

/*
 * 构造函数
 */
cmultirobotCoordinate::cmultirobotCoordinate(vector<TaskTemplate> coorTEQ, vector<TaskTemplate> currentTEQ, int coorLength, int robot_no) : 
CoorTEQ(coorTEQ), CurrentTEQ(currentTEQ), CoorLength(coorLength), Robot_No(robot_no)
{
    CoordinateTask = -1;
    MaxTaskCoorValue = 0;
}

/*
 * 任务协调
 */
void cmultirobotCoordinate::taskCoordinate()
{
    MaxTaskCoorValue = calTaskCoorUtility(CoorTEQ) + calTaskCurrentUtility(CurrentTEQ);
    for(int i = 1; i <= CoorLength; ++i)
    {
        vector<TaskTemplate> tmpCoorTEQ = CoorTEQ;
        vector<TaskTemplate> tmpCurrentTEQ = CurrentTEQ;
        TaskTemplate tmpTask = *(tmpCoorTEQ.end() - 1 - i);
        *(tmpCoorTEQ.end() - 1 - i) = *(CurrentTEQ.end() - 1);
        *(tmpCurrentTEQ.end() - 1) = tmpTask;
        float TaskCoorValue = calTaskCoorUtility(tmpCoorTEQ) + calTaskCurrentUtility(tmpCurrentTEQ);
        if (TaskCoorValue < MaxTaskCoorValue)
        {
            MaxTaskCoorValue = TaskCoorValue;
            CoordinateTask = i; //记录协调任务序号

            //存储新的任务队列
            NewCoorTEQ = tmpCoorTEQ;
            NewCurrentTEQ = tmpCurrentTEQ;
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
    for(int i = 1; i <= CoorLength + 1; ++i)
    {
        // CoorValue1 += sqrt(pow((tmp.end() - 1 - i) -> EndPoint[0] - (tmp.end() - 1 - i) -> BeginPoint[0], 2) +
        //         pow((tmp.end() - 1 - i) -> EndPoint[1] - (tmp.end() - 1 - i) -> BeginPoint[1], 2)) +
        //         sqrt(pow((tmp.end() - 1 - i) -> EndPoint[0] - (tmp.end() - 1 - (i - 1)) -> BeginPoint[0], 2) +
        //         pow((tmp.end() - 1 - i) -> EndPoint[1] - (tmp.end() - 1 - (i - 1)) -> BeginPoint[1], 2)); 

        CoorValue1 += sqrt(pow((tmp.end() - i) -> EndPoint[0] - (tmp.end() - i) -> BeginPoint[0], 2) +
                pow((tmp.end() - i) -> EndPoint[1] - (tmp.end() - i) -> BeginPoint[1], 2)) +
                sqrt(pow((tmp.end() - i) -> EndPoint[0] - (tmp.end() - (i - 1)) -> BeginPoint[0], 2) +
                pow((tmp.end() - i) -> EndPoint[1] - (tmp.end() - (i - 1)) -> BeginPoint[1], 2));     
    }

    //float CoorValue2 = sqrt(pow((tmp.end() - 1 - (CoorLength + 1)) -> EndPoint[0] - (tmp.end() - 1 - CoorLength) -> BeginPoint[0], 2) +
    //            pow((tmp.end() - 1 - (CoorLength + 1)) -> EndPoint[1] - (tmp.end() - 1 - CoorLength) -> BeginPoint[1], 2));

    //float result = 1 / (CoorValue1 + CoorValue2);

    float result = CoorValue1;
    
    cout << "对象机器人:" << Robot_No  << "路程：" << CoorValue1 << endl;
    return result;
}

/*
 * 计算机器人任务协调效用(当前机器人)
 */
float cmultirobotCoordinate::calTaskCurrentUtility(vector<TaskTemplate> tmpCurrentTEQ)
{
    vector<TaskTemplate> tmp = tmpCurrentTEQ;
    float CoorValue = sqrt(pow((tmp.end() - 2) -> EndPoint[0] - (tmp.end() - 1) -> BeginPoint[0], 2) + 
            pow((tmp.end() - 2) -> EndPoint[1] - (tmp.end() - 1) -> BeginPoint[1], 2)) + 
            sqrt(pow((tmp.end() - 1) -> EndPoint[0] - (tmp.end() - 1) -> BeginPoint[0], 2) +
            pow((tmp.end() - 1) -> EndPoint[1] - (tmp.end() - 1) -> BeginPoint[1], 2));

    //float result = 1 / CoorValue;

    float result = CoorValue;

    cout << "当前机器人:" << Robot_No  << "路程：" << CoorValue<< endl;

    return result;
}

/*
 * 返回协调后的协调对象任务队列
 */
vector<TaskTemplate> cmultirobotCoordinate::sendNewCoorTEQ()
{
    if(CoordinateTask != -1)
    {
        return NewCoorTEQ;       
    }
    else
    {
        return CoorTEQ;
    }
}

/*
 * 返回协调后的当前任务队列
 */
vector<TaskTemplate> cmultirobotCoordinate::sendNewCurrentTEQ()
{
    if(CoordinateTask != -1)
    {
        return NewCurrentTEQ;        
    }
    else
    {
        return CurrentTEQ;
    }
}