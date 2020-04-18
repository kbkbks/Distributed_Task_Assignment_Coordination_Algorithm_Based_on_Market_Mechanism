#ifndef _CMULTIROBOTCOORDINATE_H_
#define _CMULTIROBOTCOORDINATE_H_

#include "define.h"

class cmultirobotCoordinate
{
public:
    /*
     * 构造函数
     */
    cmultirobotCoordinate(vector<TaskTemplate> tmpCoorTEQ, vector<TaskTemplate> currentQueue, int coorLength, int robot_no);

    /*
     * 析构函数
     */
    ~cmultirobotCoordinate() {}

    /*
     * 任务协调
     */
    void taskCoordinate();

    /*
    * 计算机器人任务协调效用（协调机器人）
    */
    float calTaskCoorUtility(vector<TaskTemplate> tmpCoorTEQ);
    
    /*
     * 计算机器人任务协调效用(当前机器人)
     */
    float calTaskCoorCurrentUtility(vector<TaskTemplate> tmpCoorCurrentTEQ);

private:
    int Robot_No;   //机器人编号
    const vector<TaskTemplate> CoorTEQ;    //协调对象的任务执行队列
    const vector<TaskTemplate> CurrentQueue;  //当前机器人任务执行队列
    int CoorLength;   //通信协调长度
    int CoordinateTask; //协调对象TEQ的协调任务序号(最靠近当前分配任务的序号为1，最远为CoorLength)
    vector<TaskTemplate> NewCoorTEQ;    //协调后的对象任务执行队列
    vector<TaskTemplate> NewCurrentQueue;   //协调后的当前机器人任务执行队列
    float MaxTaskCoorValue; //协调对象最大任务协调效用

    vector<float> AllValue;   //所有CoorTEQ的任务协调效用



};

#endif // ! _CMULTIROBOTCOORDINATE_H_