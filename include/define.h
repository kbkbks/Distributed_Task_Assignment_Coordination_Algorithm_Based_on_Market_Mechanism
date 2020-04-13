#ifndef _DEFINE_H_
#define _DEFINE_H_

#include <iostream>
#include <stdio.h>
#include <vector>
#include <thread>
#include <future>
#include <assert.h>
#include <math.h>
#include <mutex>
#include <sys/time.h>
#include <chrono>
#include <algorithm>

#define ROBOTNUM 6  //机器人数量
#define TASKPOINT 8 //任务点数量
#define TASKCAPACITY 5  //任务点任务容量
#define RAND_NUM rand() / double(RAND_MAX) / 100 //随机数， 0～0.01

using namespace std;

/*
 * 任务集合
 * 内有所有任务都具备的信息，包括任务编号，任务点编号，任务起点，任务终点
 */
typedef struct TaskTemplate{
    int TaskNo;    //任务编号(任务点中的任务编号)
    int PointNo;   //任务点编号
    float BeginPoint[2];   //任务起点
    float EndPoint[2];  //任务终点
}TaskTemplate;

extern mutex Mymutex;   //互斥量
extern mutex Mymutex0_1;   //互斥量，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern mutex Mymutex1_2;   //互斥量，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern mutex Mymutex2_3;   //互斥量，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern mutex Mymutex3_4;   //互斥量，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern mutex Mymutex4_5;   //互斥量，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读
extern mutex Mymutex1_0;   //互斥量，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern mutex Mymutex2_1;   //互斥量，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern mutex Mymutex3_2;   //互斥量，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern mutex Mymutex4_3;   //互斥量，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern mutex Mymutex5_4;   //互斥量，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

//多机器人协调互斥量
extern mutex TEQmutex0_1;  //互斥量，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读

extern vector<float> GlobalPrice0_1;   //全局价格，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern vector<float> GlobalPrice1_2;   //全局价格，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern vector<float> GlobalPrice2_3;   //全局价格，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern vector<float> GlobalPrice3_4;   //全局价格，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern vector<float> GlobalPrice4_5;   //全局价格，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读

extern vector<int> GlobalBidder0_1;    //全局竞标者，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern vector<int> GlobalBidder1_2;    //全局竞标者，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern vector<int> GlobalBidder2_3;    //全局竞标者，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern vector<int> GlobalBidder3_4;    //全局竞标者，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern vector<int> GlobalBidder4_5;    //全局竞标者，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读

extern vector<vector<float>> GlobalAllRobotPrice0_1;   //全局所有机器人价格，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern vector<vector<float>> GlobalAllRobotPrice1_2;   //全局所有机器人价格，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern vector<vector<float>> GlobalAllRobotPrice2_3;   //全局所有机器人价格，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern vector<vector<float>> GlobalAllRobotPrice3_4;   //全局所有机器人价格，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern vector<vector<float>> GlobalAllRobotPrice4_5;   //全局所有机器人价格，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读

extern vector<vector<int>> GlobalAllRobotBidder0_1;    //全局所有机器人竞标者，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern vector<vector<int>> GlobalAllRobotBidder1_2;    //全局所有机器人竞标者，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern vector<vector<int>> GlobalAllRobotBidder2_3;    //全局所有机器人竞标者，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern vector<vector<int>> GlobalAllRobotBidder3_4;    //全局所有机器人竞标者，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern vector<vector<int>> GlobalAllRobotBidder4_5;    //全局所有机器人竞标者，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读

extern vector<float> GlobalPrice1_0;   //全局价格，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern vector<float> GlobalPrice2_1;   //全局价格，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern vector<float> GlobalPrice3_2;   //全局价格，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern vector<float> GlobalPrice4_3;   //全局价格，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern vector<float> GlobalPrice5_4;   //全局价格，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

extern vector<int> GlobalBidder1_0;    //全局竞标者，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern vector<int> GlobalBidder2_1;    //全局竞标者，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern vector<int> GlobalBidder3_2;    //全局竞标者，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern vector<int> GlobalBidder4_3;    //全局竞标者，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern vector<int> GlobalBidder5_4;    //全局竞标者，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

extern vector<vector<float>> GlobalAllRobotPrice1_0;   //全局所有机器人价格，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern vector<vector<float>> GlobalAllRobotPrice2_1;   //全局所有机器人价格，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern vector<vector<float>> GlobalAllRobotPrice3_2;   //全局所有机器人价格，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern vector<vector<float>> GlobalAllRobotPrice4_3;   //全局所有机器人价格，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern vector<vector<float>> GlobalAllRobotPrice5_4;   //全局所有机器人价格，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

extern vector<vector<int>> GlobalAllRobotBidder1_0;    //全局所有机器人竞标者，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern vector<vector<int>> GlobalAllRobotBidder2_1;    //全局所有机器人竞标者，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern vector<vector<int>> GlobalAllRobotBidder3_2;    //全局所有机器人竞标者，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern vector<vector<int>> GlobalAllRobotBidder4_3;    //全局所有机器人竞标者，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern vector<vector<int>> GlobalAllRobotBidder5_4;    //全局所有机器人竞标者，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

#endif  // !_DEFINE_H_