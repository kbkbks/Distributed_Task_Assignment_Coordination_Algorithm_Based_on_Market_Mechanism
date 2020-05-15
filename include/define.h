#ifndef _DEFINE_H_
#define _DEFINE_H_

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <future>
#include <assert.h>
#include <math.h>
#include <mutex>
#include <sys/time.h>
#include <chrono>
#include <algorithm>
#include <pthread.h>
#include <sys/syscall.h>

#define ROBOTNUM 6  //机器人数量
#define TASKPOINT 8 //任务点数量
#define TASKCAPACITY 50  //任务点任务容量
#define COORDINATE_LENGTH 3 //协调长度
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

//竞拍算法互斥量
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
extern mutex TEQrw0_1; //互斥量，线程阻塞锁，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern mutex TEQrw1_2; //互斥量，线程阻塞锁，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern mutex TEQrw2_3; //互斥量，线程阻塞锁，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern mutex TEQrw3_4; //互斥量，线程阻塞锁，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern mutex TEQrw4_5; //互斥量，线程阻塞锁，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读
extern mutex TEQrw1_0; //互斥量，线程阻塞锁，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern mutex TEQrw2_1; //互斥量，线程阻塞锁，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern mutex TEQrw3_2; //互斥量，线程阻塞锁，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern mutex TEQrw4_3; //互斥量，线程阻塞锁，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern mutex TEQrw5_4; //互斥量，线程阻塞锁，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

//多机器人协调状态互斥量
extern mutex muCoorStatus; //互斥量，协调状态CoorStatus读写

//多机器人协调条件变量
extern condition_variable conVAR0_1;   //条件变量，用于线程阻塞，Robot[0]进行协调，Robot[0]向Robot[1]交换任务
extern condition_variable conVAR1_2;   //条件变量，用于线程阻塞，Robot[1]进行协调，Robot[1]向Robot[2]交换任务
extern condition_variable conVAR2_3;   //条件变量，用于线程阻塞，Robot[2]进行协调，Robot[2]向Robot[3]交换任务
extern condition_variable conVAR3_4;   //条件变量，用于线程阻塞，Robot[3]进行协调，Robot[3]向Robot[4]交换任务
extern condition_variable conVAR4_5;   //条件变量，用于线程阻塞，Robot[4]进行协调，Robot[4]向Robot[5]交换任务
extern condition_variable conVAR1_0;   //条件变量，用于线程阻塞，Robot[1]进行协调，Robot[1]向Robot[0]交换任务
extern condition_variable conVAR2_1;   //条件变量，用于线程阻塞，Robot[2]进行协调，Robot[2]向Robot[1]交换任务
extern condition_variable conVAR3_2;   //条件变量，用于线程阻塞，Robot[3]进行协调，Robot[3]向Robot[2]交换任务
extern condition_variable conVAR4_3;   //条件变量，用于线程阻塞，Robot[4]进行协调，Robot[4]向Robot[3]交换任务
extern condition_variable conVAR5_4;   //条件变量，用于线程阻塞，Robot[5]进行协调，Robot[5]向Robot[4]交换任务

//多机器人协调写读全局标志位
extern bool GloConFlag0_1; //全局标志，Robot[0]进行协调，Robot[0]向Robot[1]交换任务，Robot[1]任务执行队列已存入
extern bool GloConFlag1_2; //全局标志，Robot[1]进行协调，Robot[1]向Robot[2]交换任务，Robot[2]任务执行队列已存入
extern bool GloConFlag2_3; //全局标志，Robot[2]进行协调，Robot[2]向Robot[3]交换任务，Robot[3]任务执行队列已存入
extern bool GloConFlag3_4; //全局标志，Robot[3]进行协调，Robot[3]向Robot[4]交换任务，Robot[4]任务执行队列已存入
extern bool GloConFlag4_5; //全局标志，Robot[4]进行协调，Robot[4]向Robot[5]交换任务，Robot[5]任务执行队列已存入
extern bool GloConFlag1_0; //全局标志，Robot[1]进行协调，Robot[1]向Robot[0]交换任务，Robot[0]任务执行队列已存入
extern bool GloConFlag2_1; //全局标志，Robot[2]进行协调，Robot[2]向Robot[1]交换任务，Robot[1]任务执行队列已存入
extern bool GloConFlag3_2; //全局标志，Robot[3]进行协调，Robot[3]向Robot[2]交换任务，Robot[2]任务执行队列已存入
extern bool GloConFlag4_3; //全局标志，Robot[4]进行协调，Robot[4]向Robot[3]交换任务，Robot[3]任务执行队列已存入
extern bool GloConFlag5_4; //全局标志，Robot[5]进行协调，Robot[5]向Robot[4]交换任务，Robot[4]任务执行队列已存入

//多机器人协调全局任务执行队列
extern vector<TaskTemplate> GlobalTEQ0_1;  //全局任务执行队列，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern vector<TaskTemplate> GlobalTEQ1_2;  //全局任务执行队列，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern vector<TaskTemplate> GlobalTEQ2_3;  //全局任务执行队列，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern vector<TaskTemplate> GlobalTEQ3_4;  //全局任务执行队列，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern vector<TaskTemplate> GlobalTEQ4_5;  //全局任务执行队列，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读
extern vector<TaskTemplate> GlobalTEQ1_0;  //全局任务执行队列，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern vector<TaskTemplate> GlobalTEQ2_1;  //全局任务执行队列，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern vector<TaskTemplate> GlobalTEQ3_2;  //全局任务执行队列，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern vector<TaskTemplate> GlobalTEQ4_3;  //全局任务执行队列，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern vector<TaskTemplate> GlobalTEQ5_4;  //全局任务执行队列，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

//多机器人协调NewCoorTEQ刷新标志位
extern bool GloNewCoorTEQFlag0_1; //全局标志，Robot[0]已发送NewCoorTEQ，Robot[1]可读
extern bool GloNewCoorTEQFlag1_2; //全局标志，Robot[1]已发送NewCoorTEQ，Robot[2]可读
extern bool GloNewCoorTEQFlag2_3; //全局标志，Robot[2]已发送NewCoorTEQ，Robot[3]可读
extern bool GloNewCoorTEQFlag3_4; //全局标志，Robot[3]已发送NewCoorTEQ，Robot[4]可读
extern bool GloNewCoorTEQFlag4_5; //全局标志，Robot[4]已发送NewCoorTEQ，Robot[5]可读
extern bool GloNewCoorTEQFlag1_0; //全局标志，Robot[1]已发送NewCoorTEQ，Robot[0]可读
extern bool GloNewCoorTEQFlag2_1; //全局标志，Robot[2]已发送NewCoorTEQ，Robot[1]可读
extern bool GloNewCoorTEQFlag3_2; //全局标志，Robot[3]已发送NewCoorTEQ，Robot[2]可读
extern bool GloNewCoorTEQFlag4_3; //全局标志，Robot[4]已发送NewCoorTEQ，Robot[3]可读
extern bool GloNewCoorTEQFlag5_4; //全局标志，Robot[5]已发送NewCoorTEQ，Robot[4]可读

//多机器人协调NewCoorTEQ读写互斥量
extern mutex NewCoorTEQrw0_1; //互斥量，线程阻塞锁，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern mutex NewCoorTEQrw1_2; //互斥量，线程阻塞锁，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern mutex NewCoorTEQrw2_3; //互斥量，线程阻塞锁，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern mutex NewCoorTEQrw3_4; //互斥量，线程阻塞锁，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern mutex NewCoorTEQrw4_5; //互斥量，线程阻塞锁，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读
extern mutex NewCoorTEQrw1_0; //互斥量，线程阻塞锁，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern mutex NewCoorTEQrw2_1; //互斥量，线程阻塞锁，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern mutex NewCoorTEQrw3_2; //互斥量，线程阻塞锁，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern mutex NewCoorTEQrw4_3; //互斥量，线程阻塞锁，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern mutex NewCoorTEQrw5_4; //互斥量，线程阻塞锁，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

//多机器人协调NewCoorTEQ刷新条件变量
extern condition_variable convarNCQ0_1;    //条件变量，Robot[1]读取Robot[0]发送的NewCoorTEQ
extern condition_variable convarNCQ1_2;    //条件变量，Robot[2]读取Robot[1]发送的NewCoorTEQ
extern condition_variable convarNCQ2_3;    //条件变量，Robot[3]读取Robot[2]发送的NewCoorTEQ
extern condition_variable convarNCQ3_4;    //条件变量，Robot[4]读取Robot[3]发送的NewCoorTEQ
extern condition_variable convarNCQ4_5;    //条件变量，Robot[5]读取Robot[4]发送的NewCoorTEQ
extern condition_variable convarNCQ1_0;    //条件变量，Robot[0]读取Robot[1]发送的NewCoorTEQ
extern condition_variable convarNCQ2_1;    //条件变量，Robot[1]读取Robot[2]发送的NewCoorTEQ
extern condition_variable convarNCQ3_2;    //条件变量，Robot[2]读取Robot[3]发送的NewCoorTEQ
extern condition_variable convarNCQ4_3;    //条件变量，Robot[3]读取Robot[4]发送的NewCoorTEQ
extern condition_variable convarNCQ5_4;    //条件变量，Robot[4]读取Robot[5]发送的NewCoorTEQ

//多机器人协调全局协调状态
extern vector<bool> GlobalCoorStatus;  //全局协调状态



//竞拍算法全局价格
extern vector<float> GlobalPrice0_1;   //全局价格，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern vector<float> GlobalPrice1_2;   //全局价格，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern vector<float> GlobalPrice2_3;   //全局价格，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern vector<float> GlobalPrice3_4;   //全局价格，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern vector<float> GlobalPrice4_5;   //全局价格，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读
extern vector<float> GlobalPrice1_0;   //全局价格，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern vector<float> GlobalPrice2_1;   //全局价格，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern vector<float> GlobalPrice3_2;   //全局价格，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern vector<float> GlobalPrice4_3;   //全局价格，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern vector<float> GlobalPrice5_4;   //全局价格，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

//多机器人协调全局NewCoorTEQ
extern vector<TaskTemplate> GlobalNewCoorTEQ0_1;  //全局NewCoorTEQ，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern vector<TaskTemplate> GlobalNewCoorTEQ1_2;  //全局NewCoorTEQ，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern vector<TaskTemplate> GlobalNewCoorTEQ2_3;  //全局NewCoorTEQ，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern vector<TaskTemplate> GlobalNewCoorTEQ3_4;  //全局NewCoorTEQ，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern vector<TaskTemplate> GlobalNewCoorTEQ4_5;  //全局NewCoorTEQ，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读
extern vector<TaskTemplate> GlobalNewCoorTEQ1_0;  //全局NewCoorTEQ，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern vector<TaskTemplate> GlobalNewCoorTEQ2_1;  //全局NewCoorTEQ，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern vector<TaskTemplate> GlobalNewCoorTEQ3_2;  //全局NewCoorTEQ，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern vector<TaskTemplate> GlobalNewCoorTEQ4_3;  //全局NewCoorTEQ，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern vector<TaskTemplate> GlobalNewCoorTEQ5_4;  //全局NewCoorTEQ，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

//竞拍算法全局竞标者
extern vector<int> GlobalBidder0_1;    //全局竞标者，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern vector<int> GlobalBidder1_2;    //全局竞标者，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern vector<int> GlobalBidder2_3;    //全局竞标者，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern vector<int> GlobalBidder3_4;    //全局竞标者，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern vector<int> GlobalBidder4_5;    //全局竞标者，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读
extern vector<int> GlobalBidder1_0;    //全局竞标者，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern vector<int> GlobalBidder2_1;    //全局竞标者，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern vector<int> GlobalBidder3_2;    //全局竞标者，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern vector<int> GlobalBidder4_3;    //全局竞标者，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern vector<int> GlobalBidder5_4;    //全局竞标者，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

//竞拍算法全局所有价格
extern vector<vector<float>> GlobalAllRobotPrice0_1;   //全局所有机器人价格，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern vector<vector<float>> GlobalAllRobotPrice1_2;   //全局所有机器人价格，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern vector<vector<float>> GlobalAllRobotPrice2_3;   //全局所有机器人价格，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern vector<vector<float>> GlobalAllRobotPrice3_4;   //全局所有机器人价格，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern vector<vector<float>> GlobalAllRobotPrice4_5;   //全局所有机器人价格，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读
extern vector<vector<float>> GlobalAllRobotPrice1_0;   //全局所有机器人价格，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern vector<vector<float>> GlobalAllRobotPrice2_1;   //全局所有机器人价格，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern vector<vector<float>> GlobalAllRobotPrice3_2;   //全局所有机器人价格，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern vector<vector<float>> GlobalAllRobotPrice4_3;   //全局所有机器人价格，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern vector<vector<float>> GlobalAllRobotPrice5_4;   //全局所有机器人价格，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

//竞拍算法全局所有竞标者
extern vector<vector<int>> GlobalAllRobotBidder0_1;    //全局所有机器人竞标者，Robot[0]向Robot[1]写，Robot[1]向Robot[0]读
extern vector<vector<int>> GlobalAllRobotBidder1_2;    //全局所有机器人竞标者，Robot[1]向Robot[2]写，Robot[2]向Robot[1]读
extern vector<vector<int>> GlobalAllRobotBidder2_3;    //全局所有机器人竞标者，Robot[2]向Robot[3]写，Robot[3]向Robot[2]读
extern vector<vector<int>> GlobalAllRobotBidder3_4;    //全局所有机器人竞标者，Robot[3]向Robot[4]写，Robot[4]向Robot[3]读
extern vector<vector<int>> GlobalAllRobotBidder4_5;    //全局所有机器人竞标者，Robot[4]向Robot[5]写，Robot[5]向Robot[4]读
extern vector<vector<int>> GlobalAllRobotBidder1_0;    //全局所有机器人竞标者，Robot[1]向Robot[0]写，Robot[0]向Robot[1]读
extern vector<vector<int>> GlobalAllRobotBidder2_1;    //全局所有机器人竞标者，Robot[2]向Robot[1]写，Robot[1]向Robot[2]读
extern vector<vector<int>> GlobalAllRobotBidder3_2;    //全局所有机器人竞标者，Robot[3]向Robot[2]写，Robot[2]向Robot[3]读
extern vector<vector<int>> GlobalAllRobotBidder4_3;    //全局所有机器人竞标者，Robot[4]向Robot[3]写，Robot[3]向Robot[4]读
extern vector<vector<int>> GlobalAllRobotBidder5_4;    //全局所有机器人竞标者，Robot[5]向Robot[4]写，Robot[4]向Robot[5]读

#endif  // !_DEFINE_H_