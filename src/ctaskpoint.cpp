/*
 # Copyright (c) 2019-2020 Xinyan Han. All rights reserved.
 */
#include "ctaskpoint.h"

/*
 * 向任务发布点传递所有初始化参数
 */
void ctaskpoint::setInitialValue(int pointnum, float bp[][2], float ep[][2]) {
    for (int i = 0; i < TASKCAPACITY; i++) {
        TaskRepository[i].TaskNo = i;
        TaskRepository[i].PointNo = pointnum;
        TaskRepository[i].BeginPoint[0] = bp[i][0];
        TaskRepository[i].BeginPoint[1] = bp[i][1];
        TaskRepository[i].EndPoint[0] = ep[i][0];
        TaskRepository[i].EndPoint[1] = ep[i][1];
        TaskRepository[i].TaskLoad = RAND_TASKLOAD;
        TaskRepository[i].TaskExeProgress = 0;
        TaskRepository[i].TaskExecutedFlag = 0;
    }
}

/*
 * 打印任务点存放的任务信息
 */
void ctaskpoint::printTaskRepository() {
    for (int i = 0; i < TASKCAPACITY; i++) {
        cout << "任务发布点存放的任务信息：" << endl;
        cout << "任务编号（任务发布点中）：" << TaskRepository[i].TaskNo << " "
            << "任务发布点编号：" << TaskRepository[i].PointNo << " "
            << "任务起点：" << TaskRepository[i].BeginPoint[0] << "," << TaskRepository[i].BeginPoint[1] << " "
            << "任务终点：" << TaskRepository[i].EndPoint[0] << "," << TaskRepository[i].EndPoint[1] << " "
            << "任务负载：" << TaskRepository[i].TaskLoad
            << endl;
    }
}

/*
 * 向外发送一个任务
 */
TaskTemplate ctaskpoint::sendTask(int i) {
    return TaskRepository[i];
}
