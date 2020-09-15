/*
 # Copyright (c) 2019-2020 Xinyan Han. All rights reserved.
 */
#include "ctasklist.h"

/*
 * 构造函数
 */
ctasklist::ctasklist():TaskNumber(0) { }

/*
 * 从任务发布点接收任务
 */
void ctasklist::getTask(ctaskpoint& taskpoint, int i) {
    TaskQueue.push_back(taskpoint.sendTask(i));
    TaskNumber++;
    // printTaskList();
}

/*
 * 打印任务列表信息
 */
void ctasklist::printTaskList() {
    cout << "在任务列表中有" << " " << TaskNumber << " " << "个任务，" << " "
        << "任务点编号为：" << TaskQueue.back().PointNo << " "
        << "任务点中的任务编号为：" << TaskQueue.back().TaskNo << " "
        << "任务起点" << TaskQueue.back().BeginPoint[0] << "," << TaskQueue.back().BeginPoint[1] << " "
        << "任务终点" << TaskQueue.back().EndPoint[0] << "," << TaskQueue.back().EndPoint[1] <<  " "
        << endl;
}

/*
 * 发送任务列表中的任务数量
 */
int ctasklist::sendTaskNumber() {
    return TaskNumber;
}

/*
 * 发送任务列表中的任务
 */
TaskTemplate * ctasklist::sendTaskQueue(int i) {
    return &TaskQueue[i];
}

/*
 * 从旧任务列表接收任务
 */
void ctasklist::getTask(TaskTemplate * residultask) {
    TaskQueue.push_back(*residultask);
    TaskNumber++;
    // printTaskList();
}
