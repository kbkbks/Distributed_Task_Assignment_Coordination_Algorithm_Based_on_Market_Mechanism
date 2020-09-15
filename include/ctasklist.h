/*
 # Copyright (c) 2019-2020 Xinyan Han. All rights reserved.
 */
#ifndef  CTASKLIST_H_
#define  CTASKLIST_H_
#include "define.h"
#include "ctaskpoint.h"

class ctasklist {
 public:
    ctasklist();
    ~ctasklist() {}

    /*
     * 从任务发布点接收任务
     */
    void getTask(ctaskpoint& taskpoint, int i);

    /*
     * 打印任务列表信息
     */
    void printTaskList();

    /*
     * 发送任务列表中的任务数量
     */
    int sendTaskNumber();

    /*
     * 发送任务列表中的任务
     */
    TaskTemplate * sendTaskQueue(int i);

    /*
     * 从旧任务列表接收任务
     */
    void getTask(TaskTemplate * residultask);

 private:
    int TaskNumber;  // 任务数量(任务列表中)
    vector<TaskTemplate> TaskQueue;  // 任务列表
};

#endif  // CTASKLIST_H_
