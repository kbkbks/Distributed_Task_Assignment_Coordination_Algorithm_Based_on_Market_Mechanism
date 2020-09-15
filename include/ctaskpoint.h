/*
 # Copyright (c) 2019-2020 Xinyan Han. All rights reserved.
 */
#ifndef CTASKPOINT_H_
#define CTASKPOINT_H_

#include "define.h"

class ctaskpoint {
 public:
    ctaskpoint() {}
    ~ctaskpoint() {
        delete[] TaskRepository;
        // cout << "taskpoint析构" << endl;
    }

    /*
     * 向任务发布点传递所有初始化参数
     */
    void setInitialValue(int pointnum, float bp[][2], float ep[][2], int dem[ATOMICLENGTH]);

    /*
     * 打印任务点存放的任务信息
     */
    void printTaskRepository();

    /*
     * 向外发送一个任务
     */
    TaskTemplate sendTask(int i);

 private:
    TaskTemplate * TaskRepository = new TaskTemplate[TASKCAPACITY];  // 任务发布点存放的任务集合
};

#endif  // CTASKPOINT_H_
