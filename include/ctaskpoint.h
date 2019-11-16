#ifndef _CTASKPOINT_H_
#define _CTASKPOINT_H_

#include "define.h"

class ctaskpoint
{
public:
    ctaskpoint() {}
    ~ctaskpoint() {}

    /*
     * 向任务发布点传递所有初始化参数
     */
    void setInitialValue(int pointnum, float bp[][2], float [][2]);

    /*
     * 打印任务点存放的任务信息
     */
    void printTaskRepository();

    /*
     * 向外发送一个任务
     */
    TaskTemplate sendTask(int i);

private:
    TaskTemplate * TaskRepository = new TaskTemplate[TASKCAPACITY];	//任务发布点存放的任务集合
};

#endif  // !_CTASKPOINT_H_