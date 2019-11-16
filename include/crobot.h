#ifndef _CROBOT_H_
#define _CROBOT_H_

#include "define.h"
#include "ctasklist.h"

class crobot
{
public:
    crobot();
    ~crobot() {}

    /*
     * 向机器人传递所有初始化参数
     */
    void setInitialValue(int r_No, float location_x, float location_y);

    /*
     * 打印机器人相关信息
     */
    void printRobotInfo();

    /*
     * 机器人子线程
     */
    void generateValueList(ctasklist * tasklist, int tasklist_num, float rand_num, promise<crobot> &PromiseRobot);

    /*
     * 机器人计算任务列表某个任务的价值
     */
    void calculateValue(ctasklist * tasklist, int i);

    /*
     * 打印任务价值列表
     */
    void printValueList(int i);

    /*
     * 常规直接计算任务价值(不带机器人自协调)
     */
    void GeneralCalculate(TaskTemplate * TmpTask);

    /*
     * 寻找使插入新任务后整体任务执行队列价值最高的插入点(机器人自协调)
     */
    void SelfCoordination(TaskTemplate * TmpTask);

    /*
     * 计算临时任务执行队列总价值（计算新任务插入执行队列某位置后，新队列的总价值）
     */
    float calculateTmpTaskExecutionQueueValue(TaskTemplate * TmpTask, int position);

    /*
     * 发送任务执行队列总价值
     */
    float sendTaskExecutionQueueValue();

    /*
     * 出价
     * @warning 竞拍算法核心代码，需要增加必要的断言或异常处理保证程序正常运行
     */
    void bidding(int tasklist_num);

    /*
     * 将出价和竞标者写入所有机器人价格和竞标者中
     */
    void setARPandARB(int tasklist_num);

    /*
     * 更新价格
     */
    void updatePrice(int tasklist_num);

    /*
     * 读全局价格，Robot[0]向Robot[1]读数据
     */
    void readPrice0(int tasklist_num);

    /*
     * 读全局价格，Robot[1]向Robot[0]读数据，Robot[1]向Robot[2]读数据
     */
    void readPrice1(int tasklist_num);

     /*
     * 读全局价格，Robot[2]向Robot[1]读数据，Robot[2]向Robot[3]读数据
     */
    void readPrice2(int tasklist_num);

    /*
     * 读全局价格，Robot[3]向Robot[2]读数据，Robot[3]向Robot[4]读数据
     */
    void readPrice3(int tasklist_num);

    /*
     * 读全局价格，Robot[4]向Robot[3]读数据，Robot[4]向Robot[5]读数据
     */
    void readPrice4(int tasklist_num);

    /*
     * 读全局价格，Robot[5]向Robot[4]读数据
     */
    void readPrice5(int tasklist_num);

    /*
     * 设置价格
     */
    void setPrice(int tasklist_num);

    /*
     * 写全局价格，Robot[0]向Robot[1]写数据
     */
    void writePrice0(int tasklist_num);

    /*
     * 写全局价格，Robot[1]向Robot[0]写数据，Robot[1]向Robot[2]写数据
     */
    void writePrice1(int tasklist_num);

    /*
     * 写全局价格，Robot[2]向Robot[1]写数据，Robot[2]向Robot[3]写数据
     */
    void writePrice2(int tasklist_num);

    /*
     * 写全局价格，Robot[3]向Robot[2]写数据，Robot[3]向Robot[4]写数据
     */
    void writePrice3(int tasklist_num);

    /*
     * 写全局价格，Robot[4]向Robot[3]写数据，Robot[4]向Robot[5]写数据
     */
    void writePrice4(int tasklist_num);

    /*
     * 写全局价格，Robot[5]向Robot[4]写数据
     */
    void writePrice5(int tasklist_num);

    /*
     * 竞拍
     */
    void auction(vector<float> PriceOld, int tasklist_num);

    /*
     * 收敛
     */
    void convergence(bool &flag);

    /*
     * 更新机器人位置坐标
     */
    void updadteRobotLocation(ctasklist * tasklist);

    /*
     * 将中标的任务存入机器人任务执行队列
     */
    void savetoTaskExecutionQueue(ctasklist * tasklist);

    /*
     * 打印分配的任务
     */
    void printAssignedTask(ctasklist * tasklist);

    /*
     * 发送分配任务的价值
     */
    float sendAssignedTaskValue();

    /*
     * 发送剩余任务数量
     */
    int sendResidualNum();

    /*
     * 发送剩余任务
     */
    TaskTemplate * sendResidualTask(ctasklist * tasklist, int i);

    /*
     * 重置机器人
     */
    void clearPropertity();

    /*
     * 发送机器人编号
     */
    int sendRobotNum();

private:
    float RobotLocation[2]; //机器人位置
    int Robot_No;   //机器人编号
    vector<float> ValueList;    //任务价值列表
    vector<float> Price;    //任务价格
    vector<int> Bidder; //竞拍机器人，其下标序号为任务序列
    vector<vector<float>> AllRobotPrice;    //所有机器人的价格
    vector<vector<int>> AllRobotBidder; //所有机器人的竞标者
    int AssignedTask;   //机器人竞标的任务
    float eps;  //松弛变量
    vector<int> ResidualTask;   //剩余任务集，下标为任务序列
    int ResidualNum;    //剩余任务数量

    vector<TaskTemplate> TaskExecutionQueue;    //机器人任务执行队列
    vector<float> TaskExecutionQueueValue;  //任务执行队列价值
    int TaskExecutionQueueNum;  //任务执行队列中任务数量
    float maxValue; //价值最大的任务
    int maxValuePosition;   //价值最大的下标
};


#endif  // !_CROBOT_H_