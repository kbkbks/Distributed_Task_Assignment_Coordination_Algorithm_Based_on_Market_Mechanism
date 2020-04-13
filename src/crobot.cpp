#include "crobot.h"

/*
 * 构造函数
 */
crobot::crobot():AllRobotPrice(ROBOTNUM), AllRobotBidder(ROBOTNUM), TaskExecutionQueueNum(0), CoorCommunicateLength(0), CoorCommunicateTime(0)
{

}

/*
 * 向机器人传递所有初始化参数
 */
void crobot::setInitialValue(int r_No, float location_x, float location_y)
{
    Robot_No = r_No;
    RobotLocation[0] = location_x;
    RobotLocation[1] = location_y;
 }

/*
 * 打印机器人相关信息
 */
void crobot::printRobotInfo()
{
    cout << "机器人编号：" << Robot_No << " "
        << "机器人位置：" << RobotLocation[0] << "," << RobotLocation[1] << " "
        << endl;
} 

/*
 * 机器人子线程
 */
void crobot::generateValueList(ctasklist * tasklist, int tasklist_num, float rand_num, promise<crobot> &PromiseRobot)
{
    cout << "子线程启动，线程ID：" << this_thread::get_id() << endl;
    float random_num = RAND_NUM;
    //cout << random_num << endl;

    //生成价值列表
    for (int i = 0; i < tasklist_num; i++)
    {
        Mymutex.lock();
        calculateValue(tasklist, i);
        Mymutex.unlock();
    }

    //局部变量赋值
    vector<float> PriceOld; //旧价格
    eps = 0.01 + rand_num;   //松弛变量，加上随机数，每个机器人不同
    bool flag = false;
    AssignedTask = -1;

    //局部价格，竞标者赋值
    for (int i = 0; i < tasklist_num; i++)
    {
        Price.push_back(0);
        PriceOld.push_back(0);
        Bidder.push_back(-1);
    }

    //所有机器人价格赋值
    //所有机器人竞标者赋值
    for (int i = 0; i < ROBOTNUM; i++)
    {
        for (int j = 0; j < tasklist_num; j++)
        {
            AllRobotPrice[i].push_back(0);
            AllRobotBidder[i].push_back(-1);
        }
    }

    //更新价格，竞拍出价，发送价格
    while(flag == false)
    {
        //更新价格
        updatePrice(tasklist_num);

        //竞标出价
        auction(PriceOld, tasklist_num);

        //设置价格
        setPrice(tasklist_num);

        //收敛
        convergence(flag);

        //保存历史价格
        for (int i = 0; i < tasklist_num; i++)
        {
            PriceOld[i] = Price[i];
        }
    }

    //更新机器人位置坐标
    //updadteRobotLocation(tasklist);

    //将中标的任务存入机器人任务执行队列
    savetoTaskExecutionQueue(tasklist);
    TaskExecutionQueueNum = TaskExecutionQueue.size();

    //多机器人任务协调策略（多线程单个机器人，完全分布式策略）


    /*
     * 以下整理剩余任务这部分代码从逻辑上讲应该归入主函数，在主线程中执行，不应该在子线程中执行，
     * 因此这一部分代码需要重构
     */
    //整理剩余任务
    for (int i = 0; i < tasklist_num; i++)
    {
        if (Bidder[i] == -1)
            //如果任务未被机器人分配
        {
            ResidualTask.push_back(i);
        }
    }
    ResidualNum = ResidualTask.size();

    PromiseRobot.set_value(*this);
}

/*
 * 机器人计算任务列表某个任务的价值
 */
void crobot::calculateValue(ctasklist * tasklist, int i)
{
    TaskTemplate * TmpTask;   //待计算价值的任务
    TmpTask = tasklist->sendTaskQueue(i);   //取任务列表中的第i个任务，别名为TmpTask

    //常规直接计算任务价值
    //GeneralCalculate(TmpTask);
    //printValueList(i);
    
    //寻找使插入新任务后整体任务执行队列价值最高的插入点(机器人自协调)
    SelfCoordination(TmpTask);
}

/*
 * 打印任务价值列表
 */
void crobot::printValueList(int i)
{
    cout << "机器人编号：" << Robot_No << " "
        << "任务" << i << "的价值为：" << ValueList[i]
        << endl;
}

/*
 * 常规直接计算任务价值(不带机器人自协调)
 */
void crobot::GeneralCalculate(TaskTemplate * TmpTask)
{
    float Distance; //机器人完成任务的路程
    float Value;    //任务价值
    
    Distance = sqrt(pow(TmpTask->BeginPoint[0] - RobotLocation[0], 2) + pow(TmpTask->BeginPoint[1] - RobotLocation[1], 2)) +
        sqrt(pow(TmpTask->EndPoint[0] - TmpTask->BeginPoint[0], 2) + pow(TmpTask->EndPoint[1] - TmpTask->BeginPoint[1], 2));
    Value = 1 / Distance;
    ValueList.push_back(Value);
}

/*
 * 寻找使插入新任务后整体任务执行队列价值最高的插入点(机器人自协调)
 */
void crobot::SelfCoordination(TaskTemplate * TmpTask)
{
    float DiffPositionAllValue = 0;  //不同位置上的任务总价值
    vector<float> DiffPositionNetValue; //不同位置上的新任务净价值
    maxValue = 0;
    maxValuePosition = -1;

    if (TaskExecutionQueueNum != 0)
        //如果任务执行队列的任务数量非0
    {
        for (int i = 0; i < TaskExecutionQueueNum + 1; i++)
        {
            DiffPositionAllValue = calculateTmpTaskExecutionQueueValue(TmpTask, i);
            DiffPositionNetValue.push_back(DiffPositionAllValue - sendTaskExecutionQueueValue());
        }
    }
    else
        //如果任务执行队列的任务数量为0
    {
        DiffPositionAllValue = calculateTmpTaskExecutionQueueValue(TmpTask, 0);
        DiffPositionNetValue.push_back(DiffPositionAllValue);
    }

    //选择最大的价值以及对应的下标（位置）
    maxValue = *max_element(DiffPositionNetValue.begin(), DiffPositionNetValue.end());
    //下标表示，新竞标的任务安插在第i个任务之后（下标为0表示，新竞标的任务安插在头位置）
    maxValuePosition = max_element(DiffPositionNetValue.begin(), DiffPositionNetValue.end()) - DiffPositionNetValue.begin();
    ValueList.push_back(maxValue);
}

/*
 * 计算临时任务执行队列总价值（计算新任务插入执行队列某位置后，新队列的总价值）
 */
float crobot::calculateTmpTaskExecutionQueueValue(TaskTemplate * TmpTask, int position)
{
    vector<TaskTemplate> TmpTaskExecutionQueue = TaskExecutionQueue;    //定义临时任务执行队列
    TmpTaskExecutionQueue.insert(TmpTaskExecutionQueue.begin() + position, *TmpTask);    //新任务插入临时任务执行队列
    float TmpTaskExecutionQueueNum;
    float tmpValue = 0;
    float tmpDistance = 0;
    TmpTaskExecutionQueueNum = TmpTaskExecutionQueue.size();
    tmpDistance = sqrt(pow(TmpTaskExecutionQueue[0].BeginPoint[0] - RobotLocation[0], 2) + 
        pow(TmpTaskExecutionQueue[0].BeginPoint[1] - RobotLocation[1], 2)) +
        sqrt(pow(TmpTaskExecutionQueue[0].EndPoint[0] - TmpTaskExecutionQueue[0].BeginPoint[0], 2) +
        pow(TmpTaskExecutionQueue[0].EndPoint[1] - TmpTaskExecutionQueue[0].BeginPoint[1], 2));
    tmpValue += 1 / tmpDistance;    //累积价值
    for (int i = 1; i < TmpTaskExecutionQueueNum; i++)
    {
        tmpDistance = sqrt(pow(TmpTaskExecutionQueue[i].BeginPoint[0] - TmpTaskExecutionQueue[i - 1].EndPoint[0], 2) + 
            pow(TmpTaskExecutionQueue[i].BeginPoint[1] - TmpTaskExecutionQueue[i - 1].EndPoint[1], 2)) +
            sqrt(pow(TmpTaskExecutionQueue[i].EndPoint[0] - TmpTaskExecutionQueue[i].BeginPoint[0], 2) +
            pow(TmpTaskExecutionQueue[i].EndPoint[1] - TmpTaskExecutionQueue[i].BeginPoint[1], 2)); 
        tmpValue += 1 / tmpDistance;    //累积价值
    }

    return tmpValue;
}

/*
 * 发送任务执行队列总价值
 */
float crobot::sendTaskExecutionQueueValue()
{
    float tmpValue = 0;
    float tmpDistance = 0;
    tmpDistance = sqrt(pow(TaskExecutionQueue[0].BeginPoint[0] - RobotLocation[0], 2) + 
        pow(TaskExecutionQueue[0].BeginPoint[1] - RobotLocation[1], 2)) +
        sqrt(pow(TaskExecutionQueue[0].EndPoint[0] - TaskExecutionQueue[0].BeginPoint[0], 2) +
        pow(TaskExecutionQueue[0].EndPoint[1] - TaskExecutionQueue[0].BeginPoint[1], 2));
    tmpValue += 1 / tmpDistance;    //累积价值
    for (int i = 1; i < TaskExecutionQueueNum; i++)
    {
        tmpDistance = sqrt(pow(TaskExecutionQueue[i].BeginPoint[0] - TaskExecutionQueue[i - 1].EndPoint[0], 2) + 
            pow(TaskExecutionQueue[i].BeginPoint[1] - TaskExecutionQueue[i - 1].EndPoint[1], 2)) +
            sqrt(pow(TaskExecutionQueue[i].EndPoint[0] - TaskExecutionQueue[i].BeginPoint[0], 2) +
            pow(TaskExecutionQueue[i].EndPoint[1] - TaskExecutionQueue[i].BeginPoint[1], 2)); 
        tmpValue += 1 / tmpDistance;    //累积价值
    }

    return tmpValue;
}

/*
 * 出价
 * @warning 竞拍算法核心代码，需要增加必要的断言或异常处理确保竞拍过程正常运行
 */
void crobot::bidding(int tasklist_num)
{
    float NetMax = 0;   //重置最大净值
    float NetSecMax = 0;    //重置次大净值
    vector<float> Net;  //净值
    float AssignedPrice = 0;    //机器人竞标任务增加的出价

    //初始化净值
    for (int i = 0; i < tasklist_num; i++)
    {
        Net.push_back(0);
    }

    /*
     * @remarks for(int i = 0; ···)，下标访问不能直接改为迭代器访问 for(auto iter = ; ···)。
     * 可以将bidding中的多种有关联的vector(Net,ValueList,Price)结构化，但Net目前不属于类成员变量。这样子便可以迭代器访问。
     */
    //首先选择净值最大的任务进行出价
    for (int i = 0; i < tasklist_num; i++)
    {
        Net[i] = ValueList[i] - Price[i];
        if (NetMax < Net[i])
        {
            NetMax = Net[i];    //最大的净值
            AssignedTask = i;   //选择竞标的任务
        }
    }

    /*
     * 可直接改为迭代器访问
     */
    //选择净值第二大的任务
    for (int i = 0; i < tasklist_num; i++)
    {
        if (i != AssignedTask)
            //非最大的净值
        {
            if (NetSecMax < Net[i])
            {
                NetSecMax = Net[i]; //次大的净值
            }
        }
    }

    //出价，设置新竞标价格和新竞标者
    AssignedPrice = NetMax - NetSecMax + eps;
    Price[AssignedTask] = AssignedPrice + Price[AssignedTask];
    Bidder[AssignedTask] = Robot_No;

    //将出价和竞标者写入所有机器人价格，竞标者中
    setARPandARB(tasklist_num);
}

/*
 * 将出价和竞标者写入所有机器人价格和竞标者中
 */
void crobot::setARPandARB(int tasklist_num)
{
    for (int i = 0; i < tasklist_num; i++)
    {
        AllRobotPrice[Robot_No][i] = Price[i];
        AllRobotBidder[Robot_No][i] = Bidder[i];
    }
}

/*
 * 更新价格
 */
void crobot::updatePrice(int tasklist_num)
{
    //选择每个机器人子线程对应的全局价格读出函数
    switch (Robot_No)
    {
    case 0:
        readPrice0(tasklist_num);
        break;
    case 1:
        readPrice1(tasklist_num);
        break;
    case 2:
        readPrice2(tasklist_num);
        break;
    case 3:
        readPrice3(tasklist_num);
        break;
    case 4:
        readPrice4(tasklist_num);
        break;
    case 5:
        readPrice5(tasklist_num);
        break;
    default:
        break;
    }
}

/*
 * 读全局价格，Robot[0]向Robot[1]读数据
 */
void crobot::readPrice0(int tasklist_num)
{
    Mymutex1_0.lock();
    //读取全局价格GlobalPrice1_0
    if (!GlobalPrice1_0.empty())
        //全局价格GlobalPrice1_0非空
    {
        if (!Price.empty())
            //局部价格Price非空
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                if (Price[i] < GlobalPrice1_0[i])
                {
                    Price[i] = GlobalPrice1_0[i];
                    Bidder[i] = GlobalBidder1_0[i];
                }
            }
            if (AssignedTask != -1)
            {
                if (Price[AssignedTask] == GlobalPrice1_0[AssignedTask])
                {
                    Bidder[AssignedTask] = GlobalBidder1_0[AssignedTask];
                }
            }
        }
        else
            //局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                Price.push_back(GlobalPrice1_0[i]);
                Bidder.push_back(GlobalBidder1_0[i]);
            }
        }
    }
    else
        //全局价格GlobalPrice1_0为空
    {
        //cout << "全局价格GlobalPrice1_0为空" << endl;
    }
    
    //读取全局所有机器人价格GlobalAllRobotPrice1_0
    if (!GlobalAllRobotPrice1_0.empty())
        //全局所有机器人价格GlobalAllRobotPrice1_0非空
    {
        if (!AllRobotPrice.empty())
            //局部所有机器人价格非空
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice1_0[i][j])
                    {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice1_0[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder1_0[i][j];
                    }
                }
            }
        }
        else
            //局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice1_0[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice1_0为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        cout << "全局所有机器人价格GlobalAllRobotPrice1_0为空" << endl;
    }

    Mymutex1_0.unlock();
}

/*
 * 读全局价格，Robot[1]向Robot[0]读数据，Robot[1]向Robot[2]读数据
 */
void crobot::readPrice1(int tasklist_num)
{
    Mymutex0_1.lock();
    Mymutex2_1.lock();

    //读取全局价格GlobalPrice0_1
    if (!GlobalPrice0_1.empty())
        //全局价格GlobalPrice0_1非空
    {
        if (!Price.empty())
            //局部价格Price非空
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                if (Price[i] < GlobalPrice0_1[i])
                {
                    Price[i] = GlobalPrice0_1[i];
                    Bidder[i] = GlobalBidder0_1[i];
                }
            }
            if (AssignedTask != -1)
            {
                if (Price[AssignedTask] == GlobalPrice0_1[AssignedTask])
                {
                    Bidder[AssignedTask] = GlobalBidder0_1[AssignedTask];
                }
            }
        }
        else
            //局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                Price.push_back(GlobalPrice0_1[i]);
                Bidder.push_back(GlobalBidder0_1[i]);
            }
        }
    }
    else
        //全局价格GlobalPrice0_1为空
    {
        //cout << "全局价格GlobalPrice0_1为空" << endl;
    }

    //读取全局价格GlobalPrice2_1
    if (!GlobalPrice2_1.empty())
        //全局价格GlobalPrice2_1非空
    {
        if (!Price.empty())
            //局部价格Price非空
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                if (Price[i] < GlobalPrice2_1[i])
                {
                    Price[i] = GlobalPrice2_1[i];
                    Bidder[i] = GlobalBidder2_1[i];
                }
            }
            if (AssignedTask != -1)
            {
                if (Price[AssignedTask] == GlobalPrice2_1[AssignedTask])
                {
                    Bidder[AssignedTask] = GlobalBidder2_1[AssignedTask];
                }
            }
        }
        else
            //局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                Price.push_back(GlobalPrice2_1[i]);
                Bidder.push_back(GlobalBidder2_1[i]);
            }
        }
    }
    else
        //全局价格GlobalPrice2_1为空
    {
        //cout << "全局价格GlobalPrice2_1为空" << endl;
    }
    
    //读取全局所有机器人价格GlobalAllRobotPrice0_1
    if (!GlobalAllRobotPrice0_1.empty())
        //全局所有机器人价格GlobalAllRobotPrice0_1非空
    {
        if (!AllRobotPrice.empty())
            //局部所有机器人价格非空
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice0_1[i][j])
                    {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice0_1[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder0_1[i][j];
                    }
                }
            }
        }
        else
            //局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice0_1[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice0_1为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        cout << "全局所有机器人价格GlobalAllRobotPrice0_1为空" << endl;
    }    

    //读取全局所有机器人价格GlobalAllRobotPrice2_1
    if (!GlobalAllRobotPrice2_1.empty())
        //全局所有机器人价格GlobalAllRobotPrice2_1非空
    {
        if (!AllRobotPrice.empty())
            //局部所有机器人价格非空
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice2_1[i][j])
                    {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice2_1[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder2_1[i][j];
                    }
                }
            }
        }
        else
            //局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice2_1[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice2_1为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        cout << "全局所有机器人价格GlobalAllRobotPrice2_1为空" << endl;
    }    

    Mymutex0_1.unlock();
    Mymutex2_1.unlock();
}

/*
 * 读全局价格，Robot[2]向Robot[1]读数据，Robot[2]向Robot[3]读数据
 */
void crobot::readPrice2(int tasklist_num)
{
    Mymutex1_2.lock();
    Mymutex3_2.lock();

    //读取全局价格GlobalPrice1_2
    if (!GlobalPrice1_2.empty())
        //全局价格GlobalPrice1_2非空
    {
        if (!Price.empty())
            //局部价格Price非空
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                if (Price[i] < GlobalPrice1_2[i])
                {
                    Price[i] = GlobalPrice1_2[i];
                    Bidder[i] = GlobalBidder1_2[i];
                }
            }
            if (AssignedTask != -1)
            {
                if (Price[AssignedTask] == GlobalPrice1_2[AssignedTask])
                {
                    Bidder[AssignedTask] = GlobalBidder1_2[AssignedTask];
                }
            }
        }
        else
            //局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                Price.push_back(GlobalPrice1_2[i]);
                Bidder.push_back(GlobalBidder1_2[i]);
            }
        }
    }
    else
        //全局价格GlobalPrice1_2为空
    {
        //cout << "全局价格GlobalPrice1_2为空" << endl;
    }

    //读取全局价格GlobalPrice3_2
    if (!GlobalPrice3_2.empty())
        //全局价格GlobalPrice3_2非空
    {
        if (!Price.empty())
            //局部价格Price非空
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                if (Price[i] < GlobalPrice3_2[i])
                {
                    Price[i] = GlobalPrice3_2[i];
                    Bidder[i] = GlobalBidder3_2[i];
                }
            }
            if (AssignedTask != -1)
            {
                if (Price[AssignedTask] == GlobalPrice3_2[AssignedTask])
                {
                    Bidder[AssignedTask] = GlobalBidder3_2[AssignedTask];
                }
            }
        }
        else
            //局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                Price.push_back(GlobalPrice3_2[i]);
                Bidder.push_back(GlobalBidder3_2[i]);
            }
        }
    }
    else
        //全局价格GlobalPrice3_2为空
    {
        //cout << "全局价格GlobalPrice3_2为空" << endl;
    }

    //读取全局所有机器人价格GlobalAllRobotPrice1_2
    if (!GlobalAllRobotPrice1_2.empty())
        //全局所有机器人价格GlobalAllRobotPrice1_2非空
    {
        if (!AllRobotPrice.empty())
            //局部所有机器人价格非空
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice1_2[i][j])
                    {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice1_2[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder1_2[i][j];
                    }
                }
            }
        }
        else
            //局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice1_2[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice1_2为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        cout << "全局所有机器人价格GlobalAllRobotPrice1_2为空" << endl;
    }  

    //读取全局所有机器人价格GlobalAllRobotPrice3_2
    if (!GlobalAllRobotPrice3_2.empty())
        //全局所有机器人价格GlobalAllRobotPrice3_2非空
    {
        if (!AllRobotPrice.empty())
            //局部所有机器人价格非空
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice3_2[i][j])
                    {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice3_2[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder3_2[i][j];
                    }
                }
            }
        }
        else
            //局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice3_2[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice3_2为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        cout << "全局所有机器人价格GlobalAllRobotPrice3_2为空" << endl;
    }  

    Mymutex1_2.unlock();
    Mymutex3_2.unlock();
}

/*
 * 读全局价格，Robot[3]向Robot[2]读数据，Robot[3]向Robot[4]读数据
 */
void crobot::readPrice3(int tasklist_num)
{
    Mymutex2_3.lock();
    Mymutex4_3.lock();

    //读取全局价格GlobalPrice2_3
    if (!GlobalPrice2_3.empty())
        //全局价格GlobalPrice2_3非空
    {
        if (!Price.empty())
            //局部价格Price非空
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                if (Price[i] < GlobalPrice2_3[i])
                {
                    Price[i] = GlobalPrice2_3[i];
                    Bidder[i] = GlobalBidder2_3[i];
                }
            }
            if (AssignedTask != -1)
            {
                if (Price[AssignedTask] == GlobalPrice2_3[AssignedTask])
                {
                    Bidder[AssignedTask] = GlobalBidder2_3[AssignedTask];
                }
            }
        }
        else
            //局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                Price.push_back(GlobalPrice2_3[i]);
                Bidder.push_back(GlobalBidder2_3[i]);
            }
        }
    }
    else
        //全局价格GlobalPrice2_3为空
    {
        //cout << "全局价格GlobalPrice2_3为空" << endl;
    }

    //读取全局价格GlobalPrice4_3
    if (!GlobalPrice4_3.empty())
        //全局价格GlobalPrice4_3非空
    {
        if (!Price.empty())
            //局部价格Price非空
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                if (Price[i] < GlobalPrice4_3[i])
                {
                    Price[i] = GlobalPrice4_3[i];
                    Bidder[i] = GlobalBidder4_3[i];
                }
            }
            if (AssignedTask != -1)
            {
                if (Price[AssignedTask] == GlobalPrice4_3[AssignedTask])
                {
                    Bidder[AssignedTask] = GlobalBidder4_3[AssignedTask];
                }
            }
        }
        else
            //局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                Price.push_back(GlobalPrice4_3[i]);
                Bidder.push_back(GlobalBidder4_3[i]);
            }
        }
    }
    else
        //全局价格GlobalPrice4_3为空
    {
        //cout << "全局价格GlobalPrice4_3为空" << endl;
    }    

    //读取全局所有机器人价格GlobalAllRobotPrice2_3
    if (!GlobalAllRobotPrice2_3.empty())
        //全局所有机器人价格GlobalAllRobotPrice2_3非空
    {
        if (!AllRobotPrice.empty())
            //局部所有机器人价格非空
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice2_3[i][j])
                    {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice2_3[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder2_3[i][j];
                    }
                }
            }
        }
        else
            //局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice2_3[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice2_3为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        cout << "全局所有机器人价格GlobalAllRobotPrice2_3为空" << endl;
    }  

    //读取全局所有机器人价格GlobalAllRobotPrice4_3
    if (!GlobalAllRobotPrice4_3.empty())
        //全局所有机器人价格GlobalAllRobotPrice4_3非空
    {
        if (!AllRobotPrice.empty())
            //局部所有机器人价格非空
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice4_3[i][j])
                    {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice4_3[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder4_3[i][j];
                    }
                }
            }
        }
        else
            //局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice4_3[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice4_3为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        cout << "全局所有机器人价格GlobalAllRobotPrice4_3为空" << endl;
    }  

    Mymutex2_3.unlock();
    Mymutex4_3.unlock();
}

/*
 * 读全局价格，Robot[4]向Robot[3]读数据，Robot[4]向Robot[5]读数据
 */
void crobot::readPrice4(int tasklist_num)
{
    Mymutex3_4.lock();
    Mymutex5_4.lock();

    //读取全局价格GlobalPrice3_4
    if (!GlobalPrice3_4.empty())
        //全局价格GlobalPrice3_4非空
    {
        if (!Price.empty())
            //局部价格Price非空
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                if (Price[i] < GlobalPrice3_4[i])
                {
                    Price[i] = GlobalPrice3_4[i];
                    Bidder[i] = GlobalBidder3_4[i];
                }
            }
            if (AssignedTask != -1)
            {
                if (Price[AssignedTask] == GlobalPrice3_4[AssignedTask])
                {
                    Bidder[AssignedTask] = GlobalBidder3_4[AssignedTask];
                }
            }
        }
        else
            //局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                Price.push_back(GlobalPrice3_4[i]);
                Bidder.push_back(GlobalBidder3_4[i]);
            }
        }
    }
    else
        //全局价格GlobalPrice3_4为空
    {
        //cout << "全局价格GlobalPrice3_4为空" << endl;
    }

    //读取全局价格GlobalPrice5_4
    if (!GlobalPrice5_4.empty())
        //全局价格GlobalPrice5_4非空
    {
        if (!Price.empty())
            //局部价格Price非空
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                if (Price[i] < GlobalPrice5_4[i])
                {
                    Price[i] = GlobalPrice5_4[i];
                    Bidder[i] = GlobalBidder5_4[i];
                }
            }
            if (AssignedTask != -1)
            {
                if (Price[AssignedTask] == GlobalPrice5_4[AssignedTask])
                {
                    Bidder[AssignedTask] = GlobalBidder5_4[AssignedTask];
                }
            }
        }
        else
            //局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                Price.push_back(GlobalPrice5_4[i]);
                Bidder.push_back(GlobalBidder5_4[i]);
            }
        }
    }
    else
        //全局价格GlobalPrice5_4为空
    {
        //cout << "全局价格GlobalPrice5_4为空" << endl;
    }    

    //读取全局所有机器人价格GlobalAllRobotPrice3_4
    if (!GlobalAllRobotPrice3_4.empty())
        //全局所有机器人价格GlobalAllRobotPrice3_4非空
    {
        if (!AllRobotPrice.empty())
            //局部所有机器人价格非空
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice3_4[i][j])
                    {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice3_4[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder3_4[i][j];
                    }
                }
            }
        }
        else
            //局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice3_4[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice3_4为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        cout << "全局所有机器人价格GlobalAllRobotPrice3_4为空" << endl;
    }  

    //读取全局所有机器人价格GlobalAllRobotPrice5_4
    if (!GlobalAllRobotPrice5_4.empty())
        //全局所有机器人价格GlobalAllRobotPrice5_4非空
    {
        if (!AllRobotPrice.empty())
            //局部所有机器人价格非空
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice5_4[i][j])
                    {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice5_4[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder5_4[i][j];
                    }
                }
            }
        }
        else
            //局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice5_4[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice5_4为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        cout << "全局所有机器人价格GlobalAllRobotPrice5_4为空" << endl;
    }      

    Mymutex3_4.unlock();
    Mymutex5_4.unlock();
}

/*
 * 读全局价格，Robot[5]向Robot[4]读数据
 */
void crobot::readPrice5(int tasklist_num)
{
    Mymutex4_5.lock();

    //读取全局价格GlobalPrice4_5
    if (!GlobalPrice4_5.empty())
        //全局价格GlobalPrice4_5非空
    {
        if (!Price.empty())
            //局部价格Price非空
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                if (Price[i] < GlobalPrice4_5[i])
                {
                    Price[i] = GlobalPrice4_5[i];
                    Bidder[i] = GlobalBidder4_5[i];
                }
            }
            if (AssignedTask != -1)
            {
                if (Price[AssignedTask] == GlobalPrice4_5[AssignedTask])
                {
                    Bidder[AssignedTask] = GlobalBidder4_5[AssignedTask];
                }
            }
        }
        else
            //局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < tasklist_num; i++)
            {
                Price.push_back(GlobalPrice4_5[i]);
                Bidder.push_back(GlobalBidder4_5[i]);
            }
        }
    }
    else
        //全局价格GlobalPrice4_5为空
    {
        //cout << "全局价格GlobalPrice4_5为空" << endl;
    }

    //读取全局所有机器人价格GlobalAllRobotPrice4_5
    if (!GlobalAllRobotPrice4_5.empty())
        //全局所有机器人价格GlobalAllRobotPrice4_5非空
    {
        if (!AllRobotPrice.empty())
            //局部所有机器人价格非空
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice4_5[i][j])
                    {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice4_5[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder4_5[i][j];
                    }
                }
            }
        }
        else
            //局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */
        {
            for (int i = 0; i < ROBOTNUM; i++)
            {
                for (int j = 0; j < tasklist_num; j++)
                {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice4_5[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice4_5为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        cout << "全局所有机器人价格GlobalAllRobotPrice4_5为空" << endl;
    }    

    Mymutex4_5.unlock();
}

/*
 * 设置价格
 */
void crobot::setPrice(int tasklist_num)
{
    //选择每个机器人子线程对应的全局价格写入函数
    switch (Robot_No)
    {
    case 0:
        writePrice0(tasklist_num);
        break;
    case 1:
        writePrice1(tasklist_num);
        break;
    case 2:
        writePrice2(tasklist_num);
        break;
    case 3:
        writePrice3(tasklist_num);
        break;
    case 4:
        writePrice4(tasklist_num);
        break;
    case 5:
        writePrice5(tasklist_num);
        break;
    default:
        break;
    }
}

/*
 * 写全局价格，Robot[0]向Robot[1]写数据
 */
void crobot::writePrice0(int tasklist_num)
{
    Mymutex0_1.lock();

    //写入全局价格GlobalPrice0_1
    if (!GlobalPrice0_1.empty())
        //全局价格GlobalPrice0_1非空
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice0_1[i] = Price[i];
            GlobalBidder0_1[i] = Bidder[i];
        }
    }
    else
        //全局价格GlobalPrice0_1为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice0_1.push_back(Price[i]);
            GlobalBidder0_1.push_back(Bidder[i]);
        }
    }

    //写入全局所有机器人价格GlobalAllRobotPrice0_1
    if (!GlobalAllRobotPrice0_1.empty())
        //全局所有机器人价格GlobalAllRobotPrice0_1非空
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice0_1[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder0_1[i][j] = AllRobotBidder[i][j];
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice0_1为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice0_1[i].push_back(AllRobotPrice[i][j]);            
            }
        }
    }
    
    Mymutex0_1.unlock();
}

/*
 * 写全局价格，Robot[1]向Robot[0]写数据，Robot[1]向Robot[2]写数据
 */
void crobot::writePrice1(int tasklist_num)
{
    Mymutex1_0.lock();
    Mymutex1_2.lock();

    //写入全局价格GlobalPrice1_0
    if (!GlobalPrice1_0.empty())
        //全局价格GlobalPrice1_0非空
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice1_0[i] = Price[i];
            GlobalBidder1_0[i] = Bidder[i];
        }
    }
    else
        //全局价格GlobalPrice1_0为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice1_0.push_back(Price[i]);
            GlobalBidder1_0.push_back(Bidder[i]);
        }
    }

    //写入全局价格GlobalPrice1_2
    if (!GlobalPrice1_2.empty())
        //全局价格GlobalPrice1_2非空
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice1_2[i] = Price[i];
            GlobalBidder1_2[i] = Bidder[i];
        }
    }
    else
        //全局价格GlobalPrice1_2为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice1_2.push_back(Price[i]);
            GlobalBidder1_2.push_back(Bidder[i]);
        }
    }    

    //写入全局所有机器人价格GlobalAllRobotPrice1_0
    if (!GlobalAllRobotPrice1_0.empty())
        //全局所有机器人价格GlobalAllRobotPrice1_0非空
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice1_0[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder1_0[i][j] = AllRobotBidder[i][j];
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice1_0为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice1_0[i].push_back(AllRobotPrice[i][j]);            
            }
        }
    }

    //写入全局所有机器人价格GlobalAllRobotPrice1_2
    if (!GlobalAllRobotPrice1_2.empty())
        //全局所有机器人价格GlobalAllRobotPrice1_2非空
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice1_2[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder1_2[i][j] = AllRobotBidder[i][j];
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice1_2为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice1_2[i].push_back(AllRobotPrice[i][j]);            
            }
        }
    }

    Mymutex1_0.unlock();
    Mymutex1_2.unlock();
}

/*
 * 写全局价格，Robot[2]向Robot[1]写数据，Robot[2]向Robot[3]写数据
 */
void crobot::writePrice2(int tasklist_num)
{
    Mymutex2_1.lock();
    Mymutex2_3.lock();

    //写入全局价格GlobalPrice2_1
    if (!GlobalPrice2_1.empty())
        //全局价格GlobalPrice2_1非空
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice2_1[i] = Price[i];
            GlobalBidder2_1[i] = Bidder[i];
        }
    }
    else
        //全局价格GlobalPrice2_1为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice2_1.push_back(Price[i]);
            GlobalBidder2_1.push_back(Bidder[i]);
        }
    }

    //写入全局价格GlobalPrice2_3
    if (!GlobalPrice2_3.empty())
        //全局价格GlobalPrice2_3非空
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice2_3[i] = Price[i];
            GlobalBidder2_3[i] = Bidder[i];
        }
    }
    else
        //全局价格GlobalPrice2_3为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice2_3.push_back(Price[i]);
            GlobalBidder2_3.push_back(Bidder[i]);
        }
    }    

    //写入全局所有机器人价格GlobalAllRobotPrice2_1
    if (!GlobalAllRobotPrice2_1.empty())
        //全局所有机器人价格GlobalAllRobotPrice2_1非空
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice2_1[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder2_1[i][j] = AllRobotBidder[i][j];
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice2_1为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice2_1[i].push_back(AllRobotPrice[i][j]);            
            }
        }
    }

    //写入全局所有机器人价格GlobalAllRobotPrice2_3
    if (!GlobalAllRobotPrice2_3.empty())
        //全局所有机器人价格GlobalAllRobotPrice2_3非空
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice2_3[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder2_3[i][j] = AllRobotBidder[i][j];
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice2_3为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice2_3[i].push_back(AllRobotPrice[i][j]);            
            }
        }
    }    

    Mymutex2_1.unlock();
    Mymutex2_3.unlock();
}

/*
 * 写全局价格，Robot[3]向Robot[2]写数据，Robot[3]向Robot[4]写数据
 */
void crobot::writePrice3(int tasklist_num)
{
    Mymutex3_2.lock();
    Mymutex3_4.lock();

    //写入全局价格GlobalPrice3_2
    if (!GlobalPrice3_2.empty())
        //全局价格GlobalPrice3_2非空
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice3_2[i] = Price[i];
            GlobalBidder3_2[i] = Bidder[i];
        }
    }
    else
        //全局价格GlobalPrice3_2为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice3_2.push_back(Price[i]);
            GlobalBidder3_2.push_back(Bidder[i]);
        }
    }

    //写入全局价格GlobalPrice3_4
    if (!GlobalPrice3_4.empty())
        //全局价格GlobalPrice3_4非空
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice3_4[i] = Price[i];
            GlobalBidder3_4[i] = Bidder[i];
        }
    }
    else
        //全局价格GlobalPrice3_4为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice3_4.push_back(Price[i]);
            GlobalBidder3_4.push_back(Bidder[i]);
        }
    }    

    //写入全局所有机器人价格GlobalAllRobotPrice3_2
    if (!GlobalAllRobotPrice3_2.empty())
        //全局所有机器人价格GlobalAllRobotPrice3_2非空
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice3_2[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder3_2[i][j] = AllRobotBidder[i][j];
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice3_2为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice3_2[i].push_back(AllRobotPrice[i][j]);            
            }
        }
    }

    //写入全局所有机器人价格GlobalAllRobotPrice3_4
    if (!GlobalAllRobotPrice3_4.empty())
        //全局所有机器人价格GlobalAllRobotPrice3_4非空
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice3_4[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder3_4[i][j] = AllRobotBidder[i][j];
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice3_4为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice3_4[i].push_back(AllRobotPrice[i][j]);            
            }
        }
    }      

    Mymutex3_2.unlock();
    Mymutex3_4.unlock();
}

/*
 * 写全局价格，Robot[4]向Robot[3]写数据，Robot[4]向Robot[5]写数据
 */
void crobot::writePrice4(int tasklist_num)
{
    Mymutex4_3.lock();
    Mymutex4_5.lock();

    //写入全局价格GlobalPrice4_3
    if (!GlobalPrice4_3.empty())
        //全局价格GlobalPrice4_3非空
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice4_3[i] = Price[i];
            GlobalBidder4_3[i] = Bidder[i];
        }
    }
    else
        //全局价格GlobalPrice4_3为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice4_3.push_back(Price[i]);
            GlobalBidder4_3.push_back(Bidder[i]);
        }
    }

    //写入全局价格GlobalPrice4_5
    if (!GlobalPrice4_5.empty())
        //全局价格GlobalPrice4_3非空
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice4_5[i] = Price[i];
            GlobalBidder4_5[i] = Bidder[i];
        }
    }
    else
        //全局价格GlobalPrice4_5为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice4_5.push_back(Price[i]);
            GlobalBidder4_5.push_back(Bidder[i]);
        }
    }    

    //写入全局所有机器人价格GlobalAllRobotPrice4_3
    if (!GlobalAllRobotPrice4_3.empty())
        //全局所有机器人价格GlobalAllRobotPrice4_3非空
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice4_3[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder4_3[i][j] = AllRobotBidder[i][j];
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice4_3为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice4_3[i].push_back(AllRobotPrice[i][j]);            
            }
        }
    }

    //写入全局所有机器人价格GlobalAllRobotPrice4_5
    if (!GlobalAllRobotPrice4_5.empty())
        //全局所有机器人价格GlobalAllRobotPrice4_5非空
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice4_5[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder4_5[i][j] = AllRobotBidder[i][j];
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice4_5为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice4_5[i].push_back(AllRobotPrice[i][j]);            
            }
        }
    }    

    Mymutex4_3.unlock();
    Mymutex4_5.unlock();
}

/*
 * 写全局价格，Robot[5]向Robot[4]写数据
 */
void crobot::writePrice5(int tasklist_num)
{
   Mymutex5_4.lock();

    //写入全局价格GlobalPrice5_4
    if (!GlobalPrice5_4.empty())
        //全局价格GlobalPrice5_4非空
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice5_4[i] = Price[i];
            GlobalBidder5_4[i] = Bidder[i];
        }
    }
    else
        //全局价格GlobalPrice5_4为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < tasklist_num; i++)
        {
            GlobalPrice5_4.push_back(Price[i]);
            GlobalBidder5_4.push_back(Bidder[i]);
        }
    } 

    //写入全局所有机器人价格GlobalAllRobotPrice5_4
    if (!GlobalAllRobotPrice5_4.empty())
        //全局所有机器人价格GlobalAllRobotPrice5_4非空
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice5_4[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder5_4[i][j] = AllRobotBidder[i][j];
            }
        }
    }
    else
        //全局所有机器人价格GlobalAllRobotPrice5_4为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
    {
        for (int i = 0; i < ROBOTNUM; i++)
        {
            for (int j = 0; j < tasklist_num; j++)
            {
                GlobalAllRobotPrice5_4[i].push_back(AllRobotPrice[i][j]);            
            }
        }
    }     

   Mymutex5_4.unlock();
}

/*
 * 竞拍
 */
void crobot::auction(vector<float> PriceOld, int tasklist_num)
{
    if (AssignedTask != -1)
    {
        if ((Price[AssignedTask] >= PriceOld[AssignedTask]) && Bidder[AssignedTask] != Robot_No)
            //原先所竞标的任务价格上涨（或不变），同时原先所竞标的任务并非当前机器人执行
        {
            //出价
            bidding(tasklist_num);
        }
        else
        {
            //将出价和竞标者写入所有机器人价格中
            for (int i = 0; i < tasklist_num; i++)
            {
                AllRobotPrice[Robot_No][i] = Price[i];
                AllRobotBidder[Robot_No][i] = Bidder[i];
            }
        }
    }
    else
    {
        //出价
        bidding(tasklist_num);
    }
}

/*
 * 收敛
 */
void crobot::convergence(bool &flag)
{
    if (AllRobotBidder[0] == AllRobotBidder[1] &&
        AllRobotBidder[1] == AllRobotBidder[2] &&
        AllRobotBidder[2] == AllRobotBidder[3] &&
        AllRobotBidder[3] == AllRobotBidder[4] &&
        AllRobotBidder[4] == AllRobotBidder[5]
        )
    {
        flag = true;
    }
    else
    {
        flag = false;
    } 
}

/*
 * 更新机器人位置坐标
 */
void crobot::updadteRobotLocation(ctasklist * tasklist)
{
    RobotLocation[0] = tasklist->sendTaskQueue(AssignedTask)->EndPoint[0];
    RobotLocation[1] = tasklist->sendTaskQueue(AssignedTask)->EndPoint[1];
}

/*
 * 将中标的任务存入机器人任务执行队列
 */
void crobot::savetoTaskExecutionQueue(ctasklist * tasklist)
{
    TaskTemplate * tmp = tasklist->sendTaskQueue(AssignedTask);
    TaskExecutionQueue.insert(TaskExecutionQueue.begin() + maxValuePosition, *tmp);
}

/*
 * 打印分配的任务
 */
void crobot::printAssignedTask(ctasklist * tasklist)
{
    cout << "机器人编号" << Robot_No
        << "任务点编号：" << tasklist->sendTaskQueue(AssignedTask)->PointNo
        << "任务编号：" << tasklist->sendTaskQueue(AssignedTask)->TaskNo
        << endl;
}

/*
 * 发送分配任务的价值
 */
float crobot::sendAssignedTaskValue()
{
    return ValueList[AssignedTask];
}

/*
 * 发送剩余任务数量
 */
int crobot::sendResidualNum()
{
    return ResidualNum;
}

/*
 * 发送剩余任务
 */
TaskTemplate * crobot::sendResidualTask(ctasklist * tasklist, int i)
{
    return tasklist->sendTaskQueue(ResidualTask[i]);
}

/*
 * 重置机器人
 */
void crobot::clearPropertity()
{
    ValueList.clear();
    Price.clear();
    Bidder.clear();
    for (int i = 0; i < ROBOTNUM; i++)
    {
        AllRobotPrice[i].clear();
        AllRobotBidder[i].clear();
    }
    AssignedTask = -1;
    ResidualTask.clear();
    ResidualNum = 0;
}

/*
 * 发送机器人编号
 */
int crobot::sendRobotNum()
{
    return Robot_No;
}

/*
 * 多机器人协调策略
 */
void multirobotCoordination()
{

}