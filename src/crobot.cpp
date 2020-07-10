/*
 # Copyright (c) 2019-2020 Xinyan Han. All rights reserved.
 */
#include "crobot.h"
#include "cmultirobotCoordinate.h"
#include "ccoordinatecommunication.h"

//机器人协调读写子线程入口函数声明
void writeTaskExecutionQueue(crobot * Robot);
void readTaskExecutionQueue(crobot * Robot);

/*
 * 构造函数
 */
crobot::crobot():AllRobotPrice(ROBOTNUM), AllRobotBidder(ROBOTNUM), TaskExecutionQueueNum(0), CoorCommunicateLength(0), CoorCommunicateTime(0) {
    CoorCommunicateWidth = 2;
    maxValuePosition = -1;
    // Rate = RAND_ROBOT_RATE;
    Rate = ROBOT_RATE;
}

/*
 * 向机器人传递所有初始化参数
 */
void crobot::setInitialValue(int r_No, float location_x, float location_y) {
    Robot_No = r_No;
    RobotLocation[0] = location_x;
    RobotLocation[1] = location_y;
}

/*
 * 打印机器人相关信息
 */
void crobot::printRobotInfo() {
    cout << "机器人编号：" << Robot_No << " "
        << "机器人位置：" << RobotLocation[0] << "," << RobotLocation[1] << " "
        << "机器人任务执行效率" << Rate
        << endl;
}

/*
 * 机器人子线程
 */
void crobot::generateValueList(ctasklist * tasklist, int tasklist_num, float rand_num, promise<crobot> &PromiseRobot) {
    cout << "子线程启动，线程ID：" << this_thread::get_id() << endl;
    float random_num = RAND_NUM;
    // cout << random_num << endl;

    // 生成价值列表
    for (int i = 0; i < tasklist_num; i++) {
        Mymutex.lock();
        calculateValue(tasklist, i);
        Mymutex.unlock();
    }

    // 局部变量赋值
    vector<float> PriceOld;  // 旧价格
    eps = 0.0001 + rand_num;   // 松弛变量，加上随机数，每个机器人不同
    bool flag = false;
    AssignedTask = -1;

    // 局部价格，竞标者赋值
    for (int i = 0; i < tasklist_num; i++) {
        Price.push_back(0);
        PriceOld.push_back(0);
        Bidder.push_back(-1);
    }

    // 所有机器人价格赋值
    // 所有机器人竞标者赋值
    for (int i = 0; i < ROBOTNUM; i++) {
        for (int j = 0; j < tasklist_num; j++) {
            AllRobotPrice[i].push_back(0);
            AllRobotBidder[i].push_back(-1);
        }
    }

    // 更新价格，竞拍出价，发送价格
    while (flag == false) {
        // 更新价格
        updatePrice(tasklist_num);

        // 竞标出价
        auction(PriceOld, tasklist_num);

        // 设置价格
        setPrice(tasklist_num);

        // 收敛
        convergence(flag);

        //保存历史价格
        for (int i = 0; i < tasklist_num; i++) {
            PriceOld[i] = Price[i];
        }
    }

    // 将中标的任务存入机器人任务执行队列
    savetoTaskExecutionQueue(tasklist);
    TaskExecutionQueueNum = TaskExecutionQueue.size();

#if MULTIROBOT_COORDINATE
    // 将任务执行队列保存至全局变量，用于协调通信
    // 机器人异步读写线程，用于协调通信
    // 创建读写线程
    setCoorTEQWidth(CoorCommunicateWidth);    // 设置机器人协调通信范围，这里为邻接机器人，即2
    thread RobotReadThread(readTaskExecutionQueue, this);
    thread RobotWriteThread(writeTaskExecutionQueue, this);
    RobotReadThread.join();
    RobotWriteThread.join();

    // 机器人协调通信类，用于进入协调过程
    ccoordinatecommunication CoordinateCommunication(this);
    CoordinateCommunication.enterCoordinate();

    // 回收CoorTEQWidth内存
    deleteCoorTEQWidth();
#endif

    /*
     * 以下整理剩余任务这部分代码从逻辑上讲应该归入主函数，在主线程中执行，不应该在子线程中执行，
     * 因此这一部分代码需要重构
     */
    // 整理剩余任务
    for (int i = 0; i < tasklist_num; i++) {
        if (Bidder[i] == -1) {
            // 如果任务未被机器人分配
            ResidualTask.push_back(i);
        }
    }
    ResidualNum = ResidualTask.size();

    PromiseRobot.set_value(*this);
}

/*
 * 机器人计算任务列表某个任务的价值
 */
void crobot::calculateValue(ctasklist * tasklist, int i) {
    TaskTemplate * TmpTask;   // 待计算价值的任务
    TmpTask = tasklist->sendTaskQueue(i);   // 取任务列表中的第i个任务，别名为TmpTask

#if GENERAL_UTILITY
    // 常规直接计算任务价值
    // GeneralCalculate(TmpTask);
    NewGeneralCalculate(TmpTask);
    printValueList(i);
#endif

#if SINGLE_COORDINATE
    // 寻找使插入新任务后整体任务执行队列价值最高的插入点(机器人自协调)
    SelfCoordination(TmpTask);
#endif

#if TASK_EXECUTION
    // 任务执行，常规直接计算任务价值
    calGeneralTaskUnexe(TmpTask);
    printValueList(i);
#endif
}

/*
 * 打印任务价值列表
 */
void crobot::printValueList(int i) {
    cout << "机器人编号：" << Robot_No << " "
        << "任务" << i << "的价值为：" << ValueList[i]
        << endl;
}

/*
 * @deprecated RobotLocation修改会引起后续问题，不建议修改和使用
 * 常规直接计算任务价值(不带机器人自协调)
 * @remark：该函数存在bug。不建议使用RobotLocation，该变量在sendTaskExecutionQueueValue中用于计算Value，在以下函数中
 * RobotLocation会随任务分配而变化。同时建议去掉位置更新函数。
 */
[[deprecated]] void crobot::GeneralCalculate(TaskTemplate * TmpTask) {
    float Distance;  // 机器人完成任务的路程
    float Value;    // 任务价值

    Distance = sqrt(pow(TmpTask->BeginPoint[0] - RobotLocation[0], 2) + pow(TmpTask->BeginPoint[1] - RobotLocation[1], 2)) +
        sqrt(pow(TmpTask->EndPoint[0] - TmpTask->BeginPoint[0], 2) + pow(TmpTask->EndPoint[1] - TmpTask->BeginPoint[1], 2));
    Value = 1 / Distance;
    ValueList.push_back(Value);
}

/*
 * 新常规直接计算任务价值(不带机器人自协调)
 */
void crobot::NewGeneralCalculate(TaskTemplate * TmpTask) {
    float Distance;  // 机器人完成任务的路程
    float Value;    // 任务价值

    if (TaskExecutionQueueNum) {
        Distance = sqrt(pow(TmpTask->BeginPoint[0] - TaskExecutionQueue.back().EndPoint[0], 2) +
         pow(TmpTask->BeginPoint[1] - TaskExecutionQueue.back().EndPoint[1], 2)) +
            sqrt(pow(TmpTask->EndPoint[0] - TmpTask->BeginPoint[0], 2) + pow(TmpTask->EndPoint[1] - TmpTask->BeginPoint[1], 2));
        Value = 1 / Distance;
        ValueList.push_back(Value);
    } else {
        Distance = sqrt(pow(TmpTask->BeginPoint[0] - RobotLocation[0], 2) + pow(TmpTask->BeginPoint[1] - RobotLocation[1], 2)) +
        sqrt(pow(TmpTask->EndPoint[0] - TmpTask->BeginPoint[0], 2) + pow(TmpTask->EndPoint[1] - TmpTask->BeginPoint[1], 2));
        Value = 1 / Distance;
        ValueList.push_back(Value);
    }
}

/*
 * 寻找使插入新任务后整体任务执行队列价值最高的插入点(机器人自协调)
 */
void crobot::SelfCoordination(TaskTemplate * TmpTask) {
    float DiffPositionAllValue = 0;  // 不同位置上的任务总价值
    vector<float> DiffPositionNetValue;  // 不同位置上的新任务净价值
    maxValue = 0;
    maxValuePosition = -1;

    if (TaskExecutionQueueNum != 0) {
        // 如果任务执行队列的任务数量非0
        for (int i = 0; i < TaskExecutionQueueNum + 1; i++) {
            DiffPositionAllValue = calculateTmpTaskExecutionQueueValue(TmpTask, i);
            DiffPositionNetValue.push_back(DiffPositionAllValue - sendTaskExecutionQueueValue());
        }
    } else {
        // 如果任务执行队列的任务数量为0
        DiffPositionAllValue = calculateTmpTaskExecutionQueueValue(TmpTask, 0);
        DiffPositionNetValue.push_back(DiffPositionAllValue);
    }

    // 选择最大的价值以及对应的下标（位置）
    maxValue = *max_element(DiffPositionNetValue.begin(), DiffPositionNetValue.end());
    // 下标表示，新竞标的任务安插在第i个任务之后（下标为0表示，新竞标的任务安插在头位置）
    maxValuePosition = max_element(DiffPositionNetValue.begin(), DiffPositionNetValue.end()) - DiffPositionNetValue.begin();
    ValueList.push_back(maxValue);
}

/*
 * 计算临时任务执行队列总价值（计算新任务插入执行队列某位置后，新队列的总价值）
 */
float crobot::calculateTmpTaskExecutionQueueValue(TaskTemplate * TmpTask, int position) {
    vector<TaskTemplate> TmpTaskExecutionQueue = TaskExecutionQueue;    // 定义临时任务执行队列
    TmpTaskExecutionQueue.insert(TmpTaskExecutionQueue.begin() + position, *TmpTask);    // 新任务插入临时任务执行队列
    float TmpTaskExecutionQueueNum;
    float tmpValue = 0;
    float tmpDistance = 0;
    TmpTaskExecutionQueueNum = TmpTaskExecutionQueue.size();
    tmpDistance = sqrt(pow(TmpTaskExecutionQueue[0].BeginPoint[0] - RobotLocation[0], 2) +
        pow(TmpTaskExecutionQueue[0].BeginPoint[1] - RobotLocation[1], 2)) +
        sqrt(pow(TmpTaskExecutionQueue[0].EndPoint[0] - TmpTaskExecutionQueue[0].BeginPoint[0], 2) +
        pow(TmpTaskExecutionQueue[0].EndPoint[1] - TmpTaskExecutionQueue[0].BeginPoint[1], 2));
    tmpValue += 1 / tmpDistance;    // 累积价值
    for (int i = 1; i < TmpTaskExecutionQueueNum; i++) {
        tmpDistance = sqrt(pow(TmpTaskExecutionQueue[i].BeginPoint[0] - TmpTaskExecutionQueue[i - 1].EndPoint[0], 2) +
            pow(TmpTaskExecutionQueue[i].BeginPoint[1] - TmpTaskExecutionQueue[i - 1].EndPoint[1], 2)) +
            sqrt(pow(TmpTaskExecutionQueue[i].EndPoint[0] - TmpTaskExecutionQueue[i].BeginPoint[0], 2) +
            pow(TmpTaskExecutionQueue[i].EndPoint[1] - TmpTaskExecutionQueue[i].BeginPoint[1], 2));
        tmpValue += 1 / tmpDistance;    // 累积价值
    }

    return tmpValue;
}

/*
 * 发送任务执行队列总价值
 * @remark：在当前常规计算任务价值函数中（GeneralCalculate），RobotLocation会随便任务分配而改变，
 * 此处为严重bug，导致以下函数计算的Value不可用。
 */
float crobot::sendTaskExecutionQueueValue() {
    float tmpValue = 0;
    float tmpDistance = 0;

    /*
     * @repair
     * 尝试修复机器人数大于任务数异常bug
     */
    if (TaskExecutionQueue.size()) {
        tmpDistance = sqrt(pow(TaskExecutionQueue[0].BeginPoint[0] - RobotLocation[0], 2) +
            pow(TaskExecutionQueue[0].BeginPoint[1] - RobotLocation[1], 2)) +
            sqrt(pow(TaskExecutionQueue[0].EndPoint[0] - TaskExecutionQueue[0].BeginPoint[0], 2) +
            pow(TaskExecutionQueue[0].EndPoint[1] - TaskExecutionQueue[0].BeginPoint[1], 2));
        tmpValue += 1 / tmpDistance;    // 累积价值
        for (int i = 1; i < TaskExecutionQueueNum; i++) {
            tmpDistance = sqrt(pow(TaskExecutionQueue[i].BeginPoint[0] - TaskExecutionQueue[i - 1].EndPoint[0], 2) +
                pow(TaskExecutionQueue[i].BeginPoint[1] - TaskExecutionQueue[i - 1].EndPoint[1], 2)) +
                sqrt(pow(TaskExecutionQueue[i].EndPoint[0] - TaskExecutionQueue[i].BeginPoint[0], 2) +
                pow(TaskExecutionQueue[i].EndPoint[1] - TaskExecutionQueue[i].BeginPoint[1], 2));
            tmpValue += 1 / tmpDistance;    // 累积价值
        }
    }

    return tmpValue;
}

/*
 * 任务执行计算任务价值（常规）
 */
void crobot::calGeneralTaskUnexe(TaskTemplate * TmpTask) {
    float Distance = 0;     // 机器人完成当前竞标任务的路程
    float Value = 0;    // 任务价值
    float UnexeDistance = 0;    // 未完成任务路程

    // 计算当前竞标任务路程
    int TEQsize = getTaskExecutionQueueLength();
    if (TEQsize != 0) {
        Distance = sqrt(pow(TaskExecutionQueue[TEQsize - 1].EndPoint[0] - TmpTask->BeginPoint[0], 2) +
        pow(TaskExecutionQueue[TEQsize - 1].EndPoint[1] - TmpTask->BeginPoint[1], 2)) +
        sqrt(pow(TmpTask->EndPoint[0] - TmpTask->BeginPoint[0], 2) +
        pow(TmpTask->EndPoint[1] - TmpTask->BeginPoint[1], 2));
    } else {
        Distance = sqrt(pow(RobotLocation[0] - TmpTask->BeginPoint[0], 2) +
        pow(RobotLocation[1] - TmpTask->BeginPoint[1], 2)) +
        sqrt(pow(TmpTask->EndPoint[0] - TmpTask->BeginPoint[0], 2) +
        pow(TmpTask->EndPoint[1] - TmpTask->BeginPoint[1], 2));
    }

    // 计算未完成任务路程
    for (int i = 0; i < TEQsize; ++i) {
        // 任务执行标志位为2， 当前任务已完成
        if (TaskExecutionQueue[i].TaskExecutedFlag == 2) continue;
        float test = static_cast<float>(TaskExecutionQueue[i].TaskExeProgress) / static_cast<float>(TaskExecutionQueue[i].TaskLoad);
        // 任务执行标志位为1， 当前任务正在执行
        // 计算当前执行任务的未执行部分比例
        if (TaskExecutionQueue[i].TaskExecutedFlag == 1) {
            UnexeDistance += sqrt(pow(TaskExecutionQueue[i].EndPoint[0] - TaskExecutionQueue[i].BeginPoint[0], 2) +
            pow(TaskExecutionQueue[i].EndPoint[1] - TaskExecutionQueue[i].BeginPoint[1], 2)) *
            (1.0f - static_cast<float>(TaskExecutionQueue[i].TaskExeProgress) / static_cast<float>(TaskExecutionQueue[i].TaskLoad));
            continue;
        }

        // 任务执行标志位为0， 当前任务未执行
        if (i >= 1) {
            UnexeDistance += sqrt(pow(TaskExecutionQueue[i - 1].EndPoint[0] - TaskExecutionQueue[i].BeginPoint[0], 2) +
            pow(TaskExecutionQueue[i - 1].EndPoint[1] - TaskExecutionQueue[i].BeginPoint[1], 2)) +
            sqrt(pow(TaskExecutionQueue[i].EndPoint[0] - TaskExecutionQueue[i].BeginPoint[0], 2) +
            pow(TaskExecutionQueue[i].EndPoint[1] - TaskExecutionQueue[i].BeginPoint[1], 2));
        } else {
            UnexeDistance += sqrt(pow(RobotLocation[0] - TaskExecutionQueue[i].BeginPoint[0], 2) +
            pow(RobotLocation[1] - TaskExecutionQueue[i].BeginPoint[1], 2)) +
            sqrt(pow(TaskExecutionQueue[i].EndPoint[0] - TaskExecutionQueue[i].BeginPoint[0], 2) +
            pow(TaskExecutionQueue[i].EndPoint[1] - TaskExecutionQueue[i].BeginPoint[1], 2));
        }
    }

    Value = 1 / (Distance + UnexeDistance);
    ValueList.push_back(Value);
}

/*
 * 出价
 * @warning 竞拍算法核心代码，需要增加必要的断言或异常处理确保竞拍过程正常运行
 */
void crobot::bidding(int tasklist_num) {
    float NetMax = 0;   // 重置最大净值
    float NetSecMax = 0;    // 重置次大净值StartTask
    vector<float> Net;  // 净值
    float AssignedPrice = 0;    // 机器人竞标任务增加的出价

    //初始化净值
    for (int i = 0; i < tasklist_num; i++) {
        Net.push_back(0);
    }

    /*
     * @remarks for(int i = 0; ···)，下标访问不能直接改为迭代器访问 for(auto iter = ; ···)。
     * 可以将bidding中的多种有关联的vector(Net,ValueList,Price)结构化，但Net目前不属于类成员变量。这样子便可以迭代器访问。
     */
    // 首先选择净值最大的任务进行出价
    for (int i = 0; i < tasklist_num; i++) {
        Net[i] = ValueList[i] - Price[i];
        if (NetMax < Net[i]) {
            NetMax = Net[i];    // 最大的净值
            AssignedTask = i;   // 选择竞标的任务
        }
    }

    /*
     * @repair
     * 尝试修复机器人数大于任务数异常bug
     */
    if (NetMax == 0) {
        AssignedTask = -1;
    }


    /*
     * 可直接改为迭代器访问
     */
    //选择净值第二大的任务
    for (int i = 0; i < tasklist_num; i++) {
        if (i != AssignedTask) {
            // 非最大的净值
            if (NetSecMax < Net[i]) {
                NetSecMax = Net[i];  // 次大的净值
            }
        }
    }

    /*
     * @repair
     * 尝试修复机器人数大于任务数异常bug
     */
    if (NetMax > 0) {
        // 出价，设置新竞标价格和新竞标者
        AssignedPrice = NetMax - NetSecMax + eps;
        Price[AssignedTask] = AssignedPrice + Price[AssignedTask];
        Bidder[AssignedTask] = Robot_No;
    }

    // 将出价和竞标者写入所有机器人价格，竞标者中
    setARPandARB(tasklist_num);
}

/*
 * 将出价和竞标者写入所有机器人价格和竞标者中
 */
void crobot::setARPandARB(int tasklist_num) {
    for (int i = 0; i < tasklist_num; i++) {
        AllRobotPrice[Robot_No][i] = Price[i];
        AllRobotBidder[Robot_No][i] = Bidder[i];
    }
}

/*
 * 更新价格
 */
void crobot::updatePrice(int tasklist_num) {
    // 选择每个机器人子线程对应的全局价格读出函数
    switch (Robot_No) {
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
void crobot::readPrice0(int tasklist_num) {
    Mymutex1_0.lock();
    // 读取全局价格GlobalPrice1_0
    if (!GlobalPrice1_0.empty()) {
        // 全局价格GlobalPrice1_0非空
        if (!Price.empty()) {
            // 局部价格Price非空
            for (int i = 0; i < tasklist_num; i++) {
                if (Price[i] < GlobalPrice1_0[i]) {
                    Price[i] = GlobalPrice1_0[i];
                    Bidder[i] = GlobalBidder1_0[i];
                }
            }
            if (AssignedTask != -1) {
                if (Price[AssignedTask] == GlobalPrice1_0[AssignedTask]) {
                    Bidder[AssignedTask] = GlobalBidder1_0[AssignedTask];
                }
            }
        } else {
            // 局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */
            for (int i = 0; i < tasklist_num; i++) {
                Price.push_back(GlobalPrice1_0[i]);
                Bidder.push_back(GlobalBidder1_0[i]);
            }
        }
    } else {
        // 全局价格GlobalPrice1_0为空
        // cout << "全局价格GlobalPrice1_0为空" << endl;
    }

    // 读取全局所有机器人价格GlobalAllRobotPrice1_0
    if (!GlobalAllRobotPrice1_0.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice1_0非空
        if (!AllRobotPrice.empty()) {
            // 局部所有机器人价格非空
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice1_0[i][j]) {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice1_0[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder1_0[i][j];
                    }
                }
            }
        } else {
            // 局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice1_0[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice1_0为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        cout << "全局所有机器人价格GlobalAllRobotPrice1_0为空" << endl;
    }

    Mymutex1_0.unlock();
}

/*
 * 读全局价格，Robot[1]向Robot[0]读数据，Robot[1]向Robot[2]读数据
 */
void crobot::readPrice1(int tasklist_num) {
    Mymutex0_1.lock();
    Mymutex2_1.lock();

    // 读取全局价格GlobalPrice0_1
    if (!GlobalPrice0_1.empty()) {
        // 全局价格GlobalPrice0_1非空
        if (!Price.empty()) {
            // 局部价格Price非空
            for (int i = 0; i < tasklist_num; i++) {
                if (Price[i] < GlobalPrice0_1[i]) {
                    Price[i] = GlobalPrice0_1[i];
                    Bidder[i] = GlobalBidder0_1[i];
                }
            }
            if (AssignedTask != -1) {
                if (Price[AssignedTask] == GlobalPrice0_1[AssignedTask]) {
                    Bidder[AssignedTask] = GlobalBidder0_1[AssignedTask];
                }
            }
        } else {
            // 局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < tasklist_num; i++) {
                Price.push_back(GlobalPrice0_1[i]);
                Bidder.push_back(GlobalBidder0_1[i]);
            }
        }
    } else {
        // 全局价格GlobalPrice0_1为空
        // cout << "全局价格GlobalPrice0_1为空" << endl;
    }

    // 读取全局价格GlobalPrice2_1
    if (!GlobalPrice2_1.empty()) {
        // 全局价格GlobalPrice2_1非空
        if (!Price.empty()) {
            // 局部价格Price非空
            for (int i = 0; i < tasklist_num; i++) {
                if (Price[i] < GlobalPrice2_1[i]) {
                    Price[i] = GlobalPrice2_1[i];
                    Bidder[i] = GlobalBidder2_1[i];
                }
            }
            if (AssignedTask != -1) {
                if (Price[AssignedTask] == GlobalPrice2_1[AssignedTask]) {
                    Bidder[AssignedTask] = GlobalBidder2_1[AssignedTask];
                }
            }
        } else {
            // 局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < tasklist_num; i++) {
                Price.push_back(GlobalPrice2_1[i]);
                Bidder.push_back(GlobalBidder2_1[i]);
            }
        }
    } else {
        // 全局价格GlobalPrice2_1为空
        // cout << "全局价格GlobalPrice2_1为空" << endl;
    }

    // 读取全局所有机器人价格GlobalAllRobotPrice0_1
    if (!GlobalAllRobotPrice0_1.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice0_1非空
        if (!AllRobotPrice.empty()) {
            // 局部所有机器人价格非空
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice0_1[i][j]) {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice0_1[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder0_1[i][j];
                    }
                }
            }
        } else {
            // 局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice0_1[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice0_1为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        cout << "全局所有机器人价格GlobalAllRobotPrice0_1为空" << endl;
    }

    // 读取全局所有机器人价格GlobalAllRobotPrice2_1
    if (!GlobalAllRobotPrice2_1.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice2_1非空
        if (!AllRobotPrice.empty()) {
            // 局部所有机器人价格非空
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice2_1[i][j]) {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice2_1[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder2_1[i][j];
                    }
                }
            }
        } else {
            // 局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice2_1[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice2_1为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        cout << "全局所有机器人价格GlobalAllRobotPrice2_1为空" << endl;
    }

    Mymutex0_1.unlock();
    Mymutex2_1.unlock();
}

/*
 * 读全局价格，Robot[2]向Robot[1]读数据，Robot[2]向Robot[3]读数据
 */
void crobot::readPrice2(int tasklist_num) {
    Mymutex1_2.lock();
    Mymutex3_2.lock();

    // 读取全局价格GlobalPrice1_2
    if (!GlobalPrice1_2.empty()) {
        // 全局价格GlobalPrice1_2非空
        if (!Price.empty()) {
            // 局部价格Price非空
            for (int i = 0; i < tasklist_num; i++) {
                if (Price[i] < GlobalPrice1_2[i]) {
                    Price[i] = GlobalPrice1_2[i];
                    Bidder[i] = GlobalBidder1_2[i];
                }
            }
            if (AssignedTask != -1) {
                if (Price[AssignedTask] == GlobalPrice1_2[AssignedTask]) {
                    Bidder[AssignedTask] = GlobalBidder1_2[AssignedTask];
                }
            }
        } else {
            // 局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < tasklist_num; i++) {
                Price.push_back(GlobalPrice1_2[i]);
                Bidder.push_back(GlobalBidder1_2[i]);
            }
        }
    } else {
        // 全局价格GlobalPrice1_2为空
        // cout << "全局价格GlobalPrice1_2为空" << endl;
    }

    // 读取全局价格GlobalPrice3_2
    if (!GlobalPrice3_2.empty()) {
        // 全局价格GlobalPrice3_2非空
        if (!Price.empty()) {
            // 局部价格Price非空
            for (int i = 0; i < tasklist_num; i++) {
                if (Price[i] < GlobalPrice3_2[i]) {
                    Price[i] = GlobalPrice3_2[i];
                    Bidder[i] = GlobalBidder3_2[i];
                }
            }
            if (AssignedTask != -1) {
                if (Price[AssignedTask] == GlobalPrice3_2[AssignedTask]) {
                    Bidder[AssignedTask] = GlobalBidder3_2[AssignedTask];
                }
            }
        } else {
            // 局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < tasklist_num; i++) {
                Price.push_back(GlobalPrice3_2[i]);
                Bidder.push_back(GlobalBidder3_2[i]);
            }
        }
    } else {
        // 全局价格GlobalPrice3_2为空
        // cout << "全局价格GlobalPrice3_2为空" << endl;
    }

    // 读取全局所有机器人价格GlobalAllRobotPrice1_2
    if (!GlobalAllRobotPrice1_2.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice1_2非空
        if (!AllRobotPrice.empty()) {
            // 局部所有机器人价格非空
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice1_2[i][j]) {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice1_2[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder1_2[i][j];
                    }
                }
            }
        } else {
            // 局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice1_2[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice1_2为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        cout << "全局所有机器人价格GlobalAllRobotPrice1_2为空" << endl;
    }

    // 读取全局所有机器人价格GlobalAllRobotPrice3_2
    if (!GlobalAllRobotPrice3_2.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice3_2非空
        if (!AllRobotPrice.empty()) {
            // 局部所有机器人价格非空
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice3_2[i][j]) {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice3_2[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder3_2[i][j];
                    }
                }
            }
        } else {
            // 局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice3_2[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice3_2为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        cout << "全局所有机器人价格GlobalAllRobotPrice3_2为空" << endl;
    }

    Mymutex1_2.unlock();
    Mymutex3_2.unlock();
}

/*
 * 读全局价格，Robot[3]向Robot[2]读数据，Robot[3]向Robot[4]读数据
 */
void crobot::readPrice3(int tasklist_num) {
    Mymutex2_3.lock();
    Mymutex4_3.lock();

    // 读取全局价格GlobalPrice2_3
    if (!GlobalPrice2_3.empty()) {
        // 全局价格GlobalPrice2_3非空
        if (!Price.empty()) {
            // 局部价格Price非空
            for (int i = 0; i < tasklist_num; i++) {
                if (Price[i] < GlobalPrice2_3[i]) {
                    Price[i] = GlobalPrice2_3[i];
                    Bidder[i] = GlobalBidder2_3[i];
                }
            }
            if (AssignedTask != -1) {
                if (Price[AssignedTask] == GlobalPrice2_3[AssignedTask]) {
                    Bidder[AssignedTask] = GlobalBidder2_3[AssignedTask];
                }
            }
        } else {
            // 局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < tasklist_num; i++) {
                Price.push_back(GlobalPrice2_3[i]);
                Bidder.push_back(GlobalBidder2_3[i]);
            }
        }
    } else {
        // 全局价格GlobalPrice2_3为空
        // cout << "全局价格GlobalPrice2_3为空" << endl;
    }

    // 读取全局价格GlobalPrice4_3
    if (!GlobalPrice4_3.empty()) {
        // 全局价格GlobalPrice4_3非空
        if (!Price.empty()) {
            // 局部价格Price非空
            for (int i = 0; i < tasklist_num; i++) {
                if (Price[i] < GlobalPrice4_3[i]) {
                    Price[i] = GlobalPrice4_3[i];
                    Bidder[i] = GlobalBidder4_3[i];
                }
            }
            if (AssignedTask != -1) {
                if (Price[AssignedTask] == GlobalPrice4_3[AssignedTask]) {
                    Bidder[AssignedTask] = GlobalBidder4_3[AssignedTask];
                }
            }
        } else {
            // 局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < tasklist_num; i++) {
                Price.push_back(GlobalPrice4_3[i]);
                Bidder.push_back(GlobalBidder4_3[i]);
            }
        }
    } else {
        // 全局价格GlobalPrice4_3为空
        // cout << "全局价格GlobalPrice4_3为空" << endl;
    }

    // 读取全局所有机器人价格GlobalAllRobotPrice2_3
    if (!GlobalAllRobotPrice2_3.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice2_3非空
        if (!AllRobotPrice.empty()) {
            // 局部所有机器人价格非空
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice2_3[i][j]) {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice2_3[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder2_3[i][j];
                    }
                }
            }
        } else {
            // 局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */        
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice2_3[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice2_3为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        cout << "全局所有机器人价格GlobalAllRobotPrice2_3为空" << endl;
    }

    // 读取全局所有机器人价格GlobalAllRobotPrice4_3
    if (!GlobalAllRobotPrice4_3.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice4_3非空
        if (!AllRobotPrice.empty()) {
            // 局部所有机器人价格非空
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice4_3[i][j]) {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice4_3[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder4_3[i][j];
                    }
                }
            }
        } else {
            // 局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice4_3[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice4_3为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        cout << "全局所有机器人价格GlobalAllRobotPrice4_3为空" << endl;
    }

    Mymutex2_3.unlock();
    Mymutex4_3.unlock();
}

/*
 * 读全局价格，Robot[4]向Robot[3]读数据，Robot[4]向Robot[5]读数据
 */
void crobot::readPrice4(int tasklist_num) {
    Mymutex3_4.lock();
    Mymutex5_4.lock();

    // 读取全局价格GlobalPrice3_4
    if (!GlobalPrice3_4.empty()) {
        // 全局价格GlobalPrice3_4非空
        if (!Price.empty()) {
            // 局部价格Price非空
            for (int i = 0; i < tasklist_num; i++) {
                if (Price[i] < GlobalPrice3_4[i]) {
                    Price[i] = GlobalPrice3_4[i];
                    Bidder[i] = GlobalBidder3_4[i];
                }
            }
            if (AssignedTask != -1) {
                if (Price[AssignedTask] == GlobalPrice3_4[AssignedTask]) {
                    Bidder[AssignedTask] = GlobalBidder3_4[AssignedTask];
                }
            }
        } else {
            // 局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < tasklist_num; i++) {
                Price.push_back(GlobalPrice3_4[i]);
                Bidder.push_back(GlobalBidder3_4[i]);
            }
        }
    } else {
        // 全局价格GlobalPrice3_4为空
        // cout << "全局价格GlobalPrice3_4为空" << endl;
    }

    // 读取全局价格GlobalPrice5_4
    if (!GlobalPrice5_4.empty()) {
        // 全局价格GlobalPrice5_4非空
        if (!Price.empty()) {
            // 局部价格Price非空
            for (int i = 0; i < tasklist_num; i++) {
                if (Price[i] < GlobalPrice5_4[i]) {
                    Price[i] = GlobalPrice5_4[i];
                    Bidder[i] = GlobalBidder5_4[i];
                }
            }
            if (AssignedTask != -1) {
                if (Price[AssignedTask] == GlobalPrice5_4[AssignedTask]) {
                    Bidder[AssignedTask] = GlobalBidder5_4[AssignedTask];
                }
            }
        } else {
            // 局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < tasklist_num; i++) {
                Price.push_back(GlobalPrice5_4[i]);
                Bidder.push_back(GlobalBidder5_4[i]);
            }
        }
    } else {
        // 全局价格GlobalPrice5_4为空
        // cout << "全局价格GlobalPrice5_4为空" << endl;
    }

    // 读取全局所有机器人价格GlobalAllRobotPrice3_4
    if (!GlobalAllRobotPrice3_4.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice3_4非空
        if (!AllRobotPrice.empty()) {
            // 局部所有机器人价格非空
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice3_4[i][j]) {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice3_4[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder3_4[i][j];
                    }
                }
            }
        } else {
            // 局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice3_4[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice3_4为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        cout << "全局所有机器人价格GlobalAllRobotPrice3_4为空" << endl;
    }

    // 读取全局所有机器人价格GlobalAllRobotPrice5_4
    if (!GlobalAllRobotPrice5_4.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice5_4非空
        if (!AllRobotPrice.empty()) {
            // 局部所有机器人价格非空
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice5_4[i][j]) {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice5_4[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder5_4[i][j];
                    }
                }
            }
        } else {
            // 局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice5_4[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice5_4为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        cout << "全局所有机器人价格GlobalAllRobotPrice5_4为空" << endl;
    }

    Mymutex3_4.unlock();
    Mymutex5_4.unlock();
}

/*
 * 读全局价格，Robot[5]向Robot[4]读数据
 */
void crobot::readPrice5(int tasklist_num) {
    Mymutex4_5.lock();

    // 读取全局价格GlobalPrice4_5
    if (!GlobalPrice4_5.empty()) {
        // 全局价格GlobalPrice4_5非空
        if (!Price.empty()) {
            // 局部价格Price非空
            for (int i = 0; i < tasklist_num; i++) {
                if (Price[i] < GlobalPrice4_5[i]) {
                    Price[i] = GlobalPrice4_5[i];
                    Bidder[i] = GlobalBidder4_5[i];
                }
            }
            if (AssignedTask != -1) {
                if (Price[AssignedTask] == GlobalPrice4_5[AssignedTask]) {
                    Bidder[AssignedTask] = GlobalBidder4_5[AssignedTask];
                }
            }
        } else {
            // 局部价格Price为空
            /*
             * @remarks 由于在generateValueList开始便给局部变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < tasklist_num; i++) {
                Price.push_back(GlobalPrice4_5[i]);
                Bidder.push_back(GlobalBidder4_5[i]);
            }
        }
    } else {
        // 全局价格GlobalPrice4_5为空
        // cout << "全局价格GlobalPrice4_5为空" << endl;
    }

    // 读取全局所有机器人价格GlobalAllRobotPrice4_5
    if (!GlobalAllRobotPrice4_5.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice4_5非空
        if (!AllRobotPrice.empty()) {
            // 局部所有机器人价格非空
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    if (AllRobotPrice[i][j] <= GlobalAllRobotPrice4_5[i][j]) {
                        AllRobotPrice[i][j] = GlobalAllRobotPrice4_5[i][j];
                        AllRobotBidder[i][j] = GlobalAllRobotBidder4_5[i][j];
                    }
                }
            }
        } else {
            // 局部所有机器人价格为空
            /*
             * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
             */            
            for (int i = 0; i < ROBOTNUM; i++) {
                for (int j = 0; j < tasklist_num; j++) {
                    AllRobotPrice[i].push_back(GlobalAllRobotPrice4_5[i][j]);
                    cout << "Something wrong in it !!" << endl;
                }
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice4_5为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        cout << "全局所有机器人价格GlobalAllRobotPrice4_5为空" << endl;
    }

    Mymutex4_5.unlock();
}

/*
 * 设置价格
 */
void crobot::setPrice(int tasklist_num) {
    // 选择每个机器人子线程对应的全局价格写入函数
    switch (Robot_No) {
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
void crobot::writePrice0(int tasklist_num) {
    Mymutex0_1.lock();

    // 写入全局价格GlobalPrice0_1
    if (!GlobalPrice0_1.empty()) {
        // 全局价格GlobalPrice0_1非空
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice0_1[i] = Price[i];
            GlobalBidder0_1[i] = Bidder[i];
        }
    } else {
        // 全局价格GlobalPrice0_1为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice0_1.push_back(Price[i]);
            GlobalBidder0_1.push_back(Bidder[i]);
        }
    }

    // 写入全局所有机器人价格GlobalAllRobotPrice0_1
    if (!GlobalAllRobotPrice0_1.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice0_1非空
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice0_1[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder0_1[i][j] = AllRobotBidder[i][j];
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice0_1为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice0_1[i].push_back(AllRobotPrice[i][j]);
            }
        }
    }

    Mymutex0_1.unlock();
}

/*
 * 写全局价格，Robot[1]向Robot[0]写数据，Robot[1]向Robot[2]写数据
 */
void crobot::writePrice1(int tasklist_num) {
    Mymutex1_0.lock();
    Mymutex1_2.lock();

    // 写入全局价格GlobalPrice1_0
    if (!GlobalPrice1_0.empty()) {
        // 全局价格GlobalPrice1_0非空
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice1_0[i] = Price[i];
            GlobalBidder1_0[i] = Bidder[i];
        }
    } else {
        // 全局价格GlobalPrice1_0为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice1_0.push_back(Price[i]);
            GlobalBidder1_0.push_back(Bidder[i]);
        }
    }

    // 写入全局价格GlobalPrice1_2
    if (!GlobalPrice1_2.empty()) {
        // 全局价格GlobalPrice1_2非空
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice1_2[i] = Price[i];
            GlobalBidder1_2[i] = Bidder[i];
        }
    } else {
        // 全局价格GlobalPrice1_2为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice1_2.push_back(Price[i]);
            GlobalBidder1_2.push_back(Bidder[i]);
        }
    }

    // 写入全局所有机器人价格GlobalAllRobotPrice1_0
    if (!GlobalAllRobotPrice1_0.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice1_0非空
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice1_0[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder1_0[i][j] = AllRobotBidder[i][j];
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice1_0为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */       
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice1_0[i].push_back(AllRobotPrice[i][j]);
            }
        }
    }

    // 写入全局所有机器人价格GlobalAllRobotPrice1_2
    if (!GlobalAllRobotPrice1_2.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice1_2非空
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice1_2[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder1_2[i][j] = AllRobotBidder[i][j];
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice1_2为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
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
void crobot::writePrice2(int tasklist_num) {
    Mymutex2_1.lock();
    Mymutex2_3.lock();

    // 写入全局价格GlobalPrice2_1
    if (!GlobalPrice2_1.empty()) {
        // 全局价格GlobalPrice2_1非空
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice2_1[i] = Price[i];
            GlobalBidder2_1[i] = Bidder[i];
        }
    } else {
        // 全局价格GlobalPrice2_1为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice2_1.push_back(Price[i]);
            GlobalBidder2_1.push_back(Bidder[i]);
        }
    }

    // 写入全局价格GlobalPrice2_3
    if (!GlobalPrice2_3.empty()) {
        // 全局价格GlobalPrice2_3非空
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice2_3[i] = Price[i];
            GlobalBidder2_3[i] = Bidder[i];
        }
    } else {
        // 全局价格GlobalPrice2_3为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice2_3.push_back(Price[i]);
            GlobalBidder2_3.push_back(Bidder[i]);
        }
    }

    // 写入全局所有机器人价格GlobalAllRobotPrice2_1
    if (!GlobalAllRobotPrice2_1.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice2_1非空
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice2_1[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder2_1[i][j] = AllRobotBidder[i][j];
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice2_1为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice2_1[i].push_back(AllRobotPrice[i][j]);
            }
        }
    }

    // 写入全局所有机器人价格GlobalAllRobotPrice2_3
    if (!GlobalAllRobotPrice2_3.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice2_3非空
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice2_3[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder2_3[i][j] = AllRobotBidder[i][j];
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice2_3为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
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
void crobot::writePrice3(int tasklist_num) {
    Mymutex3_2.lock();
    Mymutex3_4.lock();

    // 写入全局价格GlobalPrice3_2
    if (!GlobalPrice3_2.empty()) {
        // 全局价格GlobalPrice3_2非空
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice3_2[i] = Price[i];
            GlobalBidder3_2[i] = Bidder[i];
        }
    } else {
        // 全局价格GlobalPrice3_2为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice3_2.push_back(Price[i]);
            GlobalBidder3_2.push_back(Bidder[i]);
        }
    }

    // 写入全局价格GlobalPrice3_4
    if (!GlobalPrice3_4.empty()) {
        // 全局价格GlobalPrice3_4非空
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice3_4[i] = Price[i];
            GlobalBidder3_4[i] = Bidder[i];
        }
    } else {
        // 全局价格GlobalPrice3_4为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice3_4.push_back(Price[i]);
            GlobalBidder3_4.push_back(Bidder[i]);
        }
    }

    // 写入全局所有机器人价格GlobalAllRobotPrice3_2
    if (!GlobalAllRobotPrice3_2.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice3_2非空
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice3_2[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder3_2[i][j] = AllRobotBidder[i][j];
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice3_2为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice3_2[i].push_back(AllRobotPrice[i][j]);
            }
        }
    }

    // 写入全局所有机器人价格GlobalAllRobotPrice3_4
    if (!GlobalAllRobotPrice3_4.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice3_4非空
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice3_4[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder3_4[i][j] = AllRobotBidder[i][j];
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice3_4为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
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
void crobot::writePrice4(int tasklist_num) {
    Mymutex4_3.lock();
    Mymutex4_5.lock();

    // 写入全局价格GlobalPrice4_3
    if (!GlobalPrice4_3.empty()) {
        // 全局价格GlobalPrice4_3非空
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice4_3[i] = Price[i];
            GlobalBidder4_3[i] = Bidder[i];
        }
    } else {
        // 全局价格GlobalPrice4_3为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice4_3.push_back(Price[i]);
            GlobalBidder4_3.push_back(Bidder[i]);
        }
    }

    // 写入全局价格GlobalPrice4_5
    if (!GlobalPrice4_5.empty()) {
        // 全局价格GlobalPrice4_3非空
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice4_5[i] = Price[i];
            GlobalBidder4_5[i] = Bidder[i];
        }
    } else {
        // 全局价格GlobalPrice4_5为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice4_5.push_back(Price[i]);
            GlobalBidder4_5.push_back(Bidder[i]);
        }
    }

    // 写入全局所有机器人价格GlobalAllRobotPrice4_3
    if (!GlobalAllRobotPrice4_3.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice4_3非空
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice4_3[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder4_3[i][j] = AllRobotBidder[i][j];
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice4_3为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice4_3[i].push_back(AllRobotPrice[i][j]);
            }
        }
    }

    // 写入全局所有机器人价格GlobalAllRobotPrice4_5
    if (!GlobalAllRobotPrice4_5.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice4_5非空
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice4_5[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder4_5[i][j] = AllRobotBidder[i][j];
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice4_5为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
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
void crobot::writePrice5(int tasklist_num) {
    Mymutex5_4.lock();

    // 写入全局价格GlobalPrice5_4
    if (!GlobalPrice5_4.empty()) {
        // 全局价格GlobalPrice5_4非空
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice5_4[i] = Price[i];
            GlobalBidder5_4[i] = Bidder[i];
        }
    } else {
        // 全局价格GlobalPrice5_4为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < tasklist_num; i++) {
            GlobalPrice5_4.push_back(Price[i]);
            GlobalBidder5_4.push_back(Bidder[i]);
        }
    }

    // 写入全局所有机器人价格GlobalAllRobotPrice5_4
    if (!GlobalAllRobotPrice5_4.empty()) {
        // 全局所有机器人价格GlobalAllRobotPrice5_4非空
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice5_4[i][j] = AllRobotPrice[i][j];
                GlobalAllRobotBidder5_4[i][j] = AllRobotBidder[i][j];
            }
        }
    } else {
        // 全局所有机器人价格GlobalAllRobotPrice5_4为空
        /*
         * @remarks 由于在generateValueList开始便给全局变量赋值，故以下{}不会进入
         */        
        for (int i = 0; i < ROBOTNUM; i++) {
            for (int j = 0; j < tasklist_num; j++) {
                GlobalAllRobotPrice5_4[i].push_back(AllRobotPrice[i][j]);
            }
        }
    }

    Mymutex5_4.unlock();
}

/*
 * 竞拍
 */
void crobot::auction(vector<float> PriceOld, int tasklist_num) {
    if (AssignedTask != -1) {
        if ((Price[AssignedTask] >= PriceOld[AssignedTask]) && Bidder[AssignedTask] != Robot_No) {
            // 原先所竞标的任务价格上涨（或不变），同时原先所竞标的任务并非当前机器人执行
            // 出价
            bidding(tasklist_num);
        } else {
            // 将出价和竞标者写入所有机器人价格中
            for (int i = 0; i < tasklist_num; i++) {
                AllRobotPrice[Robot_No][i] = Price[i];
                AllRobotBidder[Robot_No][i] = Bidder[i];
            }
        }
    } else {
        // 出价
        bidding(tasklist_num);
    }
}

/*
 * 收敛
 */
void crobot::convergence(bool &flag) {
    if (AllRobotBidder[0] == AllRobotBidder[1] &&
        AllRobotBidder[1] == AllRobotBidder[2] &&
        AllRobotBidder[2] == AllRobotBidder[3] &&
        AllRobotBidder[3] == AllRobotBidder[4] &&
        AllRobotBidder[4] == AllRobotBidder[5]
        ) {
        flag = true;
    } else {
        flag = false;
    }
}

/*
 * @deprecated 机器人位置坐标仅在初始化时赋值，此位置坐标意为初始化坐标
 * 更新机器人位置坐标
 * @remark：不建议使用该函数，该函数存在隐患，RobotLocation不应该改变。
 */
[[deprecated]] void crobot::updadteRobotLocation(ctasklist * tasklist) {
    if (AssignedTask != -1) {
        RobotLocation[0] = tasklist->sendTaskQueue(AssignedTask)->EndPoint[0];
        RobotLocation[1] = tasklist->sendTaskQueue(AssignedTask)->EndPoint[1];
    }
}

/*
 * 将中标的任务存入机器人任务执行队列
 */
void crobot::savetoTaskExecutionQueue(ctasklist * tasklist) {
    /*
     * @repair
     * 尝试修复机器人数大于任务数异常bug
     */
    if (AssignedTask != -1) {
        TaskTemplate * tmp = tasklist->sendTaskQueue(AssignedTask);
        if (maxValuePosition != -1) {
            TaskExecutionQueue.insert(TaskExecutionQueue.begin() + maxValuePosition, *tmp);
        } else {
            TaskExecutionQueue.push_back(*tmp);
        }
    }
}

/*
 * 打印分配的任务
 */
void crobot::printAssignedTask(ctasklist * tasklist) {
    /*
     * @repair
     * 尝试修复机器人数大于任务数异常bug
     */
    if (AssignedTask != -1) {
        cout << "机器人编号" << Robot_No
            << "任务点编号：" << tasklist->sendTaskQueue(AssignedTask)->PointNo
            << "任务编号：" << tasklist->sendTaskQueue(AssignedTask)->TaskNo
            << endl;
    } else {
        cout << "机器人编号" << Robot_No << "未分配任务" << endl;
    }
}

/*
 * 发送分配任务的价值
 */
float crobot::sendAssignedTaskValue() {
    return ValueList[AssignedTask];
}

/*
 * 发送剩余任务数量
 */
int crobot::sendResidualNum() {
    return ResidualNum;
}

/*
 * 发送剩余任务
 */
TaskTemplate * crobot::sendResidualTask(ctasklist * tasklist, int i) {
    return tasklist->sendTaskQueue(ResidualTask[i]);
}

/*
 * 重置机器人
 */
void crobot::clearPropertity() {
    ValueList.clear();
    Price.clear();
    Bidder.clear();
    for (int i = 0; i < ROBOTNUM; i++) {
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
int crobot::sendRobotNum() {
    return Robot_No;
}

/*
 * 机器人读任务执行队列线程函数
 */
void readTaskExecutionQueue(crobot * Robot) {
    // cout << "机器人读TEQ子线程启动" << "线程ID：" << syscall(SYS_gettid) << endl;
    // cout << "机器人编号：" << Robot->Robot_No << endl;
    // Robot->setCoorTEQWidth(Robot->CoorCommunicateWidth);    //设置机器人协调通信范围，这里为邻接机器人，即2

    switch (Robot->Robot_No) {
    case 0:
    {
        // robot0向robot1进行协调，向robot[1]读数据
        unique_lock<mutex> lck1_0(TEQrw1_0);
        while (!GloConFlag0_1) {
            conVAR0_1.wait(lck1_0);
            // cout << "机器人" << Robot->Robot_No << "阻塞" << endl;
        }
        Robot->updateTaskExecutionQueue(GlobalTEQ1_0, 0);   //读函数

        // cout << "机器人" << Robot->Robot_No << "读完闭" << endl;

        break;
    }
    case 1:
    {
        // robot1向robot0和robot2进行协调，向robot[0]和robot[2]读数据
        unique_lock<mutex> lck0_1(TEQrw0_1);
        unique_lock<mutex> lck2_1(TEQrw2_1);
        while (!GloConFlag1_0) {
            conVAR1_0.wait(lck0_1);
            // cout << "机器人" << Robot->Robot_No << "阻塞" << endl;
        }
        Robot->updateTaskExecutionQueue(GlobalTEQ0_1, 0);   //读函数
        // cout << "机器人" << Robot->Robot_No << "读一半" << endl;
        while (!GloConFlag1_2) {
            conVAR1_2.wait(lck2_1);
            // cout << "机器人" << Robot->Robot_No << "阻塞" << endl;
        }
        Robot->updateTaskExecutionQueue(GlobalTEQ2_1, 1);   //读函数

        // cout << "机器人" << Robot->Robot_No << "读完闭" << endl;

        break;
    }
    case 2:
    {
        // robot2向robot[1]和robot[3]进行协调，向robot[1]和robot[3]读数据
        unique_lock<mutex> lck1_2(TEQrw1_2);
        unique_lock<mutex> lck3_2(TEQrw3_2);
        while (!GloConFlag2_1) {
            conVAR2_1.wait(lck1_2);
            // cout << "机器人" << Robot->Robot_No << "阻塞" << endl;
        }
        Robot->updateTaskExecutionQueue(GlobalTEQ1_2, 0);   //读函数
        // cout << "机器人" << Robot->Robot_No << "读一半" << endl;
        while (!GloConFlag2_3) {
            conVAR2_3.wait(lck3_2);
            // cout << "机器人" << Robot->Robot_No << "阻塞" << endl;
        }
        Robot->updateTaskExecutionQueue(GlobalTEQ3_2, 1);   //读函数

        // cout << "机器人" << Robot->Robot_No << "读完闭" << endl;
        break;
    }
    case 3:
    {
        // robot3向robot[2]和robot[4]进行协调，向robot[2]和robot[4]读数据
        unique_lock<mutex> lck2_3(TEQrw2_3);
        unique_lock<mutex> lck4_3(TEQrw4_3);
        while (!GloConFlag3_2) {
            conVAR3_2.wait(lck2_3);
            // cout << "机器人" << Robot->Robot_No << "阻塞" << endl;
        }
        Robot->updateTaskExecutionQueue(GlobalTEQ2_3, 0);   //读函数
        // cout << "机器人" << Robot->Robot_No << "读一半" << endl;
        while (!GloConFlag3_4) {
            conVAR3_4.wait(lck4_3);
            // cout << "机器人" << Robot->Robot_No << "阻塞" << endl;
        }
        Robot->updateTaskExecutionQueue(GlobalTEQ4_3, 1);   //读函数

        // cout << "机器人" << Robot->Robot_No << "读完闭" << endl;

        break;
    }
    case 4:
    {
        // robot4向robot[3]和robot[5]进行协调，向robot[3]和robot[5]读数据
        unique_lock<mutex> lck3_4(TEQrw3_4);
        unique_lock<mutex> lck5_4(TEQrw5_4);
        while (!GloConFlag4_3) {
            conVAR4_3.wait(lck3_4);
            // cout << "机器人" << Robot->Robot_No << "阻塞" << endl;
        }
        Robot->updateTaskExecutionQueue(GlobalTEQ3_4, 0);   //读函数
        // cout << "机器人" << Robot->Robot_No << "读一半" << endl;
        while (!GloConFlag4_5) {
            conVAR4_5.wait(lck5_4);
            // cout << "机器人" << Robot->Robot_No << "阻塞" << endl;
        }
        Robot->updateTaskExecutionQueue(GlobalTEQ5_4, 1);   //读函数

        // cout << "机器人" << Robot->Robot_No << "读完闭" << endl;

        break;
    }
    case 5:
    {
        // robot5向robot[4]进行协调，向robot[4]读数据
        unique_lock<mutex> lck4_5(TEQrw4_5);
        while (!GloConFlag5_4) {
            conVAR5_4.wait(lck4_5);
            // cout << "机器人" << Robot->Robot_No << "阻塞" << endl;
        }
        Robot->updateTaskExecutionQueue(GlobalTEQ4_5, 0);   //读函数

        // cout << "机器人" << Robot->Robot_No << "读完闭" << endl;

        break;
    }
    default:
        break;
    }
}

/*
 * 机器人写任务执行队列线程函数
 */
void writeTaskExecutionQueue(crobot * Robot) {
    // cout << "机器人写TEQ子线程启动" << "线程ID：" << syscall(SYS_gettid) << endl;
    // cout << "机器人编号：" << Robot->Robot_No << endl;

    // sleep(5);
    switch (Robot->Robot_No) {
    case 0:
    {
        // robot[0]向robot[1]写数据，robot[1]进行协调
        unique_lock<mutex> lck0_1(TEQrw0_1);
        GlobalTEQ0_1 = Robot->setTaskExecutionQueue();  //写函数
        GloConFlag1_0 = true;
        conVAR1_0.notify_all();

        break;
    }
    case 1:
    {
        // robot[1]向robot[0]和robot[2]写数据，robot[0]和robot[2]进行协调
        unique_lock<mutex> lck1_0(TEQrw1_0);
        unique_lock<mutex> lck1_2(TEQrw1_2);
        GlobalTEQ1_0 = Robot->setTaskExecutionQueue();  //写函数
        GloConFlag0_1 = true;
        conVAR0_1.notify_all();
        GlobalTEQ1_2 = Robot->setTaskExecutionQueue();  //写函数
        GloConFlag2_1 = true;
        conVAR2_1.notify_all();

        break;
    }
    case 2:
    {
        // robot[2]向robot[1]和robot[3]写数据，robot[1]和robot[3]进行协调
        unique_lock<mutex> lck2_1(TEQrw2_1);
        unique_lock<mutex> lck2_3(TEQrw2_3);
        GlobalTEQ2_1 = Robot->setTaskExecutionQueue();  //写函数
        GloConFlag1_2 = true;
        conVAR1_2.notify_all();
        GlobalTEQ2_3 = Robot->setTaskExecutionQueue();  //写函数
        GloConFlag3_2 = true;
        conVAR3_2.notify_all();

        break;
    }
    case 3:
    {
        // robot3向robot[2]和robot[4]写数据，robot[2]和robot[4]进行协调
        // unique_lock<mutex> lck3_2(TEQrw3_2);
        // unique_lock<mutex> lck3_4(TEQrw3_4);
        GlobalTEQ3_2 = Robot->setTaskExecutionQueue();  //写函数
        GloConFlag2_3 = true;
        conVAR2_3.notify_all();
        GlobalTEQ3_4 = Robot->setTaskExecutionQueue();  //写函数
        GloConFlag4_3 = true;
        conVAR4_3.notify_all();

        break;
    }
    case 4:
    {
        // robot4向robot[3]和robot[5]写数据，robot[3]和robot[5]进行协调
        unique_lock<mutex> lck4_3(TEQrw4_3);
        unique_lock<mutex> lck4_5(TEQrw4_5);
        GlobalTEQ4_3 = Robot->setTaskExecutionQueue();  //写函数
        GloConFlag3_4 = true;
        conVAR3_4.notify_all();
        GlobalTEQ4_5 = Robot->setTaskExecutionQueue();  //写函数
        GloConFlag5_4 = true;
        conVAR5_4.notify_all();

        break;
    }
    case 5:
    {
        // robot5向robot[4]写数据，robot[4]进行协调
        unique_lock<mutex> lck5_4(TEQrw5_4);
        GlobalTEQ5_4 = Robot->setTaskExecutionQueue();  //写函数
        GloConFlag4_5 = true;
        conVAR4_5.notify_all();

        break;
    }
    default:
        break;
    }
}

/*
 * 设置任务执行队列
 */
vector<TaskTemplate> crobot::setTaskExecutionQueue() {
    return TaskExecutionQueue;
}

/*
 * 设置机器人CoorTEQ个数（通信范围），这里为邻接机器人，即为2
 */
void crobot::setCoorTEQWidth(int CoorCommunicateWidth) {
    CoorTEQ = new vector<TaskTemplate>[CoorCommunicateWidth];
    NewCoorTEQ = new vector<TaskTemplate>[CoorCommunicateWidth];
}

/*
 * 回收CoorTEQWidth内存
 */
void crobot::deleteCoorTEQWidth() {
    delete[] CoorTEQ;
    delete[] NewCoorTEQ;
}

/*
 * 更新任务执行队列
 */
void crobot::updateTaskExecutionQueue(vector<TaskTemplate> GlobalTEQ, int coor_num) {
    CoorTEQ[coor_num] = GlobalTEQ;
}

/*
 * 多机器人协调策略
 */
void crobot::multirobotCoordination(int CoorCommunicateLength) {
    //按顺序对每一个协调对象进行任务协调
    int CoorTaskNum_0 = CoorTEQ[0].size();
    if (CoorTaskNum_0 > CoorCommunicateLength + 1) {
    // 协调对象任务数需至少为通信协调长度+1，如若CoorCommunicateLength为2，CoorTEQ[0]长度至少为4，才能满足任务协调效用的计算
        // 计算机器人任务协调效用（任务协调长度内）
        cout << "第一次协调对象任务执行队列长度为" << CoorTaskNum_0 << endl;

        cmultirobotCoordinate MulriRobotCoordinate0(CoorTEQ[0], TaskExecutionQueue, CoorCommunicateLength, Robot_No);
        MulriRobotCoordinate0.taskCoordinate();
        NewCoorTEQ[0] = MulriRobotCoordinate0.sendNewCoorTEQ();

        TaskExecutionQueue = MulriRobotCoordinate0.sendNewCurrentTEQ();
    } else {
        // 机器人不进行任务协调
        cout << "协调对象任务执行队列小于协调长度，不进行协调！" << endl;
    }

    int CoorTaskNum_1 = CoorTEQ[1].size();
    if (CoorTaskNum_1 > CoorCommunicateLength + 1) {
        cout << "第二次协调对象任务执行队列长度为" << CoorTaskNum_1 << endl;

        cmultirobotCoordinate MulriRobotCoordinate1(CoorTEQ[1], TaskExecutionQueue, CoorCommunicateLength, Robot_No);
        MulriRobotCoordinate1.taskCoordinate();
        NewCoorTEQ[1] = MulriRobotCoordinate1.sendNewCoorTEQ();

        TaskExecutionQueue = MulriRobotCoordinate1.sendNewCurrentTEQ();
    }
}

/*
 * 设置NewCoorTEQ
 */
vector<TaskTemplate> crobot::setNewCoorTEQ(int i) {
    return NewCoorTEQ[i];
}

/*
 * 更新NewCoorTEQ
 */
void crobot::updateNewCoorTEQ(vector<TaskTemplate> newCoorTEQ) {
    TaskExecutionQueue = newCoorTEQ;
}

/*
 * 返回任务执行列表长度
 */
int crobot::getTaskExecutionQueueLength() {
    return TaskExecutionQueue.size();
}

/*
 * 返回CoorTEQ总数
 */
int crobot::getCoorCommunicateWidth() {
    return CoorCommunicateWidth;
}

/*
 * 返回CoorTEQ长度
 */
int crobot::getCoorTEQLength(int i) {
    return CoorTEQ[i].size();
}

/*
 * 返回TaskExecutionQueue执行总距离
 */
float crobot::sendTEQDistance() {
    float tmpDistance = 0;
    float Distance = 0;

    /*
     * @repair
     * 尝试修复机器人数大于任务数异常bug
     */
    if (TaskExecutionQueue.size()) {
        Distance = sqrt(pow(TaskExecutionQueue[0].BeginPoint[0] - RobotLocation[0], 2) +
            pow(TaskExecutionQueue[0].BeginPoint[1] - RobotLocation[1], 2)) +
            sqrt(pow(TaskExecutionQueue[0].EndPoint[0] - TaskExecutionQueue[0].BeginPoint[0], 2) +
            pow(TaskExecutionQueue[0].EndPoint[1] - TaskExecutionQueue[0].BeginPoint[1], 2));
        for (int i = 1; i < TaskExecutionQueueNum; i++) {
            tmpDistance = sqrt(pow(TaskExecutionQueue[i].BeginPoint[0] - TaskExecutionQueue[i - 1].EndPoint[0], 2) +
                pow(TaskExecutionQueue[i].BeginPoint[1] - TaskExecutionQueue[i - 1].EndPoint[1], 2)) +
                sqrt(pow(TaskExecutionQueue[i].EndPoint[0] - TaskExecutionQueue[i].BeginPoint[0], 2) +
                pow(TaskExecutionQueue[i].EndPoint[1] - TaskExecutionQueue[i].BeginPoint[1], 2));
            Distance += tmpDistance;    // 累积价值
        }
    }

    return Distance;
}

/*
 * 返回机器人任务执行效率
 */
int crobot::sendRate() {
    return Rate;
}

/*
 * 接收任务执行队列
 */
void crobot::getTaskExecutionQueue(vector<TaskTemplate> TEQ) {
    TaskExecutionQueue = TEQ;
}
