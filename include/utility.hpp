#ifndef _UTILITY_HPP_
#define _UTILITY_HPP_
#include "zyre_json_test.h"

//get the timestamp, on the order of milliseconds
std::time_t getTimeStamp()
{
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());//获取当前时间点
    std::time_t timestamp =  tp.time_since_epoch().count(); //计算距离1970-1-1,00:00的时间长度
    return timestamp;
}

//get process PID
pid_t getProcessPidByName(const char *proc_name)
{
    FILE *fp;
    char buf[100];
    char cmd[200] = {'\0'};
    pid_t pid = -1;
    sprintf(cmd, "pidof %s", proc_name);
    if((fp = popen(cmd, "r")) != NULL)     {
        if(fgets(buf, 255, fp) != NULL)
        {
            pid = atoi(buf);
        }
    }
    printf("pid = %d \n", pid);
    pclose(fp);
    return pid;
}

string getMessage(robot_test Robot_test)
{
    json robot_json = {
        {"Robot_no", Robot_test.Robot_No},
        {"RobotLocation", {Robot_test.RobotLocation[0], Robot_test.RobotLocation[1]}}
    };

    cout <<  "message:" << robot_json.dump(4) << endl;
    
    return robot_json.dump(4);
}


#endif // !_UTILITY_HPP_