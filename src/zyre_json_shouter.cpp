#include "utility.hpp"

pid_t getProcessPidByName(const char *proc_name);
std::time_t getTimeStamp();
string getMessage(robot_test Robot_test);

int main(int argc, char *argv[])
{   
    //get process PID
    pid_t MyprocPid = getProcessPidByName("/home/hanxinyan/Distributed_Task_Assignment_Coordination_Algorithm_Based_on_Market_Mechanism/bin/ZYRE_JSON_SHOUTEROUT");
    cout << "shouter进程pid" << MyprocPid << endl;

    robot_test Robot_test;
    Robot_test.Robot_No = 1;
    Robot_test.RobotLocation[0] = 1;
    Robot_test.RobotLocation[1] = 1;

    string robot_msg  = getMessage(Robot_test);

    cout << "Json:" << robot_msg << endl;

    if (argc < 2)
    {
        printf("Usage: ./ZYRE_JSON_SHOUTEROUT <group name>\n");
        return 1;
    }

    char * group_name = argv[1];
    time_t timestampbegin;
    time_t timestampend;

    //creat a node
    zyre_t *node = zyre_new("shouter");
    assert(node);
    assert(streq(zyre_name(node), "shouter"));

    //this send a ENTER message
    zyre_start(node);
    timestampbegin = getTimeStamp();
    cout << " shouter时间 " << timestampbegin << endl;

    //this send a JOIN message
    int rc = zyre_join(node, group_name);
    assert(rc == 0);

    //wait for a while
    zclock_sleep(250);

    //print UUID of node 
    cout << "shouter UUID: " << zyre_uuid(node) << endl;

    for (int i = 0; i < 5; i++)
    {
        //this send a SHOUT message
        zyre_shouts(node, group_name, "%s", robot_msg.c_str());

        zclock_sleep(1000);
    }

    //this sends a LEAVE message
    zyre_leave(node, group_name);
    timestampend = getTimeStamp();
    cout << " shouter2时间 " << timestampend << endl;

    //this send a EXIT message
    zyre_stop(node);

    //wait for node to stop
    zclock_sleep(100);
    zyre_destroy(&node);

    return 0;
}