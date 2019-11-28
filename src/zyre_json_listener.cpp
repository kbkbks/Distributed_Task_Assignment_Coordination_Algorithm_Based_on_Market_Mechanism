#include "utility.hpp"

ZyreMsgContent* zmsgToZyreMsgContent(zmsg_t *msg)
{
    string sevent, speer, sname, sgroup, smessage;
    char *event, *peer, *name, *group, *message;
    event = zmsg_popstr(msg);
    peer = zmsg_popstr(msg);
    name = zmsg_popstr(msg);
    group = zmsg_popstr(msg);
    message = zmsg_popstr(msg);

    sevent = (event == nullptr) ? "" : event;
    speer = (peer == nullptr) ? "" : peer;
    sname = (name == nullptr) ? "" : name;
    if (sevent == "WHISPER")
    {
        // if event is whisper, the group is empty
        // and the message is the fourth item to be popped
        sgroup = "";
        smessage = (group == nullptr) ? "" : group;
    }
    else
    {
        sgroup = (group == nullptr) ? "" : group;
        smessage = (message == nullptr) ? "" : message;
    }

    free(event);
    free(peer);
    free(name);
    free(group);
    free(message);

    ZyreMsgContent* msg_params = new ZyreMsgContent{sevent, speer, sname, sgroup, smessage};
    return msg_params;
}

int main(int argc, char *argv[])
{
    //get process PID
    pid_t MyprocPid = getProcessPidByName("/home/hanxinyan/Distributed_Task_Assignment_Coordination_Algorithm_Based_on_Market_Mechanism/bin/ZYRE_JSON_LISTENEROUT");
    cout << "listener进程pid" << MyprocPid << endl;

    robot_test Robot_test_recv;

    if (argc < 2)
    {
        printf("Usage: ./ZYRE_JSON_LISTENEROUT <group name>\n");
        return 1;
    }

    char * group_name = argv[1];
    time_t timestampbegin;
    time_t timestampend;

    //creat a new node 
    zyre_t *node = zyre_new("listener");
    assert(node);
    assert(streq(zyre_name(node), "listener"));

    //this sends an ENTER message
    zyre_start(node);

    //this sends a JOIN message
    zyre_join(node, group_name);

    //wait for a while
    zclock_sleep(250);

    //print UUID of node
    cout << "listener UUID: " << zyre_uuid(node) << endl;

    while(true)
    {
        zmsg_t * msg = zyre_recv(node);
        timestampbegin = getTimeStamp();
        cout << " listener2时间 " << timestampbegin << endl;

        //loop through the frames in the message
        while(true)
        {
            //char * msg_str = zmsg_popstr(msg);

            //if (!msg_str)
            //{
            //    break;
            //}

            //cout << "frame: \n" <<  msg_str << endl;
            ZyreMsgContent *msgContent = zmsgToZyreMsgContent(msg);
            if ((!msgContent->message.empty()) && (streq(msgContent->event.c_str(), "SHOUT")))
            {
                //cout << msgContent->message << endl;
                //cout << msgContent->message.c_str() << endl;
                auto msg_recv_json = json::parse(msgContent->message.c_str());
                Robot_test_recv.Robot_No = msg_recv_json["Robot_no"];
                Robot_test_recv.RobotLocation[0] = msg_recv_json["RobotLocation"][0];
                Robot_test_recv.RobotLocation[1] = msg_recv_json["RobotLocation"][1];
            }

            cout << "Robot_No：" << Robot_test_recv.Robot_No << endl;
            cout << "RobotLocation1：" << Robot_test_recv.RobotLocation[0] << endl;
            cout << "RobotLocation2：" << Robot_test_recv.RobotLocation[1] << endl;

            if (msgContent->event.empty())
            {
                break;
            }

            //free(msg_str);
        }
        cout << "------\n" << endl;
        zmsg_destroy(&msg);
    }
    //this sends a LEAVE message
    zyre_leave(node, group_name);
    //this sends an EXIT message
    zyre_stop(node);
    //wait for node to stop
    zclock_sleep(100);
    zyre_destroy(&node);

    return 0;
}