#ifndef _ZYRE_JSON_TEST_H_
#define _ZYRE_JSON_TEST_H_
#include <string>
#include <iostream>
#include <json.hpp>
#include "zyre_json_test.h"
#include "zyre.h"
#include <cassert>
#include <sys/time.h>
#include <chrono>
#include <unistd.h>
#include <cstdio> 
#include <cstdlib>
#include <csignal>
#include "spdlog/spdlog.h"

using namespace std;
using json = nlohmann::json;

typedef struct robot_test{
    int Robot_No;
    float RobotLocation[2];
}robor_test;

typedef struct ZyreMsgContent{
    std::string event;
    std::string peer;
    std::string name;
    std::string group;
    std::string message;
}ZyreMsgContent;

#endif //!_ZYRE_JSON_TEST_H_