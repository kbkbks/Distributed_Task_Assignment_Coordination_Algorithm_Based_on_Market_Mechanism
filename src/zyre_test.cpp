#include "zyre_test.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include "json.hpp"
#include <string>

using namespace std;
using json = nlohmann::json;

int main()
{
    char tmp[100];
    person p1;
    p1.age = 33;
    p1.high = 1.80f;
    strncpy(p1.name, "zhangsan", sizeof(p1.name));
    strncpy(p1.school, "hdu", sizeof(p1.school));
    memset(tmp, 0, sizeof(tmp));
    memcpy(tmp, &p1, sizeof(person));
    //tmp[1] = '\n';

    json json_object = {
        {"pi", 3.141},
        {"happy", true}
    };

    json json_person = {
        {"age", p1.age},
        {"name", p1.name},
        {"high", p1.high},
        {"school", p1.school},
        {"nothing", nullptr}
    };

    std::string s = json_object.dump();
    cout << json_object.dump() << endl;
    //cout << json_object["happy"] << endl;

    json j = " { \" happy \":true,\" pi \":3.141} "_json;
    auto json_object2 = json::parse(" { \" happy \":true,\" pi \":3.141} ");
    auto json_object3 = json::parse(s);
    cout << "json_object2" << json_object2 << endl;
    cout << "json_object3" << json_object3 << endl;
    cout <<  "j" << j << endl;

    char my_char[100];
    strcpy(my_char, s.c_str());
    auto json_object4 = json::parse(my_char);
    cout << "json_object4" << json_object4 << endl;



    string person_s = json_person.dump(4);
    char arr[150];
    memset(arr, 0, sizeof(arr));
    strcpy(arr, person_s.c_str());
    cout << json_person << endl;
    cout << json_person.dump(4) << endl;

    cout << "TEST" << endl;
    cout << "TEST 20191122"  << "test" << "more" << endl;
}


