#include <iostream>
#include <thread>
#include <vector>

using namespace std;

void generator()
{
    cout << "子线程ID：" << this_thread::get_id() << endl;
}

int main()
{
    cout << "主线程ID：" << this_thread::get_id() << endl;

    vector<thread> ThreadsRobot;
    ThreadsRobot.push_back(thread(&generator));

    ThreadsRobot[0].join();

}