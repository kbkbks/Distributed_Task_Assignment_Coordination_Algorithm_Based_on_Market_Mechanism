#ifndef _CCOORDINATECOMMUNICATION_H_
#define _CCOORDINATECOMMUNICATION_H_

#include "define.h"
#include "crobot.h"

class ccoordinatecommunication
{
public:
    ccoordinatecommunication(crobot * Robot);
    ~ccoordinatecommunication(){}

    /*
     * 机器人协调通信
     */
    void enterCoordinate();

    /*
     * 状态检测
     */
    bool checkCoorStatus();

private:
    //bool CoorStatus[ROBOTNUM];
    vector<bool> CoorStatus;    //协调状态

};

#endif // ! _CCOORDINATECOMMUNICATION_H_