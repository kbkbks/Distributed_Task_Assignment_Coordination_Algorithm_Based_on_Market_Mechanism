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

    /*
     * 读协调状态，下标0为当前机器人协调状态，下标1和2按字典序存放邻接机器人协调状态
     */
    void readCoorStatus();

    /*
     * 写协调状态
     */
    void writeCoorStatus();

    /*
     * 读CoorTEQ
     */
    void readTEQ();

    /*
     * 写CoorTEQ
     */
    void writeTEQ();    

    /*
     * readNewCoorTEQ
     */
    void readNewCoorTEQ();

    /*
     * 检查协调对象NewCoorTEQ刷新
     */
    void checkObjectNewCoorTEQ();


private:
    crobot * CurrentRobot; 
    vector<bool> CoorStatus;    //协调状态
    bool CurrentCoorStatus; //当前机器人协调状态
    int NewCoorTEQNumber;   //读取NewCoorTEQ次数


};

#endif // ! _CCOORDINATECOMMUNICATION_H_