/***
 *robotcontroller.h - declarations for 7-DOF robot
 *
 *
 *Purpose:
 *       This file defines the structures, values, and functions
 *       used to manage the cycles and the control loop of the 
 *       robotic system.
 *
 *       [Public]
 *
 ****/
#ifndef _ROBOTCONTROLLER_H_
#define _ROBOTCONTROLLER_H_

#include "strdef.h"
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define _USE_MATH_DEFINES
#define MAXBUFLEN 512

typedef int SOCKET;
typedef sockaddr_in SOCKADDR_IN;

class RobotController{

  private:

    //Communication information
    std::string RobAddress;
    unsigned short Port;
    unsigned long triesCounter;

    bool initialized;

    char str[10];

    //Data packets
    MXTCMD MXTsend;
    MXTCMD MXTrecv;

    //Position/orientation variables
    POSE m_target_Pose;
    POSE m_current_Pose;
    POSE m_last_working_Pose;
    JOINT m_target_Joint;
    JOINT m_current_Joint;

    //Socket variables
    SOCKET destSocket;
    SOCKADDR_IN destSockAddr;
    unsigned long destAddr;
    fd_set SockSet;
    timeval sTimeOut;

    //Bounds variables
    float maxIncrement[6];
    float jointBounds[6];
    float workspaceBounds[6];

    //Inaccessible packet and socket functions
    int SendPacket(void);
    int ReceivePacket(void);
    int InitializeSocket(void);

  public:
    RobotController(void);
    int Init(void);
    ~RobotController(void);

    //Get parameters
    void GetCurrentCmdPose(float[6]);
    void GetCurrentActPose(float[6]);
    void GetCurrentOrientation(float[3]);
    void GetCurrentPosition(float[3]);
    void GetIncrements(float[6]);

    //Set parameters
    void SetMaxIncrements(float[6]);
    void SetMaxXYZBounds(float[6]);
    void SetMaxJointBounds(float[6]);

    //Move calls
    void MoveTo(float, float, float, float, float, float);
    void MoveIncTo(float new_xyzcoord[3]);
    void MoveStraightTo(float xyz[3]);
    void MoveStraightToXYOnly(float xyz[3]);
    void RotateStraightTo(float abc[3]);
    void MoveXYZfromIncs(float xyz_increms[3]);
    void MoveXYZfromIncs2(float deltas[3]);
    void RotateABCfromIncs(float abc_increms[3]);
    int MoveToHomePoint(float, float, float, float, float, float);
    int MoveToHomePoint(float[3]);

    //Validate calls
    void UpdateCurrentRobotPose(void);

    static void Sleep(time_t duration);

};



#endif
