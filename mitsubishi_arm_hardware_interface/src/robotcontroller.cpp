/***************************************************************************************
  RobotController.h - Functions to control the behavior of the robot
****************************************************************************************/

//#include "stdafx.h"
#include "robotcontroller.h"
#include <iostream>
#include <time.h>
#include <string>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define NO_FLAGS_SET  0
#define MAX_ERROR   5
#define MAX_ANGLE_ERROR 9*M_PI/180
#define DO_PRINT  GARRULOUS
#define INVALID_SOCKET  -1
#define SOCKET_ERROR  -1

using namespace std;

clock_t start_time = 0;

RobotController::RobotController(void)
{
  //Initialize robot communication (COMM)
  RobAddress = "192.168.0.1";
  Port = 10000;
  triesCounter = 0;
  initialized = false;
}
int RobotController::Init(){

  //Clear packet data for robot
  memset(&m_target_Pose, 0, sizeof(POSE));
  memset(&m_current_Pose, 0, sizeof(POSE));
  memset(&m_last_working_Pose, 0, sizeof(POSE));
  memset(&m_target_Joint, 0, sizeof(JOINT));
  memset(&m_current_Joint, 0, sizeof(JOINT));
  memset(&MXTsend, 0, sizeof(MXTsend));
  memset(&MXTrecv, 0, sizeof(MXTrecv));

  //Initialize packet
  MXTsend.Command = MXT_CMD_NULL;
  MXTsend.SendType = MXT_TYP_NULL;
  MXTsend.RecvType = MXT_TYP_POSE;
  MXTsend.SendIOType = MXT_IO_NULL;
  MXTsend.RecvIOType = MXT_IO_NULL;
  MXTsend.RecvType1 = MXT_TYP_JOINT;
  MXTsend.RecvType2 = MXT_TYP_NULL;
  MXTsend.RecvType3 = MXT_TYP_NULL;
  MXTsend.BitMask = 0xffff;
  MXTsend.BitTop = 0;
  MXTsend.IoData = 0;
  MXTsend.CCount = 0;
  MXTsend.TCount = 0;

  int ret = InitializeSocket();
  if(ret <1)
  {
    cout<<"Robot is not connected.  Connect robot and restart."<<endl;
    exit(1);
  }
  initialized = true;
  UpdateCurrentRobotPose();

  m_target_Pose = m_current_Pose;
}

int RobotController::InitializeSocket(void)
{

  //IP address, port, etc. setting
  memset(&destSockAddr, 0, sizeof(destSockAddr));
  destAddr = inet_addr(RobAddress.c_str());
  memcpy(&destSockAddr.sin_addr, &destAddr, sizeof(destAddr));
  destSockAddr.sin_port = htons(Port);
  destSockAddr.sin_family = AF_INET;

  //Create socket
  destSocket = socket(AF_INET, SOCK_DGRAM, 0);
  if(destSocket == INVALID_SOCKET)
  {
    cout<<"ERROR in RobotController::Initialize Socket;"<<endl;
    cout<<"Socket creation unsuccessful."<<endl;
    return -1;
  }
}

int RobotController::MoveToHomePoint(float target[3])
{
  //Alias
  //float x = target[0];
  //float y = target[1];
  //float z = target[2];

  //First send a packet that will just return the initial pose and joint angles.
  MXTsend.Command = MXT_CMD_NULL;
  MXTsend.CCount = triesCounter = 0;
  memset(&m_target_Pose, 0, sizeof(POSE));
  memset(&m_target_Joint, 0, sizeof(JOINT));

  int status = SendPacket();
  if(!status)
  {
    cout<<"Error in sending packet for initializing the position!"<<endl;
    return -1;
  }

  status = ReceivePacket();
  if(status<1)
  {
    cout<<"Could not receive packet for initializing the position!"<<endl;
    return -1;
  }

  MoveIncTo(target);

  return 1;
}

void RobotController::MoveIncTo(float target[3])
{
  //Alias
  float x = target[0];
  float y = target[1];
  float z = target[2];

  //Set packet information for future use
  MXTsend.Command = MXT_CMD_MOVE;
  MXTsend.SendType = MXT_TYP_POSE;

  //set increment factors
  float inc[] = {maxIncrement[0], maxIncrement[1], maxIncrement[2],
    maxIncrement[3], maxIncrement[4], maxIncrement[5]};

  //Clear target pose and target joint position
  memset(&m_target_Pose, 0, sizeof(POSE));
  memset(&m_target_Joint, 0, sizeof(JOINT));

  bool isfinished = false;
  float total_error = 99999;

  while(!isfinished)
  {
    m_target_Pose = m_current_Pose;
    float travelVector[3] = {x - m_current_Pose.w.x, y - m_current_Pose.w.y, z - m_current_Pose.w.z};

#if GARRULOUS ==1
    cout<<"IP Goal: x: "<<x<<" y: "<<y<<" z: "<<z<<"."<<endl;
    cout<<"IP Curr: x: "<<m_current_Pose.w.x<<" y: "<<m_current_Pose.w.y<<" z: "<<m_current_Pose.w.z<<endl;
#endif
    //Set the pose with the maximum increment to get there, and still enforce workspace bounds
    if(sqrt(pow(m_current_Pose.w.x - x,2))> maxIncrement[0])
      m_target_Pose.w.x += inc[0]*travelVector[0]/fabs(travelVector[0]);//gives direction of movement
    else
      m_target_Pose.w.x = x;

    if(sqrt(pow(m_current_Pose.w.y - y,2))> maxIncrement[1])
      m_target_Pose.w.y += inc[1]*travelVector[1]/fabs(travelVector[1]);
    else
      m_target_Pose.w.y = y;

    if(sqrt(pow(m_current_Pose.w.z - z,2))> maxIncrement[2])
      m_target_Pose.w.z += inc[2]*travelVector[2]/fabs(travelVector[2]);
    else
      m_target_Pose.w.z = z;

    //Keep end effector joints constant in POSE Packet
    m_target_Pose.w.a = m_current_Pose.w.a;
    m_target_Pose.w.b = m_current_Pose.w.b;
    m_target_Pose.w.c = m_current_Pose.w.c;

#if GARRULOUS==1
    cout<<"MIT IMTarg: x: "<<m_target_Pose.w.x<<" y: "<<m_target_Pose.w.y<<" z: "<<m_target_Pose.w.z<<"."<<endl;
    cout<<"MIT IMDiff: x: "<<m_current_Pose.w.x - m_target_Pose.w.x<<" y: "<<m_current_Pose.w.y - m_target_Pose.w.y<<" z: "<<m_current_Pose.w.z - m_target_Pose.w.z<<"."<<endl;
#endif
    //Send desired pose.
    SendPacket();

    //Receive new pose and joint angles.
    ReceivePacket();

    //Check to see if pose has been reached within reason and exit if so
    total_error = sqrt(pow(x - m_current_Pose.w.x,2) + pow(y - m_current_Pose.w.y,2) + pow(z - m_current_Pose.w.z,2));

    if(total_error<MAX_ERROR)
      isfinished = true;

#if GARRULOUS==1
    cout<<"IP Pos Error: "<<total_error<<".\n"<<endl;
#endif

  }

  return;
}

void RobotController::MoveStraightTo(float target[3])
{
  //Alias
  float x = (float)target[0];
  float y = (float)target[1];
  float z = (float)target[2];

  //Set packet information for future use
  MXTsend.Command = MXT_CMD_MOVE;
  MXTsend.SendType = MXT_TYP_POSE;

  m_target_Pose.w.x = x;
  m_target_Pose.w.y = y;
  m_target_Pose.w.z = z;

  //Keep end effector joints constant in POSE Packet
  m_target_Pose.w.a = m_target_Pose.w.a;
  m_target_Pose.w.b = m_target_Pose.w.b;
  m_target_Pose.w.c = m_target_Pose.w.c;

#if GARRULOUS ==1
  cout<<"MST IMCurr: x: "<<m_current_Pose.w.x<<" y: "<<m_current_Pose.w.y<<" x: "<<m_current_Pose.w.z<<"."<<endl;
  cout<<"MST IMTarg: x: "<<m_target_Pose.w.x<<" y: "<<m_target_Pose.w.y<<" z: "<<m_target_Pose.w.z<<"."<<endl;
  cout<<"MST IMDiff: x: "<<m_current_Pose.w.x - m_target_Pose.w.x<<" y: "<<m_current_Pose.w.y - m_target_Pose.w.y<<" z: "<<m_current_Pose.w.z - m_target_Pose.w.z<<".\n"<<endl;
#endif

  //Send desired pose
  SendPacket();

  //Receive new pose
  ReceivePacket();

  Sleep(100);

  //Validate pose
  UpdateCurrentRobotPose();


}

void RobotController::MoveStraightToXYOnly(float target[3])
{
  //Alias
  float x = target[0];
  float y = target[1];

  //Set packet information for future use
  MXTsend.Command = MXT_CMD_MOVE;
  MXTsend.SendType = MXT_TYP_POSE;

  m_target_Pose.w.x = x;
  m_target_Pose.w.y = y;

  //Keep z and end effector and joints constant in POSE Packet
  m_target_Pose.w.z = m_current_Pose.w.z;

  m_target_Pose.w.a = m_current_Pose.w.a;
  m_target_Pose.w.b = m_current_Pose.w.b;
  m_target_Pose.w.c = m_current_Pose.w.c;

#if GARRULOUS ==1
  cout<<"MST IMCurr: x: "<<m_current_Pose.w.x<<" y: "<<m_current_Pose.w.y<<" x: "<<m_current_Pose.w.z<<"."<<endl;
  cout<<"MST IMTarg: x: "<<m_target_Pose.w.x<<" y: "<<m_target_Pose.w.y<<" z: "<<m_target_Pose.w.z<<"."<<endl;
  cout<<"MST IMDiff: x: "<<m_current_Pose.w.x - m_target_Pose.w.x<<" y: "<<m_current_Pose.w.y - m_target_Pose.w.y<<" z: "<<m_current_Pose.w.z - m_target_Pose.w.z<<".\n"<<endl;
#endif

  //Send desired pose
  SendPacket();

  //Receive new pose
  ReceivePacket();

}

void RobotController::RotateStraightTo(float target[3])
{
  //Alias
  float a = (float)target[0];
  float b = (float)target[1];
  float c = (float)target[2];

  //Set packet information for future use
  MXTsend.Command = MXT_CMD_MOVE;
  MXTsend.SendType = MXT_TYP_POSE;

  //Set new rotation position
  m_target_Pose.w.a = a;
  m_target_Pose.w.b = b;
  m_target_Pose.w.c = c;

  //Keep XYZ coordinates constant
  m_target_Pose.w.x = m_target_Pose.w.x;
  m_target_Pose.w.y = m_target_Pose.w.y;
  m_target_Pose.w.z = m_target_Pose.w.z;

#if GARRULOUS ==1
  cout<<"MST IMCurr: x: "<<m_current_Pose.w.x<<" y: "<<m_current_Pose.w.y<<" x: "<<m_current_Pose.w.z<<"."<<endl;
  cout<<"MST IMTarg: x: "<<m_target_Pose.w.x<<" y: "<<m_target_Pose.w.y<<" z: "<<m_target_Pose.w.z<<"."<<endl;
  cout<<"MST IMDiff: x: "<<m_current_Pose.w.x - m_target_Pose.w.x<<" y: "<<m_current_Pose.w.y - m_target_Pose.w.y<<" z: "<<m_current_Pose.w.z - m_target_Pose.w.z<<".\n"<<endl;
#endif

  //Send desired pose
  SendPacket();

  //Receive new pose
  ReceivePacket();

}

void RobotController::RotateABCfromIncs(float deltas[3])
{
  //Set packet information for future use
  MXTsend.Command = MXT_CMD_MOVE;
  MXTsend.SendType = MXT_TYP_POSE;

  //Increment a,b,c as desired
  //Keep end effector joints constant in POSE Packet
  m_target_Pose.w.a = m_target_Pose.w.a + (float)deltas[0];
  m_target_Pose.w.b = m_target_Pose.w.b + (float)deltas[1];
  m_target_Pose.w.c = m_target_Pose.w.c + (float)deltas[2];

  //TODO: Implement joint bound limits or some kind of joint checking; or start out at joint degree 0 and continue from there?
  //Calculate end effector roll, M_PItch, and yaw

  //a rotation fixing
  if(m_target_Pose.w.a > M_PI) m_target_Pose.w.a = m_target_Pose.w.a - 2*M_PI;
  if(m_target_Pose.w.a <-M_PI) m_target_Pose.w.a = m_target_Pose.w.a + 2*M_PI;

  //b rotation fixing
  if(m_target_Pose.w.b > M_PI) m_target_Pose.w.b = m_target_Pose.w.b - 2*M_PI;
  if(m_target_Pose.w.b <-M_PI) m_target_Pose.w.b = m_target_Pose.w.b + 2*M_PI;

  //c rotation fixing
  if(m_target_Pose.w.c > M_PI) m_target_Pose.w.c = m_target_Pose.w.c - 2*M_PI;
  if(m_target_Pose.w.c <-M_PI) m_target_Pose.w.c = m_target_Pose.w.c + 2*M_PI;


#if GARRULOUS == 1
  cout<<"IP ROT GOAL: a: "<<deltas[0]<<" b: "<<deltas[1]<<" c: "<<deltas[2]<<endl;
  cout<<"IP ROT Curr: a: "<<m_current_Pose.w.a<<" b: "<<m_current_Pose.w.b<<" c: "<<m_current_Pose.w.c<<endl;
  cout<<"IP ROT Targ: a: "<<m_target_Pose.w.a<<" b: "<<m_target_Pose.w.b<<" c: "<<m_target_Pose.w.c<<endl;
#endif

  //Keep positional joints constant in POSE Packet
  m_target_Pose.w.x = m_target_Pose.w.x;
  m_target_Pose.w.y = m_target_Pose.w.y;
  m_target_Pose.w.z = m_target_Pose.w.z;

  //Send desired pose
  SendPacket();

  //Receive new pose
  ReceivePacket();

}

void RobotController::MoveXYZfromIncs(float deltas[3])
{
  //Set packet information for future use
  MXTsend.Command = MXT_CMD_MOVE;
  MXTsend.SendType = MXT_TYP_POSE;

  //m_target_Pose = m_current_Pose;

  //Increment x, y, and z as desired
  m_target_Pose.w.x = m_target_Pose.w.x + deltas[0];
  m_target_Pose.w.y = m_target_Pose.w.y + deltas[1];
  m_target_Pose.w.z = m_target_Pose.w.z + deltas[2];

  //Bounds checking
  if(m_target_Pose.w.x < workspaceBounds[0])
    m_target_Pose.w.x = workspaceBounds[0];
  if(m_target_Pose.w.x > workspaceBounds[3])
    m_target_Pose.w.x = workspaceBounds[3];

  if(m_target_Pose.w.y < workspaceBounds[1])
    m_target_Pose.w.y = workspaceBounds[1];
  if(m_target_Pose.w.y > workspaceBounds[4])
    m_target_Pose.w.y = workspaceBounds[4];

  if(m_target_Pose.w.z < workspaceBounds[2])
    m_target_Pose.w.z = workspaceBounds[2];
  if(m_target_Pose.w.z > workspaceBounds[5])
    m_target_Pose.w.z = workspaceBounds[5];

  //More bounds checking - to make sure the target point is in the sphere of radius workspaceBounds[3]
  /*if(sqrt(pow(m_target_Pose.w.x,2) + pow(m_target_Pose.w.y,2) + pow((m_target_Pose.w.z-225),2)) > workspaceBounds[3])
    {
    m_target_Pose.w.x = m_current_Pose.w.x;
    m_target_Pose.w.y = m_current_Pose.w.y;
    m_target_Pose.w.z = m_current_Pose.w.z;
    }*/

  //Keep end effector joints constant in POSE Packet
  m_target_Pose.w.a = m_target_Pose.w.a;
  m_target_Pose.w.b = m_target_Pose.w.b;
  m_target_Pose.w.c = m_target_Pose.w.c;



#if GARRULOUS ==1
  cout<<"MXYZI IMTarg: x: "<<m_target_Pose.w.x<<" y: "<<m_target_Pose.w.y<<" z: "<<m_target_Pose.w.z<<"."<<endl;
  cout<<"MXYZI IMDiff: x: "<<m_current_Pose.w.x - m_target_Pose.w.x<<" y: "<<m_current_Pose.w.y - m_target_Pose.w.y<<" z: "<<m_current_Pose.w.z - m_target_Pose.w.z<<".\n"<<endl;
#endif

  //Send desired pose
  SendPacket();

  //Receive new pose
  ReceivePacket();

  UpdateCurrentRobotPose();

}


void RobotController::MoveXYZfromIncs2(float deltas[3])
{
  //Set packet information for future use
  MXTsend.Command = MXT_CMD_MOVE;
  MXTsend.SendType = MXT_TYP_POSE;

  //Increment x, y, and z as desired
  m_target_Pose.w.x = m_current_Pose.w.x + deltas[0];
  m_target_Pose.w.y = m_current_Pose.w.y + deltas[1];
  m_target_Pose.w.z = m_current_Pose.w.z + deltas[2];

  //Bounds checking
  if(m_target_Pose.w.x < workspaceBounds[0])
    m_target_Pose.w.x = workspaceBounds[0];
  if(m_target_Pose.w.x > workspaceBounds[3])
    m_target_Pose.w.x = workspaceBounds[3];

  if(m_target_Pose.w.y < workspaceBounds[1])
    m_target_Pose.w.y = workspaceBounds[1];
  if(m_target_Pose.w.y > workspaceBounds[4])
    m_target_Pose.w.y = workspaceBounds[4];

  if(m_target_Pose.w.z < workspaceBounds[2])
    m_target_Pose.w.z = workspaceBounds[2];
  if(m_target_Pose.w.z > workspaceBounds[5])
    m_target_Pose.w.z = workspaceBounds[5];

  //More bounds checking - to make sure the target point is in the sphere of radius workspaceBounds[3]
  /*if(sqrt(pow(m_target_Pose.w.x,2) + pow(m_target_Pose.w.y,2) + pow((m_target_Pose.w.z-225),2)) > workspaceBounds[3])
    {
    m_target_Pose.w.x = m_current_Pose.w.x;
    m_target_Pose.w.y = m_current_Pose.w.y;
    m_target_Pose.w.z = m_current_Pose.w.z;
    }*/

  //Keep end effector joints constant in POSE Packet
  m_target_Pose.w.a = m_target_Pose.w.a;
  m_target_Pose.w.b = m_target_Pose.w.b;
  m_target_Pose.w.c = m_target_Pose.w.c;



#if GARRULOUS ==1
  cout<<"MXYZI IMTarg: x: "<<m_target_Pose.w.x<<" y: "<<m_target_Pose.w.y<<" z: "<<m_target_Pose.w.z<<"."<<endl;
  cout<<"MXYZI IMDiff: x: "<<m_current_Pose.w.x - m_target_Pose.w.x<<" y: "<<m_current_Pose.w.y - m_target_Pose.w.y<<" z: "<<m_current_Pose.w.z - m_target_Pose.w.z<<".\n"<<endl;
#endif

  //Send desired pose
  SendPacket();

  //Receive new pose
  ReceivePacket();

  Sleep(5);

  UpdateCurrentRobotPose();

}




int RobotController::MoveToHomePoint(float x, float y, float z, float a, float b, float c)
{

  //First send a packet that will just return the initial pose and joint angles.
  MXTsend.Command = MXT_CMD_NULL;
  MXTsend.CCount = triesCounter = 0;
  memset(&m_target_Pose, 0, sizeof(POSE));
  memset(&m_target_Joint, 0, sizeof(JOINT));

  int status = SendPacket();
  if(!status)
  {
    cout<<"Error in sending packet for initializing the position!"<<endl;
    return -1;
  }

  status = ReceivePacket();
  if(status<1)
  {
    cout<<"Could not receive packet for initializing the position!"<<endl;
    return -1;
  }

  //Set packet information for future use
  MXTsend.Command = MXT_CMD_MOVE;
  MXTsend.SendType = MXT_TYP_POSE;

  //set increment factors
  float inc[] = {maxIncrement[0], maxIncrement[1], maxIncrement[2],
    maxIncrement[3], maxIncrement[4], maxIncrement[5]};

  //Clear target pose and target joint position
  memset(&m_target_Pose, 0, sizeof(POSE));
  memset(&m_target_Joint, 0, sizeof(JOINT));

  bool isfinished = false;
  float total_error =99999;
  float old_total_error = 9999999;
  float old_error[6] = {99999,99999,99999,99999,99999,99999};
  float error[6] = {0,0,0,0,0,0};

  while(!isfinished)
  {
    m_target_Pose = m_current_Pose;
    float travelVector[3] = {x - m_current_Pose.w.x, y - m_current_Pose.w.y, z - m_current_Pose.w.z};

#if DO_PRINT ==1
    cout<<"IP Goal: x: "<<x<<" y: "<<y<<" z: "<<z<<"; a: "<<a<<" b: "<<b<<" c: "<<c<<"."<<endl;
    cout<<"IP Curr: x: "<<m_current_Pose.w.x<<" y: "<<m_current_Pose.w.y<<" z: "<<m_current_Pose.w.z<<
      " a: "<<m_current_Pose.w.a<<" b: "<<m_current_Pose.w.b<<" c: "<<m_current_Pose.w.c<<"."<<endl;
#endif
    //Set the pose with the maximum increment to get there, and still enforce workspace bounds
    if(sqrt(pow(m_current_Pose.w.x - x,2))> maxIncrement[0])
      m_target_Pose.w.x += inc[0]*travelVector[0]/fabs(travelVector[0]);//gives direction of movement
    else
      m_target_Pose.w.x = x;

    if(sqrt(pow(m_current_Pose.w.y - y,2))> maxIncrement[1])
      m_target_Pose.w.y += inc[1]*travelVector[1]/fabs(travelVector[1]);
    else
      m_target_Pose.w.y = y;

    if(sqrt(pow(m_current_Pose.w.z - z,2))> maxIncrement[2])
      m_target_Pose.w.z += inc[2]*travelVector[2]/fabs(travelVector[2]);
    else
      m_target_Pose.w.z = z;

    //Calculate end effector roll, pitch, and yaw
    float dir_a, dir_b, dir_c = 0;

    //a rotation
    dir_a = a - m_current_Pose.w.a;
    if(dir_a > M_PI) error[3] = sqrt(pow(dir_a-2*M_PI,2));
    else if(dir_a <-M_PI) error[3] = sqrt(pow(dir_a+2*M_PI,2));
    else error[3]= sqrt(pow(dir_a,2));

    if(error[3] > maxIncrement[3])
    {
      if(fabs(dir_a) > M_PI) dir_a = dir_a*-1;
      m_target_Pose.w.a = m_current_Pose.w.a + inc[3]*dir_a/fabs(dir_a);
      if(m_target_Pose.w.a > M_PI) m_target_Pose.w.a = m_target_Pose.w.a - 2*M_PI;
      if(m_target_Pose.w.a <-M_PI) m_target_Pose.w.a = m_target_Pose.w.a + 2*M_PI;
    }
    else
      m_target_Pose.w.a = a;

    //b rotation
    dir_b = b - m_current_Pose.w.b;
    if(dir_b > M_PI) error[4] = sqrt(pow(dir_b-2*M_PI,2));
    else if(dir_b <-M_PI) error[4] = sqrt(pow(dir_b+2*M_PI,2));
    else error[4]= sqrt(pow(dir_b,2));

    if(error[4] > maxIncrement[4])
    {
      if(fabs(dir_b) > M_PI) dir_b = dir_b*-1;
      m_target_Pose.w.b = m_current_Pose.w.b + inc[4]*dir_b/fabs(dir_b);
      if(m_target_Pose.w.b > M_PI) m_target_Pose.w.b = m_target_Pose.w.b - 2*M_PI;
      if(m_target_Pose.w.b <-M_PI) m_target_Pose.w.b = m_target_Pose.w.b + 2*M_PI;
    }
    else
      m_target_Pose.w.b = b;

    //c rotation
    dir_c = c - m_current_Pose.w.c;
    if(dir_c > M_PI) error[5] = sqrt(pow(dir_c-2*M_PI,2));
    else if(dir_c <-M_PI) error[5] = sqrt(pow(dir_c+2*M_PI,2));
    else error[5]= sqrt(pow(dir_c,2));

    if(error[5] > maxIncrement[5])
    {
      if(fabs(dir_c) > M_PI) dir_c = dir_c*-1;
      m_target_Pose.w.c = m_current_Pose.w.c + inc[5]*dir_c/fabs(dir_c);
      if(m_target_Pose.w.c > M_PI) m_target_Pose.w.c = m_target_Pose.w.c - 2*M_PI;
      if(m_target_Pose.w.c <-M_PI) m_target_Pose.w.c = m_target_Pose.w.c + 2*M_PI;
    }
    else
      m_target_Pose.w.c = c;

    //Keep end effector joints constant in POSE Packet
    //m_target_Pose.w.a = m_current_Pose.w.a;
    //m_target_Pose.w.b = m_current_Pose.w.b;
    //m_target_Pose.w.c = m_current_Pose.w.c;

    //Don't violate joint angles; assume correlation between a,b,c and j4, j5, and j6.
    if(m_current_Joint.j4 <= jointBounds[0] || m_current_Joint.j4 >= jointBounds[3])
    {
      m_target_Pose.w.a = m_current_Pose.w.a;
    }
    if(m_current_Joint.j5 <= jointBounds[1] || m_current_Joint.j5 >= jointBounds[4])
    {
      m_target_Pose.w.b = m_current_Pose.w.b;
    }
    if(m_current_Joint.j6 <= jointBounds[2] || m_current_Joint.j6 >= jointBounds[5])
    {
      m_target_Pose.w.c = m_current_Pose.w.c;
    }

    //Print stuff
#if DO_PRINT==1
    cout<<"IP IMTarg: x: "<<m_target_Pose.w.x<<" y: "<<m_target_Pose.w.y<<" z: "<<m_target_Pose.w.z<<"."<<endl;
    cout<<"IP IMDiff: x: "<<m_current_Pose.w.x - m_target_Pose.w.x<<" y: "<<m_current_Pose.w.y - m_target_Pose.w.y<<" z: "<<m_current_Pose.w.z - m_target_Pose.w.z<<".\n"<<endl;

    cout<<"IP ROT GOAL: a: "<<a<<" b: "<<b<<" c: "<<c<<endl;
    cout<<"IP ROT Curr: a: "<<m_current_Pose.w.a<<" b: "<<m_current_Pose.w.b<<" c: "<<m_current_Pose.w.c<<endl;
    cout<<"IP ROT Targ: a: "<<m_target_Pose.w.a<<" b: "<<m_target_Pose.w.b<<" b: "<<m_target_Pose.w.c<<endl;

    float rot_diff[3];
    rot_diff[0] = m_target_Pose.w.a - m_current_Pose.w.a;
    if(rot_diff[0] >M_PI) rot_diff[0] -= 2*M_PI;
    if(rot_diff[0] <-M_PI) rot_diff[0] += 2*M_PI;
    rot_diff[1] = m_target_Pose.w.b - m_current_Pose.w.b;
    if(rot_diff[1] >M_PI) rot_diff[1] -= 2*M_PI;
    if(rot_diff[1] <-M_PI) rot_diff[1] += 2*M_PI;
    rot_diff[2] = m_target_Pose.w.c - m_current_Pose.w.c;
    if(rot_diff[2] >M_PI) rot_diff[2] -= 2*M_PI;
    if(rot_diff[2] <-M_PI) rot_diff[2] += 2*M_PI;

    cout<<"IP ROT diff: a: "<<rot_diff[0]<<" b: "<<rot_diff[1]<<" c: "<<rot_diff[2]<<endl;
    cout<<"If right, press 1 to keep going."<<endl;
#endif

    //Send desired pose.
    SendPacket();

    //Receive new pose and joint angles.
    ReceivePacket();

    //Check to see if pose has been reached within reason and exit if so
    total_error = sqrt(pow(x - m_current_Pose.w.x,2) + pow(y - m_current_Pose.w.y,2) + pow(z - m_current_Pose.w.z,2));
    //+pow(j4 - m_current_Joint.j4,2) + pow(j5 - m_current_Joint.j5,2) + pow(j6 - m_current_Joint.j6,2));


    float total_angle_error = sqrt(pow(a - m_current_Pose.w.a,2) + pow(b - m_current_Pose.w.b,2) + pow(c - m_current_Pose.w.c,2));

    if(total_error<MAX_ERROR && total_angle_error < MAX_ANGLE_ERROR)
      isfinished = true;
    else
    {
      //Check to see if you've passed your target mark; a,b,c already calculated.
      error[0] = sqrt(pow(x-m_current_Pose.w.x,2));
      error[1] = sqrt(pow(y-m_current_Pose.w.y,2));
      error[2] = sqrt(pow(z-m_current_Pose.w.z,2));

      //Set old error equal to current error for next iteration.
      old_error[0] = error[0];
      old_error[1] = error[1];
      old_error[2] = error[2];
      old_error[3] = error[3];
      old_error[4] = error[4];
      old_error[5] = error[5];
    }

    old_total_error = total_error;
#if DO_PRINT==1
    cout<<"IP Pos Error: "<<total_error<<"."<<endl;
#endif
  }

  memcpy(&m_last_working_Pose, &m_current_Pose, sizeof(POSE));
  return 1;

}

void RobotController::MoveTo(float x, float y, float z, float a, float b, float c)
{
  cout<<"Start move."<<endl;
  float error[6] = {0,0,0,0,0,0};

  float total_error = 999999;
  float old_error[6] = {99999,99999,99999,99999,99999,99999};
  float precision_corrector = 0.001;
  bool isFinished = false;

  float inc[] = {maxIncrement[0], maxIncrement[1], maxIncrement[2], 
    maxIncrement[3], maxIncrement[4], maxIncrement[5]};

  //Clear target pose position
  //memset(&m_target_Pose, 0, sizeof(POSE));

  //Fix command packet
  MXTsend.Command = MXT_CMD_MOVE;
  MXTsend.SendType = MXT_TYP_POSE;

  m_target_Pose = m_current_Pose;
  while(isFinished == false)
  {
    //Determine vector of travel to get from current point to target point.
    float travelVector[3] = {x - m_current_Pose.w.x, y - m_current_Pose.w.y, z - m_current_Pose.w.z};

    //Set the pose with the maximum increment to get there, and still enforce workspace bounds
    if(sqrt(pow(m_current_Pose.w.x - x,2))> maxIncrement[0])
      m_target_Pose.w.x = m_current_Pose.w.x + inc[0]*travelVector[0]/fabs(travelVector[0]);
    else
      m_target_Pose.w.x = x;

    if(sqrt(pow(m_current_Pose.w.y - y,2))> maxIncrement[1])
      m_target_Pose.w.y = m_current_Pose.w.y + inc[1]*travelVector[1]/fabs(travelVector[1]);
    else
      m_target_Pose.w.y = y;

    if(sqrt(pow(m_current_Pose.w.z - z,2))> maxIncrement[2])
      m_target_Pose.w.z = m_current_Pose.w.z + inc[2]*travelVector[2]/fabs(travelVector[2]);
    else
      m_target_Pose.w.z = z;

#if DO_PRINT==1
    cout<<"Branch2: IP CurPos: x: "<<m_current_Pose.w.x<<" y: "<<m_current_Pose.w.y<<" z: "<<m_current_Pose.w.z<<"."<<endl;
    cout<<"Verify: IP ImGoal: x: "<<m_target_Pose.w.x<<" y: "<<m_target_Pose.w.y<<" z: "<<m_target_Pose.w.z<<"."<<endl;
    cout<<"Branch2: FinGoal: x: "<<x<<" y: "<<y<<" z: "<<z<<"."<<endl;
    cout<<"Verify: B2 Diff: x: "<<m_target_Pose.w.x - m_current_Pose.w.x<<" y: "<<m_target_Pose.w.y - m_current_Pose.w.y<<" z: "<<m_target_Pose.w.z - m_current_Pose.w.z<<"."<<endl;
#endif
    //Calculate end effector roll, M_PItch, and yaw
    float dir_a, dir_b, dir_c = 0;

    //Don't violate joint angles; assume correlation between a,b,c and j4, j5, and j6.
    if(m_current_Joint.j4 <= jointBounds[0] || m_current_Joint.j4 >= jointBounds[3])
    {
      m_target_Pose.w.a = m_last_working_Pose.w.a;
      error[3] =0;
    }
    else
    {
      //a rotation
      dir_a = a - m_current_Pose.w.a;
      if(dir_a > M_PI) error[3] = sqrt(pow(dir_a-2*M_PI,2));
      else if(dir_a <-M_PI) error[3] = sqrt(pow(dir_a+2*M_PI,2));
      else error[3]= sqrt(pow(dir_a,2));

      if(error[3] > maxIncrement[3])
      {
        if(fabs(dir_a) > M_PI) dir_a = dir_a*-1;
        m_target_Pose.w.a = m_current_Pose.w.a + inc[3]*dir_a/fabs(dir_a);
        if(m_target_Pose.w.a > M_PI) m_target_Pose.w.a = m_target_Pose.w.a - 2*M_PI;
        if(m_target_Pose.w.a <-M_PI) m_target_Pose.w.a = m_target_Pose.w.a + 2*M_PI;
      }
      else
        m_target_Pose.w.a = a;
    }

    if(m_current_Joint.j5 <= jointBounds[1] || m_current_Joint.j5 >= jointBounds[4])
    {
      m_target_Pose.w.b = m_last_working_Pose.w.b;
      error[4] = 0;
    }
    else
    {
      //b rotation
      dir_b = b - m_current_Pose.w.b;
      if(dir_b > M_PI) error[4] = sqrt(pow(dir_b-2*M_PI,2));
      else if(dir_b <-M_PI) error[4] = sqrt(pow(dir_b+2*M_PI,2));
      else error[4]= sqrt(pow(dir_b,2));

      if(error[4] > maxIncrement[4])
      {
        if(fabs(dir_b) > M_PI) dir_b = dir_b*-1;
        m_target_Pose.w.b = m_current_Pose.w.b + inc[4]*dir_b/fabs(dir_b);
        if(m_target_Pose.w.b > M_PI) m_target_Pose.w.b = m_target_Pose.w.b - 2*M_PI;
        if(m_target_Pose.w.b <-M_PI) m_target_Pose.w.b = m_target_Pose.w.b + 2*M_PI;
      }
      else
        m_target_Pose.w.b = b;
    }

    if(m_current_Joint.j6 <= jointBounds[2] || m_current_Joint.j6 >= jointBounds[5])
    {
      m_target_Pose.w.c = m_last_working_Pose.w.c;
      error[5] = 0;
    }
    else
    {
      //c rotation
      dir_c = c - m_current_Pose.w.c;
      if(dir_c > M_PI) error[5] = sqrt(pow(dir_c-2*M_PI,2));
      else if(dir_c <-M_PI) error[5] = sqrt(pow(dir_c+2*M_PI,2));
      else error[5]= sqrt(pow(dir_c,2));

      if(error[5] > maxIncrement[5])
      {
        if(fabs(dir_c) > M_PI) dir_c = dir_c*-1;
        m_target_Pose.w.c = m_current_Pose.w.c + inc[5]*dir_c/fabs(dir_c);
        if(m_target_Pose.w.c > M_PI) m_target_Pose.w.c = m_target_Pose.w.c - 2*M_PI;
        if(m_target_Pose.w.c <-M_PI) m_target_Pose.w.c = m_target_Pose.w.c + 2*M_PI;
      }
      else
        m_target_Pose.w.c = c;
    }

    //Keep end effector joints constant in POSE Packet
    //m_target_Pose.w.a = m_current_Pose.w.a;
    //m_target_Pose.w.b = m_current_Pose.w.b;
    //m_target_Pose.w.c = m_current_Pose.w.c;

    //forse, if robot pitch is near -90 degrees, the robot starts acting crazy, so, just directly input
    //the position; pitch value will be correct
    /*if((fabs(m_current_Pose.w.b)- PI/2 < 6*PI/180 && fabs(m_current_Pose.w.b)-PI/2 > -6*PI/180) &&
      (fabs(a) - PI/2 < 6*PI/180 && fabs(a) - PI/2 > -6*PI/180))
      {
      m_target_Pose.w.b = -85*PI/180;
      error[4] = 3*PI/180;
      error[4] = 0;
      }*/

#if DO_PRINT==1
    //Print stuff
    cout<<"IP IMTarg: x: "<<m_target_Pose.w.x<<" y: "<<m_target_Pose.w.y<<" z: "<<m_target_Pose.w.z<<"."<<endl;
    cout<<"IP IMDiff: x: "<<m_current_Pose.w.x - m_target_Pose.w.x<<" y: "<<m_current_Pose.w.y - m_target_Pose.w.y<<" z: "<<m_current_Pose.w.z - m_target_Pose.w.z<<".\n"<<endl;

    cout<<"IP ROT GOAL: a: "<<a<<" b: "<<b<<" c: "<<c<<endl;
    cout<<"IP ROT Curr: a: "<<m_current_Pose.w.a<<" b: "<<m_current_Pose.w.b<<" c: "<<m_current_Pose.w.c<<endl;
    cout<<"IP ROT Targ: a: "<<m_target_Pose.w.a<<" b: "<<m_target_Pose.w.b<<" b: "<<m_target_Pose.w.c<<endl;

    float rot_diff[3];
    rot_diff[0] = m_target_Pose.w.a - m_current_Pose.w.a;
    if(rot_diff[0] >M_PI) rot_diff[0] -= 2*M_PI;
    if(rot_diff[0] <-M_PI) rot_diff[0] += 2*M_PI;
    rot_diff[1] = m_target_Pose.w.b - m_current_Pose.w.b;
    if(rot_diff[1] >M_PI) rot_diff[1] -= 2*M_PI;
    if(rot_diff[1] <-M_PI) rot_diff[1] += 2*M_PI;
    rot_diff[2] = m_target_Pose.w.c - m_current_Pose.w.c;
    if(rot_diff[2] >M_PI) rot_diff[2] -= 2*M_PI;
    if(rot_diff[2] <-M_PI) rot_diff[2] += 2*M_PI;

    cout<<"IP ROT diff: a: "<<rot_diff[0]<<" b: "<<rot_diff[1]<<" c: "<<rot_diff[2]<<endl;
    cout<<"Press F5 to send packet."<<endl;
#endif

    //Send desired pose with more boundary constraints
    if((fabs(m_target_Pose.w.x - m_current_Pose.w.x)-precision_corrector > inc[0]) || (fabs(m_target_Pose.w.y - m_current_Pose.w.y)-precision_corrector > inc[1]) || (fabs(m_target_Pose.w.z - m_current_Pose.w.z)-precision_corrector > inc[2]))
    {
      cout << "ERROR in MoveTo: increment bounds have been violated."<<endl;
      getchar();
      return;
    }
    else if(m_target_Pose.w.x < workspaceBounds[0] || m_target_Pose.w.x > workspaceBounds[3]
        || m_target_Pose.w.y < workspaceBounds[1] || m_target_Pose.w.y > workspaceBounds[4]
        || m_target_Pose.w.z < workspaceBounds[2] || m_target_Pose.w.z > workspaceBounds[5])
    {
      cout <<"ERROR in MoveTo: workspace bounds have been violated. Press Enter"<<endl;
      getchar();
      return;
    }
    else
    {
#if DO_PRINT == 1
      cout<<"Difference in time is: "<<difftime(clock(), start_time)*CLOCKS_PER_SEC<<endl;
#endif
      SendPacket();
      start_time = clock();
    }
    //The last current pose worked; so save it.
    memcpy(&m_last_working_Pose, &m_current_Pose, sizeof(POSE));

    //Receive new pose
    ReceivePacket();

    //Check to see if pose has been reached within reason and exit if so
    total_error = sqrt(pow(x - m_current_Pose.w.x,2) + pow(y - m_current_Pose.w.y,2) + pow(z - m_current_Pose.w.z,2));
    //+pow(j4 - m_current_Joint.j4,2) + pow(j5 - m_current_Joint.j5,2) + pow(j6 - m_current_Joint.j6,2));

    float total_angle_error = sqrt(pow(error[3],2) + pow(error[4],2) + pow(error[5],2));

    if(total_error<MAX_ERROR && total_angle_error < MAX_ANGLE_ERROR)
      isFinished = true;
#if DO_PRINT==1
    cout<<"IP Pos Error: "<<total_error<<".\n"<<endl;
    cout<<"IP Ang Error: "<<total_angle_error<<".\n"<<endl;
#endif
  }

  cout<<"End move."<<endl;
}

void RobotController::UpdateCurrentRobotPose(void)
{
  MXTsend.Command = MXT_CMD_NULL;
  MXTsend.SendType = MXT_TYP_NULL;
  MXTsend.CCount = triesCounter = 0;
  //memset(&m_target_Pose, 0, sizeof(POSE));
  //memset(&m_target_Joint, 0, sizeof(JOINT));

  int status = SendPacket();
  if(!status)
  {
    cout<<"Error in sending packet for initializing the position!"<<endl;
  }

  status = ReceivePacket();
  if(status<1)
  {
    cout<<"Could not receive packet for initializing the position!"<<endl;
  }

  //Reset back to normal type
  MXTsend.Command = MXT_CMD_MOVE;
  MXTsend.SendType = MXT_TYP_POSE;

}
void RobotController::SetMaxXYZBounds(float bounds[6])
{
  //Array setup: minx, miny, minz, maxx, maxy, maxz
  workspaceBounds[0] = bounds[0];
  workspaceBounds[1] = bounds[1];
  workspaceBounds[2] = bounds[2];
  workspaceBounds[3] = bounds[3];
  workspaceBounds[4] = bounds[4];
  workspaceBounds[5] = bounds[5];
}

void RobotController::SetMaxJointBounds(float bounds[6])
{
  jointBounds[0] = bounds[0];
  jointBounds[1] = bounds[1];
  jointBounds[2] = bounds[2];
  jointBounds[3] = bounds[3];
  jointBounds[4] = bounds[4];
  jointBounds[5] = bounds[5];

}
void RobotController::SetMaxIncrements(float incs[6])
{
  maxIncrement[0] = incs[0];
  maxIncrement[1] = incs[1];
  maxIncrement[2] = incs[2];
  maxIncrement[3] = incs[3];
  maxIncrement[4] = incs[4];
  maxIncrement[5] = incs[5];
}

int RobotController::SendPacket(void)
{
  //Set sending parameters
  //MXTsend.Command = MXT_CMD_MOVE;
  //MXTsend.SendType = MXT_TYP_POSE; //for XYZ coordinates
  MXTsend.RecvType = MXT_TYP_POSE;
  MXTsend.SendIOType = MXT_TYP_NULL; //no signals desired
  MXTsend.RecvIOType = MXT_TYP_NULL; //no signals desired
  MXTsend.RecvType1 = MXT_TYP_JOINT;
  MXTsend.RecvType2 = MXT_TYP_NULL;
  MXTsend.RecvType3 = MXT_TYP_NULL;
  MXTsend.CCount = 0;

  //Set data pose information
  MXTsend.dat.pos = m_target_Pose;

  //Make, clear, and set a text buffer
  char infoToSend[MAXBUFLEN];
  memset(infoToSend, 0, MAXBUFLEN);
  memcpy(infoToSend, &MXTsend, sizeof(MXTsend));

  //Send information
  int numSent = sendto(destSocket, infoToSend, sizeof(MXTCMD), NO_FLAGS_SET, (sockaddr *) &destSockAddr, sizeof(destSockAddr));
  if(numSent !=sizeof(MXTCMD))
  {
    cout<<"ERROR in RobotController::SendPacket."<<endl;
    cout<<"sendTo unsuccessful."<<endl;

    int status = close(destSocket);
    if(status == SOCKET_ERROR)
    {
      cout<<"ERROR in Robotcontroller::SendPacket."<<endl;
      cout<<"closesocket unsuccessful."<<endl;
    }
    return -1;
  }

  return 1;
}

int RobotController::ReceivePacket(void)
{
  char infoToRecv[MAXBUFLEN];
  int status;
  int numRecvd;

  //Clear buffer
  memset(infoToRecv, 0, MAXBUFLEN);
  int retry = 1;    //number of reception retries
  while(retry)
  {
    FD_ZERO(&SockSet);          //SockSet initialization
    FD_SET(destSocket, &SockSet);   //Socket registration
    sTimeOut.tv_sec = 5;        //Transmission timeout setting(sec)
    sTimeOut.tv_usec = 0;
    status = select(destSocket+1, &SockSet, (fd_set*)NULL, (fd_set*)NULL, &sTimeOut);
    if(status==SOCKET_ERROR)
    {
      cout<<"ERROR in RobotController::ReceivePacket"<<endl;
      cout<<"select Unsuccessful."<<endl;
      return 0;
    }

    //If you have received data before the timeout
    if((status>0) && (FD_ISSET(destSocket, &SockSet) !=0))
    {
      numRecvd = recvfrom(destSocket, infoToRecv, MAXBUFLEN, NO_FLAGS_SET, NULL, NULL);
      if(numRecvd == SOCKET_ERROR)
      {
        cout<<"ERROR in ReceivePacket;"<<endl;
        cout<<"recvfrom unsuccessful."<<endl;
        status = close(destSocket);
        if(status == SOCKET_ERROR)
        {
          cout<<"ERROR in ReceivePacket;"<<endl;
          cout<<"closesocket unsuccessful."<<endl;
        }
        return -1;
      }

      //Update pose information with new reached pose.
      memcpy(&MXTrecv, &infoToRecv, sizeof(MXTrecv));
      memcpy(&m_current_Pose, &MXTrecv.dat.pos, sizeof(POSE));
      //m_current_Pose = MXTrecv.dat.pos;
      //m_current_Joint = MXTrecv.dat1.jnt1;

      //Increment counter for sucessful communication and signal to
      //leave reception loop by decrementing retry.
      //triesCounter++;
      retry = 0;
    }
    else //in the case of a reception timeout
    {
      cout <<"Reception timeout in RobotController::ReceivePacket"<<endl;
      getchar();
      retry--;
      if(retry==0)
      {
        cout<<"Robot is not connected.  Connect robot and restart program."<<endl;
        exit(1);
        return -2;
      }
    }
  }
  return 1;
}

void RobotController::GetCurrentCmdPose(float retPose[6])
{
  retPose[0] = m_target_Pose.w.x;
  retPose[1] = m_target_Pose.w.y;
  retPose[2] = m_target_Pose.w.z;
  retPose[3] = m_target_Pose.w.a;
  retPose[4] = m_target_Pose.w.b;
  retPose[5] = m_target_Pose.w.c;
}

void RobotController::GetCurrentActPose(float retPose[6])
{

  retPose[0] = m_current_Pose.w.x;
  retPose[1] = m_current_Pose.w.y;
  retPose[2] = m_current_Pose.w.z;
  retPose[3] = m_current_Pose.w.a;
  retPose[4] = m_current_Pose.w.b;
  retPose[5] = m_current_Pose.w.c;
}

void RobotController::GetCurrentPosition(float retpsn[3])
{
  retpsn[0] = m_target_Pose.w.x;
  retpsn[1] = m_target_Pose.w.y;
  retpsn[2] = m_target_Pose.w.z;
}

void RobotController::GetCurrentOrientation(float retorntn[3])
{
  retorntn[0] = m_target_Pose.w.a;
  retorntn[1] = m_target_Pose.w.b;
  retorntn[2] = m_target_Pose.w.c;
}

void RobotController::GetIncrements(float retincs[6])
{
  memcpy(retincs, maxIncrement, sizeof(retincs));
}

RobotController::~RobotController(void)
{
  if(initialized){

    std::cout << "Disconnecting from Robot" << std::endl;
    //Send end packet
    MXTsend.Command = MXT_CMD_END;
    MXTsend.SendType = MXT_TYP_NULL;
    MXTsend.RecvType = MXT_TYP_NULL;
    MXTsend.SendIOType = MXT_IO_NULL;
    MXTsend.RecvIOType = MXT_IO_NULL;
    MXTsend.BitMask = 0xffff;
    MXTsend.BitTop = 0;
    MXTsend.IoData = 0;
    MXTsend.CCount = 0;
    MXTsend.TCount = 0;

    //Clear pose
    memset(&m_target_Pose, 0, sizeof(POSE));

    //Send end packet
    SendPacket();

    //Close socket
    int status = close(destSocket);
    if(status == SOCKET_ERROR)
    {
      cout<<"ERROR: Destructor error in closesocket."<<endl;
    }
  }
}

// duration is in milliseconds
void RobotController::Sleep(time_t duration){ 
  struct timespec ts;
  ts.tv_sec = duration/1000;
  ts.tv_nsec = (duration%1000)*1e6;
  nanosleep(&ts, NULL);
}

