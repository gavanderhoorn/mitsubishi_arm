"""This module contains the protocol definition for real-time control of the
Mitsubishi robot. It also contains a class for TCP/UDP socket communication
with the robot controller"""

import socket
from cffi import FFI
MXT_FFI = FFI()
MXT_FFI.cdef("""
//************************************************************************************
// Real-time control sample program
// Communication packet data structure definition header file
//************************************************************************************
// strdef.h
// If the software version of the controller is H7 or later, validate the following define line.
// If the version is H6 or earlier, make the following line the comment. (invalid).
/*************************************************************************/
/* Joint coordinate system (Set unused axis to 0) */
/* Refer to the instruction manual enclosed */
/* with each robot for details on each element. */
/*************************************************************************/
typedef struct{
	float j1;                   // J1 axis angle (radian)
	float j2;                   // J2 axis angle (radian)
	float j3;                   // J3 axis angle (radian)
	float j4;                   // J4 axis angle (radian)
	float j5;                   // J5 axis angle (radian)
	float j6;                   // J6 axis angle (radian)
	float j7;                   // Additional axis 1 (J7 axis angle) (radian)
	float j8;                   // Additional axis 2 (J8 axis angle) (radian)
} JOINT;

/*************************************************************************/
/* XYZ coordinate system (Set unused axis to 0) */
/* Refer to the instruction manual enclosed */
/* with each robot for details on each element. */
/*************************************************************************/
typedef struct{
	float x;                    // X axis coordinate value (mm)
	float y;                    // Y axis coordinate value (mm)
	float z;                    // Z axis coordinate value (mm)
	float a;                    // A axis coordinate value (radian)
	float b;                    // B axis coordinate value (radian)
	float c;                    // C axis coordinate value (radian)
	float l1;                   // Additional axis 1 (mm or radian)
	float l2;                   // Additional axis 2 (mm or radian)
} WORLD;

typedef struct{
	WORLD w;
	uint32_t sflg1;         // Structural flag 1
	uint32_t sflg2;         // Structural flag 2
} POSE;

/*************************************************************************/
/* Pulse coordinate system (Set unused axis to 0) */
/* These coordinates express each joint */
/* with a motor pulse value. */
/*************************************************************************/
typedef struct{
	int32_t p1;                    // Motor 1 axis
	int32_t p2;                    // Motor 2 axis
	int32_t p3;                    // Motor 3 axis
	int32_t p4;                    // Motor 4 axis
	int32_t p5;                    // Motor 5 axis
	int32_t p6;                    // Motor 6 axis
	int32_t p7;                    // Additional axis 1 (Motor 7 axis)
	int32_t p8;                    // Additional axis 2 (Motor 8 axis)
} PULSE;


/************************************************************/
/* Real-time function communication data packet */
/************************************************************/
typedef struct enet_rtcmd_str {
	uint16_t Command;     // Command
#define MXT_CMD_NULL 0          // Real-time external command invalid
#define MXT_CMD_MOVE 1          // Real-time external command valid
#define MXT_CMD_END 255         // Real-time external command end
	uint16_t SendType;    // Command data type designation
	uint16_t RecvType;    // Monitor data type designation
	//////////// Command or monitor data type ///
#define MXT_TYP_NULL 0          // No data
	// For the command and monitor ////////////////////
#define MXT_TYP_POSE  1          // XYZ data
#define MXT_TYP_JOINT 2         // Joint data
#define MXT_TYP_PULSE 3         // pulse data
	///////////// For position related monitor ///
#define MXT_TYP_FPOSE    4         // XYZ data (after filter process)
#define MXT_TYP_FJOINT   5        // Joint data (after filter process)
#define MXT_TYP_FPULSE   6        // Pulse data (after filter process)
#define MXT_TYP_FB_POSE  7       // XYZ data (Encoder feedback value) <H7A>
#define MXT_TYP_FB_JOINT 8      // Joint data (Encoder feedback value) <H7A>
#define MXT_TYP_FB_PULSE 9      // Pulse data (Encoder feedback value) <H7A>
	// For current related monitors //////////////////// <H7A>
#define MXT_TYP_CMDCUR 10       // Electric current command <H7A>
#define MXT_TYP_FBKCUR 11       // Electric current feedback <H7A>
	uint16_t reserve;     // Reserved
	union rtdata {              // Command data
		POSE pos;               // XYZ type [mm/rad]
		JOINT jnt;              // Joint type [rad]
		PULSE pls;              // Pulse type [pls]
		int32_t lng1[8];           // Integer type [% / non-unit]
	} dat;
	uint16_t SendIOType;  // Send input/output signal data designation
	uint16_t RecvIOType;  // Return input/output signal data designation
#define MXT_IO_NULL 0           // No data
#define MXT_IO_OUT  1           // Output signal
#define MXT_IO_IN   2           // Input signal

	uint16_t BitTop;      // Head bit No.
	uint16_t BitMask;     // Transmission bit mask pattern designation (0x0001-0xffff)
	uint16_t IoData;      // Input/output signal data (0x0000-0xffff)
	uint16_t TCount;      // Timeout time counter value
	uint32_t  CCount;      // Transmission data counter value
	uint16_t RecvType1;   // Reply data-type specification 1 .
	uint16_t reserve1;    // Reserved 1
	union rtdata1 {             // Monitor data 1 .
		POSE pos1;              // XYZ type [mm/rad] .
		JOINT jnt1;             // JOINT type [mm/rad] .
		PULSE pls1;             // PULSE type [mm/rad] .
		int32_t lng1[8];           // Integer type [% / non-unit] .
	} dat1;
	uint16_t RecvType2;   // Reply data-type specification 2 .
	uint16_t reserve2;    // Reserved 2
	union rtdata2 {             // Monitor data 2 .
		POSE pos2;              // XYZ type [mm/rad] .
		JOINT jnt2;             // JOINT type [mm/rad] .
		PULSE pls2;             // PULSE type [mm/rad] or Integer type [% / non-unit].
		int32_t lng2[8];           // Integer type [% / non-unit] .
	} dat2;
	uint16_t RecvType3;   // Reply data-type specification 3 .
	uint16_t reserve3;    // Reserved 3
	union rtdata3 {             // Monitor data 3 .
		POSE pos3;              // XYZ type [mm/rad] .
		JOINT jnt3;             // JOINT type [mm/rad] .
		PULSE pls3;             // PULSE type [mm/rad] or Integer type [% / non-unit].
		int32_t lng3[8];           // Integer type [% / non-unit] .
	} dat3;
} MXTCMD;
""")

class RobotConnection(object):

    """A class that initializes tcp and udp connections to robot"""

    def __init__(self, host="192.168.0.1"):
        """

        :host: host address of the robot controller
        """
        self._host = host
        self._ctrl_port = 10003
        self._mxt_port = 10000
        self._mxt_sock = None

    def mxt_connect(self):
        """Make udp connection to Robot
        :returns: None

        """
        self._mxt_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._mxt_sock.connect((self._host, 10000))

    def mxt_send(self, data):
        """Send data through mxt port

        :data: data to send
        :returns: number of bytes sent

        """
        return self._mxt_sock.send(data)

    def mxt_recv(self, bufsize=4096):
        """Received data from mxt port
        :bufsize: maximum amount of data to receive
        :returns: received data

        """
        return self._mxt_sock.recv(bufsize)

