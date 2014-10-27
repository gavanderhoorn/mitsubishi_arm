#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <errno.h>      // Error number definitions

// Netwokring
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <boost/asio.hpp>

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_trajectory_controller/hardware_interface_adapter.h>

#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>
#include <math.h>

// Robot communication protocol
#include <mitsubishi_arm_hardware_interface/strdef.h>

#define PI_ 3.14159265359
#define DEG_TO_RAD PI_/180.0

#define ROBOT_PASSWORD "STORMLAB"
#define MAXBUFLEN 512

bool isEqual(double & a, double & b, double threshold)
{
    return fabs(a-b)<threshold;

}

class MitsubishiArmInterface : public hardware_interface::RobotHW
{
public:
    MitsubishiArmInterface(const std::string& ip_addr, const std::string& ctrl_port, const std::string& mxt_port);
    ~MitsubishiArmInterface();

    bool init();

    int startRobot();
    void stopRobot();



    void readHW();
    void writeHW();

private:
    static const unsigned int NUMBER_OF_JOINTS=6;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;
    std::vector<double> cmd_;
    std::vector<double> pos_;
    std::vector<double> xyz_pos_;
    std::vector<double> vel_;
    std::vector<double> eff_;
    std::vector<double> cmd_previous_;

    //boost::mutex io_mutex;

    std::string host_addr_;
    std::string ctrl_port_;
    std::string mxt_port_;

    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::socket ctrl_socket_;
    boost::asio::ip::udp::socket mxt_socket_;
    boost::asio::ip::udp::endpoint host_endpoint_;

    bool robot_started_;

    int initializeSockets();
    bool init(hardware_interface::JointStateInterface &jnt_state_interface,
              hardware_interface::PositionJointInterface &jnt_pos_interface);

};

