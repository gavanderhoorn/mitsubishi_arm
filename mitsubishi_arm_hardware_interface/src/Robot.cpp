#include <controller_manager/controller_manager.h>
#include <mitsubishi_arm_hardware_interface/MitsubishiArmInterface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mitsubishi_arm_hardware_interface");
    ros::NodeHandle node;

    std::string host_addr;
    std::string ctrl_port;
    std::string mtx_port;

    node.param<std::string>("mitsubishi_host", host_addr, "192.168.0.1");
    node.param<std::string>("mitsubishi_mtx_port", mtx_port, "10000"); // real-time control
    node.param<std::string>("mitsubishi_port", ctrl_port, "10003"); // control servo

    ROS_INFO_STREAM("IP " << host_addr << " Ports " << ctrl_port << " " << mtx_port);
    MitsubishiArmInterface robot(host_addr, ctrl_port, mtx_port);

    if (!robot.init())
    {
      ROS_FATAL_STREAM("Exiting because robot couldn't be started");
      return -1;
    }

    controller_manager::ControllerManager cm(&robot, node);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time previous=ros::Time::now();

    ros::Rate rate(10.0);
    while (ros::ok())
    {
        ros::Duration period;
        robot.readHW();
        ros::Time now=ros::Time::now();
        period=now-previous;
        //std::cout << "period:"<<period<<std::endl;
        cm.update(now, period);
        previous = now;
        robot.writeHW();
        rate.sleep();
    }

    spinner.stop();

    return 0;
}


