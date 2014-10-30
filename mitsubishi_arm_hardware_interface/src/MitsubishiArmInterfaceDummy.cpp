#include <mitsubishi_arm_hardware_interface/MitsubishiArmInterface.h>
#include <sstream>
using namespace boost;
using namespace boost::asio::ip;

MitsubishiArmInterface::MitsubishiArmInterface(const std::string &host_addr, const std::string&  ctrl_port, const std::string& mxt_port):
  host_addr_(host_addr), 
  ctrl_port_(ctrl_port),
  mxt_port_(mxt_port),
  ctrl_socket_(io_service_),
  mxt_socket_(io_service_)
{
    //joint_state_pub=n_priv.advertise<sensor_msgs::JointState>( "joint_states", 1);

    pos_.resize(NUMBER_OF_JOINTS);
    vel_.resize(NUMBER_OF_JOINTS);
    eff_.resize(NUMBER_OF_JOINTS);
    cmd_.resize(NUMBER_OF_JOINTS);


    cmd_=pos_;

    cmd_previous_=cmd_;

  }

MitsubishiArmInterface::~MitsubishiArmInterface() {
}

bool MitsubishiArmInterface::init() {

  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_j1("j1", &pos_[0], &vel_[0], &eff_[0]);
  jnt_state_interface_.registerHandle(state_handle_j1);

  hardware_interface::JointStateHandle state_handle_j2("j2", &pos_[1], &vel_[1], &eff_[1]);
  jnt_state_interface_.registerHandle(state_handle_j2);

  hardware_interface::JointStateHandle state_handle_j3("j3", &pos_[2], &vel_[2], &eff_[2]);
  jnt_state_interface_.registerHandle(state_handle_j3);

  hardware_interface::JointStateHandle state_handle_j4("j4", &pos_[3], &vel_[3], &eff_[3]);
  jnt_state_interface_.registerHandle(state_handle_j4);

  hardware_interface::JointStateHandle state_handle_j5("j5", &pos_[4], &vel_[4], &eff_[4]);
  jnt_state_interface_.registerHandle(state_handle_j5);

  hardware_interface::JointStateHandle state_handle_j6("j6", &pos_[5], &vel_[5], &eff_[5]);
  jnt_state_interface_.registerHandle(state_handle_j6);

  registerInterface(&jnt_state_interface_);

  // connect and register the joint position interface
  hardware_interface::JointHandle pos_handle_j1(jnt_state_interface_.getHandle("j1"), &cmd_[0]);
  jnt_pos_interface_.registerHandle(pos_handle_j1);

  hardware_interface::JointHandle pos_handle_j2(jnt_state_interface_.getHandle("j2"), &cmd_[1]);
  jnt_pos_interface_.registerHandle(pos_handle_j2);

  hardware_interface::JointHandle pos_handle_j3(jnt_state_interface_.getHandle("j3"), &cmd_[2]);
  jnt_pos_interface_.registerHandle(pos_handle_j3);

  hardware_interface::JointHandle pos_handle_j4(jnt_state_interface_.getHandle("j4"), &cmd_[3]);
  jnt_pos_interface_.registerHandle(pos_handle_j4);

  hardware_interface::JointHandle pos_handle_j5(jnt_state_interface_.getHandle("j5"), &cmd_[4]);
  jnt_pos_interface_.registerHandle(pos_handle_j5);

  hardware_interface::JointHandle pos_handle_j6(jnt_state_interface_.getHandle("j6"), &cmd_[5]);
  jnt_pos_interface_.registerHandle(pos_handle_j6);

  registerInterface(&jnt_pos_interface_);

  return true;
}

void MitsubishiArmInterface::readHW() {
  ros::Duration(0.005).sleep();

  // convert to radians and add to state
  /*    for(int i=0; i< pos_.size(); ++i)
        {
        pos_[i]=cmd_[i];
        }*/


  eff_[0]=0.0;
  eff_[1]=0.0;
  eff_[2]=0.0;
  eff_[3]=0.0;
  eff_[4]=0.0;
  eff_[5]=0.0;

  vel_[0]=0.0;
  vel_[1]=0.0;
  vel_[2]=0.0;
  vel_[3]=0.0;
  vel_[4]=0.0;
  vel_[5]=0.0;
}


void MitsubishiArmInterface::writeHW() {
  if(isEqual(cmd_previous_[0],cmd_[0],0.00001)&&
      isEqual(cmd_previous_[1],cmd_[1],0.00001)&&
      isEqual(cmd_previous_[2],cmd_[2],0.00001)&&
      isEqual(cmd_previous_[3],cmd_[3],0.00001)&&
      isEqual(cmd_previous_[4],cmd_[4],0.00001)&&
      isEqual(cmd_previous_[5],cmd_[5],0.00001))
  {
    //std::cout << "new command to write"<< cmd_<< std::endl;
    cmd_previous_=cmd_;
    return;
  }
  cmd_previous_=cmd_;
  for(int i=0; i< pos_.size(); ++i)
  {
    pos_[i]=cmd_[i];
  }
  static int new_command_count=0;
  new_command_count++;
  ROS_INFO_STREAM_THROTTLE(0.1,"New command. Writing to robot");
  ros::Duration(0.005).sleep();
}


