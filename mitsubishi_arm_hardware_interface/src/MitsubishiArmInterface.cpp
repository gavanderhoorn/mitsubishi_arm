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

  pos_.resize(NUMBER_OF_JOINTS);
  vel_.resize(NUMBER_OF_JOINTS);
  eff_.resize(NUMBER_OF_JOINTS);
  cmd_.resize(NUMBER_OF_JOINTS);
  cmd_previous_.resize(NUMBER_OF_JOINTS);
  robot_started_ = false;


  //char buf [256];
  //read( USB, &buf, 1); // CLEAN BUFFER
  //readHW();

  //cmd_=pos_;
  //cmd_previous_=cmd_;
  //// convert to radians and add to state
  //for(int i=0; i< pos_.size(); ++i) {
  //  std::cout << cmd_[i] << std::endl;
  //}


  //std::cout << "Init done!" << '\n';

  //    init();
}

MitsubishiArmInterface::~MitsubishiArmInterface()
{
  if(robot_started_)
  {
    stopRobot();
    mxt_socket_.close();
    ctrl_socket_.close();
  }
}

int MitsubishiArmInterface::initializeSockets(void)
{

  try
  {
    // Create sockets
    // Initialize address structures
    // TCP for ctrl port
    tcp::resolver tcp_resolver(io_service_);
    tcp::resolver::query tcp_query(tcp::v4(), host_addr_, ctrl_port_);
    connect(ctrl_socket_, tcp_resolver.resolve(tcp_query));

    // UDP for mxt Real-time control
    udp::resolver udp_resolver(io_service_);
    udp::resolver::query udp_query(udp::v4(), host_addr_, mxt_port_);
    //host_endpoint_ = *udp_resolver.resolve(udp_query);
    connect(mxt_socket_, udp_resolver.resolve(udp_query));
    //mxt_socket_.open(udp::v4());

  }
  catch (std::exception& e)
  {
    ROS_FATAL_STREAM("Socket creation unsuccessful.");
    return -1;
  }
  return 0;
}

bool MitsubishiArmInterface::init()
{
  if(initializeSockets() != 0){
    return false;
  }
  if (startRobot() != 0)
  {
    return false;
  }

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

int MitsubishiArmInterface::startRobot()
{
  // Just send the password return the result
  char buf[] = ROBOT_PASSWORD;
  boost::asio::write(ctrl_socket_, boost::asio::buffer(buf, strlen(buf)));

  //char reply[MAXBUFLEN];
  boost::asio::streambuf reply;
  boost::asio::read_until(ctrl_socket_, reply,'\r');

  // we have  a valid packet
  std::istream convertor(&reply);
  int rc = 0;
  convertor >> rc;

  if (rc == 1)
  {
    robot_started_ = true;
    return 0;
  } 
  else 
  {
    ROS_FATAL_STREAM("Could not start robot");
    return -1;
  }
}

void MitsubishiArmInterface::stopRobot()
{
  // Just send the password return the result
  MXTCMD end_cmd;
  memset(&end_cmd, 0, sizeof(end_cmd));
  end_cmd.BitMask = 0xffff; // Not sure if this is needed
  end_cmd.Command = MXT_CMD_END;

  mxt_socket_.send(boost::asio::buffer((char *)&end_cmd, sizeof(end_cmd)));
  robot_started_ = false;
}

void MitsubishiArmInterface::readHW()
{
  MXTCMD joint_cmd;
  MXTCMD joint_cmd_reply;
  memset(&joint_cmd, 0, sizeof(joint_cmd));
  joint_cmd.RecvType= MXT_TYP_JOINT;
  joint_cmd.BitMask = 0xffff; // Not sure if this is needed

  mxt_socket_.send(boost::asio::buffer((char *)&joint_cmd, sizeof(joint_cmd)));
  mxt_socket_.receive(boost::asio::buffer((char *)&joint_cmd_reply,sizeof(joint_cmd_reply)));

  //boost::mutex::scoped_lock lock(io_mutex);
  JOINT& jnt = joint_cmd_reply.dat.jnt;
  pos_[0] = jnt.j1;
  pos_[1] = jnt.j2;
  pos_[2] = jnt.j3;
  pos_[3] = jnt.j4;
  pos_[4] = jnt.j5;
  pos_[5] = jnt.j6;

  // convert to radians and add to state
  //for(int i=0; i< pos_.size(); ++i)
  //{
  //  pos_[i]=pos_[i]*(DEG_TO_RAD);
  //}


  // Not sure if this is necessary
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

  ROS_INFO_STREAM_THROTTLE(0.1, "cmd_: " 
      << cmd_[0] << " "
      << cmd_[1] << " "
      << cmd_[2] << " "
      << cmd_[3] << " "
      << cmd_[4] << " "
      << cmd_[5]);
  //std::cout << "cmd_: " 
  //    << cmd_[0] << " "
  //    << cmd_[1] << " "
  //    << cmd_[2] << " "
  //    << cmd_[3] << " "
  //    << cmd_[4] << " "
  //    << cmd_[5] << std::endl;

    //if(isEqual(cmd_previous_[0],cmd_[0],0.00001)&&
    //    isEqual(cmd_previous_[1],cmd_[1],0.00001)&&
    //    isEqual(cmd_previous_[2],cmd_[2],0.00001)&&
    //    isEqual(cmd_previous_[3],cmd_[3],0.00001)&&
    //    isEqual(cmd_previous_[4],cmd_[4],0.00001)&&
    //    isEqual(cmd_previous_[5],cmd_[5],0.00001))
    //{
  
    //  cmd_previous_=cmd_;
    //  return;
    //}

    pos_ = cmd_;
}


