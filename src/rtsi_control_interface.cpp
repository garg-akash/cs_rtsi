/*
Author: Elite_akashgarg
CreateDate: 2023-02-08
Description: rtsi control interface implementation
*/
#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/rtsi_control_interface.h>
#include <cs_rtsi/rtsi_utility.h>
#include <cs_rtsi/robot_state.h>

#include <string>
#include <thread>

RTSIControlInterface::RTSIControlInterface(std::string hostip, double frequency, bool verbose)
  : hostip_(std::move(hostip)),
    frequency_(frequency),
    verbose_(verbose)
{
  register_offset_ = 0;
  port_ = 30004;
  rtsi_ = RTSI::getRTSIInstance(hostip_, port_, verbose_);
  if(frequency_ < 0)
    frequency_ = 250;
  delta_time_ = 1 / frequency_;
  setupRecipes(frequency_);
  robot_state_ = std::make_shared<RobotState>(state_names_);
  rtsi_->sendStart();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

RTSIControlInterface::~RTSIControlInterface()
{
  if(rtsi_ != nullptr)
    if(rtsi_->isConnected())
      rtsi_->disconnect();
}

bool RTSIControlInterface::setupRecipes(const double &frequency)
{
  state_names_ = {"runtime_state"};
  rtsi_->sendOutputSetup(state_names_, frequency);

  // Recipe 1
  std::vector<std::string> servoj_input = {inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
                                           inDoubleReg(3), inDoubleReg(4), inDoubleReg(5), 
                                           inDoubleReg(6), inDoubleReg(7), inDoubleReg(8)};
  rtsi_->sendInputSetup(servoj_input);

  return true;
}

std::string RTSIControlInterface::inDoubleReg(int reg)
{
  return "input_double_register" + std::to_string(register_offset_ + reg);
};

bool RTSIControlInterface::servoJ(const std::vector<double> &q, double time,
                                  double lookahead_time, double gain)
{
  // TODO : verify limits

  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SERVOJ;
  robot_cmd.recipe_id_ = RTSI::RobotCommand::Recipe::RECIPE_1;
  robot_cmd.val_ = q;
  robot_cmd.val_.push_back(time);
  robot_cmd.val_.push_back(lookahead_time);
  robot_cmd.val_.push_back(gain);
  return sendCommand(robot_cmd);
}

bool RTSIControlInterface::sendCommand(const RTSI::RobotCommand &cmd)
{
  uint32_t runtime_state;
  if (!robot_state_->getStateData("runtime_state", runtime_state))
      throw std::runtime_error("unable to get state data for specified key: runtime_state");
  
  // TODO : use runtime_state to see if robot is running/stopped
  
  if(cmd.type_ == RTSI::RobotCommand::Type::SERVOJ)
  {
    rtsi_->send(cmd);
    return true;
  }
  return true;
}

void RTSIControlInterface::waitFunction(const std::chrono::steady_clock::time_point& t_start)
{
  RTSIUtility::waitFunction(t_start, delta_time_);
}