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
#include <bitset>
#include <chrono>

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
  state_names_ = {"robot_status_bits", "safety_status_bits", "runtime_state"};
  for (int i = 0; i <= 2; i++)
    state_names_.emplace_back(outIntReg(i));
  for (int i = 0; i <= 5; i++)
    state_names_.emplace_back(outDoubleReg(i));  

  rtsi_->sendOutputSetup(state_names_, frequency);

  // Recipe 1
  std::vector<std::string> servoj_input = {inIntReg(0), inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
                                           inDoubleReg(3), inDoubleReg(4), inDoubleReg(5), 
                                           inDoubleReg(6), inDoubleReg(7), inDoubleReg(8)};
  rtsi_->sendInputSetup(servoj_input);

  // Recipe 2
  std::vector<std::string> no_cmd_input = {inIntReg(0)};
  rtsi_->sendInputSetup(no_cmd_input);

  return true;
}

std::string RTSIControlInterface::inIntReg(int reg)
{
  return "input_int_register" + std::to_string(register_offset_ + reg);
};

std::string RTSIControlInterface::inDoubleReg(int reg)
{
  return "input_double_register" + std::to_string(register_offset_ + reg);
};

std::string RTSIControlInterface::outIntReg(int reg)
{
  return "output_int_register" + std::to_string(register_offset_ + reg);
};

std::string RTSIControlInterface::outDoubleReg(int reg)
{
  return "output_double_register" + std::to_string(register_offset_ + reg);
};

int RTSIControlInterface::getOutputIntReg(int reg)
{
  std::string output_int_register_key = "output_int_register" + std::to_string(register_offset_ + reg);
  int32_t output_int_register_val;
  if (robot_state_->getStateData(output_int_register_key, output_int_register_val))
    return output_int_register_val;
  else
    throw std::runtime_error("unable to get state data for specified key: " + output_int_register_key);
};

int RTSIControlInterface::getControlScriptState()
{
  if (robot_state_ != nullptr)
  {
    return getOutputIntReg(0);
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

bool RTSIControlInterface::isProgramRunning()
{
  uint32_t runtime_state;
  if (!robot_state_->getStateData("runtime_state", runtime_state))
    throw std::runtime_error("unable to get state data for specified key: runtime_state");

  if (runtime_state == RuntimeState::PLAYING)
    return true;
  else
    return false;
}

bool RTSIControlInterface::isProtectiveStopped()
{
  if(robot_state_ != nullptr)
  {
    uint32_t safety_status_bits;
    if(robot_state_->getStateData("safety_status_bits", safety_status_bits))
    {
      std::bitset<32> safety_status_bitset(safety_status_bits);
      return safety_status_bitset.test(SafetyStatus::IS_PROTECTIVE_STOPPED);
    }
    else
      throw std::runtime_error("unable to get state data for specified key: safety_status_bits");
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

bool RTSIControlInterface::isEmergencyStopped()
{
  if(robot_state_ != nullptr)
  {
    uint32_t safety_status_bits;
    if(robot_state_->getStateData("safety_status_bits", safety_status_bits))
    {
      std::bitset<32> safety_status_bitset(safety_status_bits);
      return safety_status_bitset.test(SafetyStatus::IS_EMERGENCY_STOPPED);
    }
    else
      throw std::runtime_error("unable to get state data for specified key: safety_status_bits");
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

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
void RTSIControlInterface::sendClearCommand()
{
  RTSI::RobotCommand clear_cmd;
  clear_cmd.type_ = RTSI::RobotCommand::Type::NO_CMD;
  clear_cmd.recipe_id_ = RTSI::RobotCommand::Recipe::RECIPE_2;
  rtsi_->send(clear_cmd);
}

bool RTSIControlInterface::sendCommand(const RTSI::RobotCommand &cmd)
{
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  uint32_t runtime_state;
  if (!robot_state_->getStateData("runtime_state", runtime_state))
      throw std::runtime_error("unable to get state data for specified key: runtime_state");
  
  // TODO : use runtime_state to see if robot is running/stopped
  
  if(cmd.type_ == RTSI::RobotCommand::Type::SERVOJ)
  {
    rtsi_->send(cmd);
    return true;
  }
  else
  {
    rtsi_->send(cmd);

    //TODO : use STOP_SCRIPT test case
    start_time = std::chrono::steady_clock::now();
    while(getControlScriptState() != CS_CONTROLLER_DONE_WITH_CMD)
    {
      // if script causes an error, then it may be possible that the script
      // no longer runs and we don't receive the DONE_WITH_CMD signal
      if (!isProgramRunning())
      {
        std::cerr << "RTSIControlInterface: CS control script is not running!" << std::endl;
        sendClearCommand();
        return false;
      }

      if (isProtectiveStopped() || isEmergencyStopped())
      {
        sendClearCommand();
        return false;
      }

      // Waiting to give controller time to finish or timeout
      auto current_time = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
      if (duration > CS_EXECUTION_TIMEOUT)
      {
        sendClearCommand();
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  // Make controller ready for next command
  sendClearCommand();
  return true;
}

void RTSIControlInterface::waitFunction(const std::chrono::steady_clock::time_point& t_start)
{
  RTSIUtility::waitFunction(t_start, delta_time_);
}