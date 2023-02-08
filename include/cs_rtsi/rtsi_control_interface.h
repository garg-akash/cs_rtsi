#pragma once

#include <cs_rtsi/rtsi.h>
#include <string>

class RTSIControlInterface
{
private:
  std::string hostip_;
  int port_;
  double frequency_;
  bool verbose_;
  std::shared_ptr<RTSI> rtsi_;
  std::vector<std::string> state_names_;
  int register_offset_;
  std::shared_ptr<RobotState> robot_state_;

public:
  RTSIControlInterface(std::string hostip, double frequency = 250, bool verbose = true);
  
  virtual ~RTSIControlInterface();

  bool sendCommand(const RTSI::RobotCommand &cmd);

  bool setupRecipes(const double &frequency);
  
  bool servoJ(const std::vector<double> &q, double time = 0.1, double lookahead_time = 0.08, double gain = 300);

  std::string inDoubleReg(int reg);
};