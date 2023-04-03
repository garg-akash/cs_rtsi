#pragma once

#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/rtsi_utility.h>
#include <string>
#include <chrono>

class RTSIControlInterface
{
private:
  std::string hostip_;
  int port_;
  double frequency_;
  double delta_time_;
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

  std::string inIntReg(int reg);

  std::string inDoubleReg(int reg);

  void waitFunction(const std::chrono::steady_clock::time_point& t_start);
};