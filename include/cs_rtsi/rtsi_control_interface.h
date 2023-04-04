#pragma once

#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/rtsi_utility.h>
#include <string>
#include <chrono>

#define CS_CONTROLLER_READY_FOR_CMD 1
#define CS_CONTROLLER_DONE_WITH_CMD 2
#define CS_EXECUTION_TIMEOUT 300

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

  enum SafetyStatus
  {
    IS_NORMAL_MODE = 0,
    IS_REDUCED_MODE = 1,
    IS_PROTECTIVE_STOPPED = 2,
    IS_RECOVERY_MODE = 3,
    IS_SAFEGUARD_STOPPED = 4,
    IS_SYSTEM_EMERGENCY_STOPPED = 5,
    IS_ROBOT_EMERGENCY_STOPPED = 6,
    IS_EMERGENCY_STOPPED = 7
  };

  enum RuntimeState
  {
    UNKNOWN = 0,
    PLAYING = 1,
    PAUSED = 2,
    STOPPED = 3
  };

  bool sendCommand(const RTSI::RobotCommand &cmd);

  bool setupRecipes(const double &frequency);
  
  bool servoJ(const std::vector<double> &q, double time = 0.1, double lookahead_time = 0.08, double gain = 300);

  void sendClearCommand();

  std::string inIntReg(int reg);

  std::string inDoubleReg(int reg);

  std::string outIntReg(int reg);

  std::string outDoubleReg(int reg);

  int getOutputIntReg(int reg);

  int getControlScriptState();

  bool isProgramRunning();

  bool isProtectiveStopped();

  bool isEmergencyStopped();

  void waitFunction(const std::chrono::steady_clock::time_point& t_start);
};