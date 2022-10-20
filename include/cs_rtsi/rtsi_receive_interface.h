#pragma once

#include <cs_rtsi/robot_state.h>

class RTSIRecieveInterface
{
 public:
	RTSIReceiveInterface(std::string hostip, double frequency = -1.0,
                         std::vector<std::string> variables = {},
                         bool verbose = false);

	virtual ~RTSIRecieveInterface();

	bool setupRecipes(const double& frequency);

 private:
 	std::string hostip_;
 	double frequency_;
 	std::vector<std::string> variables_;
 	int port_;
 	bool verbose_;
 	double delta_time_;
 	std::shared_ptr<RTSI> rtsi_;
 	std::shared_ptr<RobotState> robot_state_;
};