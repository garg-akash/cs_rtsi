#pragma once

#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/robot_state.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <boost/thread/thread.hpp>

class RTSIReceiveInterface
{
 public:
	RTSIReceiveInterface(std::string hostip, double frequency = -1.0,
                         std::vector<std::string> variables = {},
                         bool verbose = false);

	// virtual ~RTSIReceiveInterface();

	bool setupRecipes(const double& frequency);

    void receiveCallback();

    std::vector<double> getActualJointPositions();

 private:
 	std::string hostip_;
 	double frequency_;
 	std::vector<std::string> variables_;
 	int port_;
 	bool verbose_;
 	double delta_time_;
 	std::shared_ptr<RTSI> rtsi_;
    std::atomic<bool> stop_receive_thread{false};
    std::shared_ptr<boost::thread> th_;
 	std::shared_ptr<RobotState> robot_state_;
    size_t no_bytes_avail_cnt_;
};