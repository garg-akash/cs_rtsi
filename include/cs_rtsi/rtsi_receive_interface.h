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

	virtual ~RTSIReceiveInterface();

    void disconnect();

	bool setupOutputRecipes(const double& frequency);

    void receiveCallback();

    double getPayloadMass();

    std::vector<double> getPayloadCog();

    uint32_t getScriptControlLine();

    double getTimestamp();

    std::vector<double> getTargetJointPositions();

    std::vector<double> getTargetJointSpeeds();

    std::vector<double> getTargetJointTorques();
    
    std::vector<double> getActualJointPositions();

    std::vector<double> getActualJointSpeeds();

    std::vector<double> getActualJointCurrent();
    
    std::vector<double> getActualTCPPose();

    std::vector<double> getActualTCPSpeed();

    std::vector<double> getTargetTCPPose();

    std::vector<double> getTargetTCPSpeed();

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