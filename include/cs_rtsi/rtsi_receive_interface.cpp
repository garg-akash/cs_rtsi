#include <cs_rtsi/robot_state.h>
#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/rtsi_receive_interface.h>

#include <boost/thread/thread.hpp>
#include <chrono>
#include <iostream>
#include <thread>

RTSIReceiveInterface::RTSIReceiveInterface(std::string hotsip, double frequency, std::vector<std::string> variables,
										   bool verbose)
	: hotsip_(std::move(hotsip)),
	  frequency_(frequency),
	  variables_(std::move(variables)),
	  verbose_(verbose)
{
	if(verbose_)
	{
		std::cout << "RTSIReceiveInterface: initiated\n"
	}

	port_ = 30004;
	rtsi_ = std::make_shared<RTSI>(hostip_, port_, verbose_);
	rtsi_->connect();
	rtsi_->negotiateProtocolVersion();
	frequency_ = 500;
	delta_time_ = 1 / frequency_;

	setupRecipes(frequency_);

	robot_state_ = std::make_shared<RobotState>(variables_);

  	rtsi_->sendStart();

  	th_ = std::make_shared<boost::thread>(boost::bind(&RTSIReceiveInterface::receiveCallback, this));

	while(!robot_state_->getFirstStateReceived())
	{
		std::this_thread::sleep_for(std::chrono::microseconds(100));
	}
}

bool RTSIReceiveInterface::setupRecipes(const double& frequency)
{
	if(variables_.empty())
	{
		variables_ = {"timestamp",
					  "actual_joint_positions",
					  "actual_TCP_pose"};
	}
	// Setup output
  	rtsi_->sendOutputSetup(variables_, frequency);
  	return true;
}
