#include <cs_rtsi/robot_state.h>
#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/rtsi_receive_interface.h>

#include <boost/thread/thread.hpp>
#include <chrono>
#include <iostream>
#include <thread>

RTSIReceiveInterface::RTSIReceiveInterface(std::string hotsip, double frequency, std::vector<std::string> variables,
										   bool verbose)
	: hostip_(std::move(hotsip)),
	  frequency_(frequency),
	  variables_(std::move(variables)),
	  verbose_(verbose)
{
	if(verbose_)
	{
		std::cout << "RTSIReceiveInterface: initiated\n";
	}

	port_ = 30004;
	rtsi_ = std::make_shared<RTSI>(hostip_, port_, verbose_);
	rtsi_->connect();
	rtsi_->negotiateProtocolVersion();
	delta_time_ = 1 / frequency_;

	setupOutputRecipes(frequency_);

	robot_state_ = std::make_shared<RobotState>(variables_);

  rtsi_->sendStart();

  th_ = std::make_shared<boost::thread>(boost::bind(&RTSIReceiveInterface::receiveCallback, this));

	while(!robot_state_->getFirstStateReceived())
	{
		std::cout << "Waiting to get state\n";
		std::this_thread::sleep_for(std::chrono::microseconds(100));
	}
}

RTSIReceiveInterface::~RTSIReceiveInterface()
{
	disconnect();
}

void RTSIReceiveInterface::disconnect()
{
	stop_receive_thread = true;
	th_->interrupt();
	th_->join();

	if(rtsi_ != nullptr)
	{
		if(rtsi_->isConnected())
			rtsi_->disconnect();
	}
	
	//waiting to disconnect fully
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

bool RTSIReceiveInterface::setupOutputRecipes(const double& frequency)
{
	if(variables_.empty())
	{
		variables_ = {"timestamp"};
	}
	// Setup output
  	rtsi_->sendOutputSetup(variables_, frequency);
  	return true;
}

void RTSIReceiveInterface::receiveCallback()
{
	while (!stop_receive_thread)
	{
  	// Receive and update the robot state
  	try
  	{
  		if (rtsi_->isDataAvailable())
  		{
    		no_bytes_avail_cnt_ = 0;
    		boost::system::error_code ec = rtsi_->receiveData(robot_state_);
    		if(ec)
    		{
      			if(ec == boost::asio::error::eof)
      			{
        			std::cerr << "RTSIReceiveInterface: Robot closed the connection!" << std::endl;
      			}
      			throw std::system_error(ec);
    		}
  		}
  		else
  		{
    		// Register that data was not available in this cycle
    		no_bytes_avail_cnt_++;
    		// If at least 2ms has passed without data available, try to read data again to detect de-synchronization.
    		if (no_bytes_avail_cnt_ > 20)
    		{
      			boost::system::error_code ec = rtsi_->receiveData(robot_state_);
     			if(ec)
    			{
      			if(ec == boost::asio::error::eof)
      			{
        				std::cerr << "RTSIReceiveInterface: Robot closed the connection!" << std::endl;
      			}
      			throw std::system_error(ec);
    			}
      			no_bytes_avail_cnt_ = 0;
    		}

// #if defined(__linux__) || defined(__APPLE__)
//         // Data not available on socket yet, yield to other threads and sleep before trying to receive data again.
        std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::microseconds(100));
// #endif
  		}
		}
		catch (std::exception& e)
		{
		  std::cerr << "RTSIReceiveInterface Exception: " << e.what() << std::endl;
		  if (rtsi_->isConnected())
		    rtsi_->disconnect();
		  stop_receive_thread = true;
		  // stop_record_thread = true;
		}
	}
}

double RTSIReceiveInterface::getPayloadMass()
{
  double payload_mass;
  if (robot_state_->getStateData("payload_mass", payload_mass))
    return payload_mass;
  else
    throw std::runtime_error("unable to get state data for payload_mass");
}

std::vector<double> RTSIReceiveInterface::getPayloadCog()
{
  std::vector<double> payload_cog;
  if (robot_state_->getStateData("payload_cog", payload_cog))
    return payload_cog;
  else
    throw std::runtime_error("unable to get state data for payload_cog");
}

uint32_t RTSIReceiveInterface::getScriptControlLine()
{
  uint32_t script_control_line;
  if (robot_state_->getStateData("script_control_line", script_control_line))
    return script_control_line;
  else
    throw std::runtime_error("unable to get state data for script_control_line");
}

double RTSIReceiveInterface::getTimestamp()
{
	double timestamp;
	if(robot_state_->getStateData("timestamp", timestamp))
		return timestamp;
	else
		throw std::runtime_error("unable to get state data for timestamp");
}

std::vector<double> RTSIReceiveInterface::getTargetJointPositions()
{
	std::vector<double> target_joint_positions;
	if (robot_state_->getStateData("target_joint_positions", target_joint_positions))
		return target_joint_positions;
	else
		throw std::runtime_error("unable to get state data for target_joint_positions");
}

std::vector<double> RTSIReceiveInterface::getTargetJointSpeeds()
{
	std::vector<double> target_joint_speeds;
	if (robot_state_->getStateData("target_joint_speeds", target_joint_speeds))
		return target_joint_speeds;
	else
		throw std::runtime_error("unable to get state data for target_joint_speeds");
}

std::vector<double> RTSIReceiveInterface::getTargetJointTorques()
{
	std::vector<double> target_joint_torques;
	if (robot_state_->getStateData("target_joint_torques", target_joint_torques))
		return target_joint_torques;
	else
		throw std::runtime_error("unable to get state data for target_joint_torques");
}

std::vector<double> RTSIReceiveInterface::getActualJointPositions()
{
	std::vector<double> actual_joint_positions;
	if (robot_state_->getStateData("actual_joint_positions", actual_joint_positions))
		return actual_joint_positions;
	else
		throw std::runtime_error("unable to get state data for actual_joint_positions");
}

std::vector<double> RTSIReceiveInterface::getActualJointSpeeds()
{
	std::vector<double> actual_joint_speeds;
	if (robot_state_->getStateData("actual_joint_speeds", actual_joint_speeds))
		return actual_joint_speeds;
	else
		throw std::runtime_error("unable to get state data for actual_joint_speeds");
}

std::vector<double> RTSIReceiveInterface::getActualJointCurrent()
{
	std::vector<double> actual_joint_current;
	if (robot_state_->getStateData("actual_joint_current", actual_joint_current))
		return actual_joint_current;
	else
		throw std::runtime_error("unable to get state data for actual_joint_current");
}

std::vector<double> RTSIReceiveInterface::getActualTCPPose()
{
	std::vector<double> actual_tcp_pose;
	if (robot_state_->getStateData("actual_TCP_pose", actual_tcp_pose))
	    return actual_tcp_pose;
	else
	    throw std::runtime_error("unable to get state data for actual_TCP_pose");
}

std::vector<double> RTSIReceiveInterface::getActualTCPSpeed()
{
	std::vector<double> actual_tcp_speed;
	if (robot_state_->getStateData("actual_TCP_speed", actual_tcp_speed))
	    return actual_tcp_speed;
	else
	    throw std::runtime_error("unable to get state data for actual_TCP_speed");
}

std::vector<double> RTSIReceiveInterface::getTargetTCPPose()
{
	std::vector<double> target_tcp_pose;
	if (robot_state_->getStateData("target_TCP_pose", target_tcp_pose))
	    return target_tcp_pose;
	else
	    throw std::runtime_error("unable to get state data for target_TCP_pose");
}

std::vector<double> RTSIReceiveInterface::getTargetTCPSpeed()
{
	std::vector<double> target_tcp_speed;
	if (robot_state_->getStateData("target_TCP_speed", target_tcp_speed))
	    return target_tcp_speed;
	else
	    throw std::runtime_error("unable to get state data for target_TCP_speed");
}