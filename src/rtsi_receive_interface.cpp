/*
Author: Elite_akashgarg
CreateDate: 2022-11-09
Description: handles rtsi output subscription
*/
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

std::uint32_t RTSIReceiveInterface::getActualDigitalInputBits()
{
  std::uint32_t actual_digital_input_bits;
  if (robot_state_->getStateData("actual_digital_input_bits", actual_digital_input_bits))
    return actual_digital_input_bits;
  else
    throw std::runtime_error("unable to get state data for actual_digital_input_bits");
}

std::vector<double> RTSIReceiveInterface::getJointTemperatures()
{
	std::vector<double> joint_temperatures;
	if (robot_state_->getStateData("joint_temperatures", joint_temperatures))
	    return joint_temperatures;
	else
	    throw std::runtime_error("unable to get state data for joint_temperatures");
}

std::int32_t RTSIReceiveInterface::getRobotMode()
{
  std::int32_t robot_mode;
  if (robot_state_->getStateData("robot_mode", robot_mode))
    return robot_mode;
  else
    throw std::runtime_error("unable to get state data for robot_mode");
}

std::vector<std::int32_t> RTSIReceiveInterface::getJointMode()
{
	std::vector<std::int32_t> joint_mode;
	if (robot_state_->getStateData("joint_mode", joint_mode))
	    return joint_mode;
	else
	    throw std::runtime_error("unable to get state data for joint_mode");
}

std::int32_t RTSIReceiveInterface::getSafetyMode()
{
  std::int32_t safety_mode;
  if (robot_state_->getStateData("safety_mode", safety_mode))
    return safety_mode;
  else
    throw std::runtime_error("unable to get state data for safety_mode");
}

std::int32_t RTSIReceiveInterface::getSafetyStatus()
{
  std::int32_t safety_status;
  if (robot_state_->getStateData("safety_status", safety_status))
    return safety_status;
  else
    throw std::runtime_error("unable to get state data for safety_status");
}

double RTSIReceiveInterface::getSpeedScaling()
{
	double speed_scaling;
	if(robot_state_->getStateData("speed_scaling", speed_scaling))
		return speed_scaling;
	else
		throw std::runtime_error("unable to get state data for speed_scaling");
}

double RTSIReceiveInterface::getTargeSpeedFraction()
{
	double target_speed_fraction;
	if(robot_state_->getStateData("target_speed_fraction", target_speed_fraction))
		return target_speed_fraction;
	else
		throw std::runtime_error("unable to get state data for target_speed_fraction");
}

double RTSIReceiveInterface::getActualRobotVoltage()
{
	double actual_robot_voltage;
	if(robot_state_->getStateData("actual_robot_voltage", actual_robot_voltage))
		return actual_robot_voltage;
	else
		throw std::runtime_error("unable to get state data for actual_robot_voltage");
}

double RTSIReceiveInterface::getActualRobotCurrent()
{
	double actual_robot_current;
	if(robot_state_->getStateData("actual_robot_current", actual_robot_current))
		return actual_robot_current;
	else
		throw std::runtime_error("unable to get state data for actual_robot_current");
}

std::uint32_t RTSIReceiveInterface::getActualDigitalOutputBits()
{
  std::uint32_t actual_digital_output_bits;
  if (robot_state_->getStateData("actual_digital_output_bits", actual_digital_output_bits))
    return actual_digital_output_bits;
  else
    throw std::runtime_error("unable to get state data for actual_digital_output_bits");
}

std::uint32_t RTSIReceiveInterface::getRuntimeState()
{
  std::uint32_t runtime_state;
  if (robot_state_->getStateData("runtime_state", runtime_state))
    return runtime_state;
  else
    throw std::runtime_error("unable to get state data for runtime_state");
}

std::vector<double> RTSIReceiveInterface::getElbowPosition()
{
	std::vector<double> elbow_position;
	if (robot_state_->getStateData("elbow_position", elbow_position))
	    return elbow_position;
	else
	    throw std::runtime_error("unable to get state data for elbow_position");
}

std::uint32_t RTSIReceiveInterface::getRobotStatusBits()
{
  std::uint32_t robot_status_bits;
  if (robot_state_->getStateData("robot_status_bits", robot_status_bits))
    return robot_status_bits;
  else
    throw std::runtime_error("unable to get state data for robot_status_bits");
}

std::uint32_t RTSIReceiveInterface::getSafetyStatusBits()
{
  std::uint32_t safety_status_bits;
  if (robot_state_->getStateData("safety_status_bits", safety_status_bits))
    return safety_status_bits;
  else
    throw std::runtime_error("unable to get state data for safety_status_bits");
}

std::uint32_t RTSIReceiveInterface::getAnalogIOTypes()
{
  std::uint32_t analog_io_types;
  if (robot_state_->getStateData("analog_io_types", analog_io_types))
    return analog_io_types;
  else
    throw std::runtime_error("unable to get state data for analog_io_types");
}