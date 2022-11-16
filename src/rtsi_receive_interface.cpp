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
	std::vector<double> actual_joint_torques;
	if (robot_state_->getStateData("actual_joint_torques", actual_joint_torques))
		return actual_joint_torques;
	else
		throw std::runtime_error("unable to get state data for actual_joint_torques");
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

std::vector<double> RTSIReceiveInterface::getActualJointVoltage()
{
	std::vector<double> actual_joint_voltage;
	if(robot_state_->getStateData("actual_joint_voltage", actual_joint_voltage))
		return actual_joint_voltage;
	else
		throw std::runtime_error("unable to get state data for actual_joint_voltage");
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

double RTSIReceiveInterface::getStandardAnalogInput0()
{
	double standard_analog_input0;
	if(robot_state_->getStateData("standard_analog_input0", standard_analog_input0))
		return standard_analog_input0;
	else
		throw std::runtime_error("unable to get state data for standard_analog_input0");
}

double RTSIReceiveInterface::getStandardAnalogInput1()
{
	double standard_analog_input1;
	if(robot_state_->getStateData("standard_analog_input1", standard_analog_input1))
		return standard_analog_input1;
	else
		throw std::runtime_error("unable to get state data for standard_analog_input1");
}

double RTSIReceiveInterface::getStandardAnalogOutput0()
{
	double standard_analog_output0;
	if(robot_state_->getStateData("standard_analog_output0", standard_analog_output0))
		return standard_analog_output0;
	else
		throw std::runtime_error("unable to get state data for standard_analog_output0");
}

double RTSIReceiveInterface::getStandardAnalogOutput1()
{
	double standard_analog_output1;
	if(robot_state_->getStateData("standard_analog_output1", standard_analog_output1))
		return standard_analog_output1;
	else
		throw std::runtime_error("unable to get state data for standard_analog_output1");
}

double RTSIReceiveInterface::getIOCurrent()
{
	double io_current;
	if(robot_state_->getStateData("io_current", io_current))
		return io_current;
	else
		throw std::runtime_error("unable to get state data for io_current");
}

std::uint32_t RTSIReceiveInterface::getToolMode()
{
  std::uint32_t tool_mode;
  if (robot_state_->getStateData("tool_mode", tool_mode))
    return tool_mode;
  else
    throw std::runtime_error("unable to get state data for tool_mode");
}

std::uint32_t RTSIReceiveInterface::getToolAnalogInputTypes()
{
  std::uint32_t tool_analog_input_types;
  if (robot_state_->getStateData("tool_analog_input_types", tool_analog_input_types))
    return tool_analog_input_types;
  else
    throw std::runtime_error("unable to get state data for tool_analog_input_types");
}

std::uint32_t RTSIReceiveInterface::getToolAnalogOutputTypes()
{
  std::uint32_t tool_analog_output_types;
  if (robot_state_->getStateData("tool_analog_output_types", tool_analog_output_types))
    return tool_analog_output_types;
  else
    throw std::runtime_error("unable to get state data for tool_analog_output_types");
}

double RTSIReceiveInterface::getToolAnalogInput()
{
	double tool_analog_input;
	if(robot_state_->getStateData("tool_analog_input", tool_analog_input))
		return tool_analog_input;
	else
		throw std::runtime_error("unable to get state data for tool_analog_input");
}

double RTSIReceiveInterface::getToolAnalogOutput()
{
	double tool_analog_output;
	if(robot_state_->getStateData("tool_analog_output", tool_analog_output))
		return tool_analog_output;
	else
		throw std::runtime_error("unable to get state data for tool_analog_output");
}

double RTSIReceiveInterface::getToolOutputVoltage()
{
	double tool_output_voltage;
	if(robot_state_->getStateData("tool_output_voltage", tool_output_voltage))
		return tool_output_voltage;
	else
		throw std::runtime_error("unable to get state data for tool_output_voltage");
}

double RTSIReceiveInterface::getToolOutputCurrent()
{
	double tool_output_current;
	if(robot_state_->getStateData("tool_output_current", tool_output_current))
		return tool_output_current;
	else
		throw std::runtime_error("unable to get state data for tool_output_current");
}

double RTSIReceiveInterface::getToolTemperature()
{
	double tool_temperature;
	if(robot_state_->getStateData("tool_temperature", tool_temperature))
		return tool_temperature;
	else
		throw std::runtime_error("unable to get state data for tool_temperature");
}

std::uint32_t RTSIReceiveInterface::getOutputBitRegisters0to31()
{
  std::uint32_t output_bit_registers0_to_31;
  if (robot_state_->getStateData("output_bit_registers0_to_31", output_bit_registers0_to_31))
    return output_bit_registers0_to_31;
  else
    throw std::runtime_error("unable to get state data for output_bit_registers0_to_31");
}

std::uint32_t RTSIReceiveInterface::getOutputBitRegisters32to63()
{
  std::uint32_t output_bit_registers32_to_63;
  if (robot_state_->getStateData("output_bit_registers32_to_63", output_bit_registers32_to_63))
    return output_bit_registers32_to_63;
  else
    throw std::runtime_error("unable to get state data for output_bit_registers32_to_63");
}

std::uint8_t RTSIReceiveInterface::getToolDigitalMode()
{
  std::uint8_t tool_digital_mode;
  if (robot_state_->getStateData("tool_digital_mode", tool_digital_mode))
    return tool_digital_mode;
  else
    throw std::runtime_error("unable to get state data for tool_digital_mode");
}

std::uint8_t RTSIReceiveInterface::getToolDigital0Mode()
{
  std::uint8_t tool_digital0_mode;
  if (robot_state_->getStateData("tool_digital0_mode", tool_digital0_mode))
    return tool_digital0_mode;
  else
    throw std::runtime_error("unable to get state data for tool_digital0_mode");
}

std::uint8_t RTSIReceiveInterface::getToolDigital1Mode()
{
  std::uint8_t tool_digital1_mode;
  if (robot_state_->getStateData("tool_digital1_mode", tool_digital1_mode))
    return tool_digital1_mode;
  else
    throw std::runtime_error("unable to get state data for tool_digital1_mode");
}

std::uint8_t RTSIReceiveInterface::getToolDigital2Mode()
{
  std::uint8_t tool_digital2_mode;
  if (robot_state_->getStateData("tool_digital2_mode", tool_digital2_mode))
    return tool_digital2_mode;
  else
    throw std::runtime_error("unable to get state data for tool_digital2_mode");
}

std::uint8_t RTSIReceiveInterface::getToolDigital3Mode()
{
  std::uint8_t tool_digital3_mode;
  if (robot_state_->getStateData("tool_digital3_mode", tool_digital3_mode))
    return tool_digital3_mode;
  else
    throw std::runtime_error("unable to get state data for tool_digital3_mode");
}

std::uint32_t RTSIReceiveInterface::getInputBitRegisters0to31()
{
  std::uint32_t input_bit_registers0_to_31;
  if (robot_state_->getStateData("input_bit_registers0_to_31", input_bit_registers0_to_31))
    return input_bit_registers0_to_31;
  else
    throw std::runtime_error("unable to get state data for input_bit_registers0_to_31");
}

std::uint32_t RTSIReceiveInterface::getInputBitRegisters32to63()
{
  std::uint32_t input_bit_registers32_to_63;
  if (robot_state_->getStateData("input_bit_registers32_to_63", input_bit_registers32_to_63))
    return input_bit_registers32_to_63;
  else
    throw std::runtime_error("unable to get state data for input_bit_registers32_to_63");
}

bool RTSIReceiveInterface::getOutputBitRegister(int output_id)
{
  if (!isWithinBounds(output_id, 0, 127))
  {
    throw std::range_error("The supported range of getOutputBitRegister() is [0-127], you specified: "
    											 + std::to_string(output_id));
  }
  
  std::string output_bit_register_key = "output_bit_register" + std::to_string(output_id);
  bool output_bit_register_val;
  if (robot_state_->getStateData(output_bit_register_key, output_bit_register_val))
    return output_bit_register_val;
  else
    throw std::runtime_error("unable to get state data for "+output_bit_register_key);
}

std::int32_t RTSIReceiveInterface::getOutputIntRegister(int output_id)
{
  if (!isWithinBounds(output_id, 0, 47))
  {
    throw std::range_error("The supported range of getOutputIntRegister() is [0-47], you specified: "
    											 + std::to_string(output_id));
  }
  
  std::string output_int_register_key = "output_int_register" + std::to_string(output_id);
  std::int32_t output_int_register_val;
  if (robot_state_->getStateData(output_int_register_key, output_int_register_val))
    return output_int_register_val;
  else
    throw std::runtime_error("unable to get state data for "+output_int_register_key);
}

double RTSIReceiveInterface::getOutputDoubleRegister(int output_id)
{
  if (!isWithinBounds(output_id, 0, 47))
  {
    throw std::range_error("The supported range of getOutputDoubleRegister() is [0-47], you specified: "
    											 + std::to_string(output_id));
  }
  
  std::string output_double_register_key = "output_double_register" + std::to_string(output_id);
  std::int32_t output_double_register_val;
  if (robot_state_->getStateData(output_double_register_key, output_double_register_val))
    return output_double_register_val;
  else
    throw std::runtime_error("unable to get state data for "+output_double_register_key);
}

bool RTSIReceiveInterface::getInputBitRegister(int input_id)
{
  if (!isWithinBounds(input_id, 0, 127))
  {
    throw std::range_error("The supported range of getInputBitRegister() is [0-127], you specified: " 
    												+ std::to_string(input_id));
  }
  
  std::string input_bit_register_key = "input_bit_register" + std::to_string(input_id);
  bool input_bit_register_val;
  if (robot_state_->getStateData(input_bit_register_key, input_bit_register_val))
    return input_bit_register_val;
  else
    throw std::runtime_error("unable to get state data for "+input_bit_register_key);
}

std::int32_t RTSIReceiveInterface::getInputIntRegister(int input_id)
{
  if (!isWithinBounds(input_id, 0, 47))
  {
    throw std::range_error("The supported range of getIutputIntRegister() is [0-47], you specified: "
    											 + std::to_string(input_id));
  }
  
  std::string input_int_register_key = "input_int_register" + std::to_string(input_id);
  std::int32_t input_int_register_val;
  if (robot_state_->getStateData(input_int_register_key, input_int_register_val))
    return input_int_register_val;
  else
    throw std::runtime_error("unable to get state data for "+input_int_register_key);
}

double RTSIReceiveInterface::getInputDoubleRegister(int input_id)
{
  if (!isWithinBounds(input_id, 0, 47))
  {
    throw std::range_error("The supported range of getInputDoubleRegister() is [0-47], you specified: "
    											 + std::to_string(input_id));
  }
  
  std::string input_double_register_key = "input_double_register" + std::to_string(input_id);
  std::int32_t input_double_register_val;
  if (robot_state_->getStateData(input_double_register_key, input_double_register_val))
    return input_double_register_val;
  else
    throw std::runtime_error("unable to get state data for "+input_double_register_key);
}