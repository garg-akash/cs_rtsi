/*
Author: Elite_akashgarg
CreateDate: 2022-11-09
LastEdited: 2022-11-22
Description: handles rtsi input subscription
*/
#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/rtsi_io_interface.h>
#include <cs_rtsi/rtsi_utility.h>

#include <string>
#include <chrono>
#include <thread>

RTSIIOInterface::RTSIIOInterface(std::string hostip, bool verbose, bool use_upper_range_registers)
	: hostip_(hostip), verbose_(verbose), use_upper_range_registers_(use_upper_range_registers)
{
	if(verbose_)
	{
		std::cout << "RTSIIOInterface: initiated\n";
	}

	port_ = 30004;
	rtsi_ = std::make_shared<RTSI>(hostip_, port_, verbose_);
  rtsi_->connect();
  rtsi_->negotiateProtocolVersion();
  register_offset_ = 0; //can be utilized for later functionality

  setupInputRecipes();
  rtsi_->sendStart(); //TODO
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

RTSIIOInterface::~RTSIIOInterface()
{
  if(rtsi_ != nullptr)
    if(rtsi_->isConnected())
      rtsi_->disconnect();
}

std::string RTSIIOInterface::getPluginName()
{
  return "ELITE ROBOT RTSI_IO_INTERFACE";
}


void RTSIIOInterface::disconnect()
{
  if(rtsi_ != nullptr)
    if(rtsi_->isConnected())
      rtsi_->disconnect();
}

bool RTSIIOInterface::setupInputRecipes()
{
  // Recipe 1
  std::vector<std::string> set_speed_slider = {"speed_slider_mask", "speed_slider_fraction"};
  rtsi_->sendInputSetup(set_speed_slider);

  // Recipe 2
  std::vector<std::string> set_std_digital_output = {"standard_digital_output_mask",
                                                     "standard_digital_output"};
  rtsi_->sendInputSetup(set_std_digital_output);

  // Recipe 3
  std::vector<std::string> set_conf_digital_output = {"configurable_digital_output_mask",
                                                      "configurable_digital_output"};
  rtsi_->sendInputSetup(set_conf_digital_output);

  // Recipe 4
  std::vector<std::string> set_std_analog_output = {"standard_analog_output_mask", "standard_analog_output_type", 
                                                    "standard_analog_output_0", "standard_analog_output_1"};
  rtsi_->sendInputSetup(set_std_analog_output);

  // Recipe 5-52
  for(int i = 0; i <= 47; i++)
  {
    std::vector<std::string> set_input_int_reg_input = {inIntReg(i)};
    rtsi_->sendInputSetup(set_input_int_reg_input);
  }

  // Recipe 53-100
  for(int i = 0; i <= 47; i++)
  {
    std::vector<std::string> set_input_double_reg_input = {inDoubleReg(i)};
    rtsi_->sendInputSetup(set_input_double_reg_input);
  }

  // Recipe 101
  std::vector<std::string> set_input_bit_reg_inputO_to_31 = {"input_bit_registers0_to_31"};
  rtsi_->sendInputSetup(set_input_bit_reg_inputO_to_31);

  // Recipe 102
  std::vector<std::string> set_input_bit_reg_input32_to_63 = {"input_bit_registers32_to_63"};
  rtsi_->sendInputSetup(set_input_bit_reg_input32_to_63);

  // Recipe 103-166
  for(int i = 64; i <= 127; i++)
  {
    std::vector<std::string> set_input_bit_reg_input = {inBitReg(i)};
    rtsi_->sendInputSetup(set_input_bit_reg_input);
  }
  
  return true;
}

bool RTSIIOInterface::setSpeedSlider(double speed)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_SPEED_SLIDER;
  robot_cmd.recipe_id_ = 1;
  robot_cmd.speed_slider_mask_ = 1;
  robot_cmd.speed_slider_fraction_ = speed;
  return sendCommand(robot_cmd);
}

bool RTSIIOInterface::setStandardDigitalOut(std::uint16_t output_id, bool signal_level)
{
  // std::cout << "Original : " << (int)output_id << "\n";
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_STD_DIGITAL_OUT;
  robot_cmd.recipe_id_ = 2;

  if (signal_level)
  {
    robot_cmd.std_digital_out_mask_ = static_cast<uint16_t>(1u << output_id);
    robot_cmd.std_digital_out_ = static_cast<uint16_t>(1u << output_id);
  }
  else
  {
    robot_cmd.std_digital_out_mask_ = static_cast<uint16_t>(1u << output_id);
    robot_cmd.std_digital_out_ = 0;
  }
  return sendCommand(robot_cmd);
}

bool RTSIIOInterface::setConfigurableDigitalOut(std::uint8_t output_id, bool signal_level)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_CONF_DIGITAL_OUT;
  robot_cmd.recipe_id_ = 3;

  if (signal_level)
  {
    robot_cmd.configurable_digital_out_mask_ = static_cast<uint8_t>(1u << output_id);
    robot_cmd.configurable_digital_out_ = static_cast<uint8_t>(1u << output_id);
  }
  else
  {
    robot_cmd.configurable_digital_out_mask_ = static_cast<uint8_t>(1u << output_id);
    robot_cmd.configurable_digital_out_ = 0;
  }
  return sendCommand(robot_cmd);
}

bool RTSIIOInterface::setAnalogOutputVoltage(std::uint8_t output_id, double signal_ratio)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_STD_ANALOG_OUT;
  robot_cmd.recipe_id_ = 4;
  robot_cmd.std_analog_output_mask_ = static_cast<uint8_t>(1u << output_id);
  robot_cmd.std_analog_output_type_ = 1;  // set output type (0:Current; 1:voltage)
  if (output_id == 0)
    robot_cmd.std_analog_output_0_ = signal_ratio;
  else if (output_id == 1)
    robot_cmd.std_analog_output_1_ = signal_ratio;
  return sendCommand(robot_cmd);
}

bool RTSIIOInterface::setAnalogOutputCurrent(std::uint8_t output_id, double signal_ratio)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_STD_ANALOG_OUT;
  robot_cmd.recipe_id_ = 4;
  robot_cmd.std_analog_output_mask_ = static_cast<uint8_t>(1u << output_id);
  robot_cmd.std_analog_output_type_ = 0;  // set output type (0:Current; 1:voltage)
  if (output_id == 0)
    robot_cmd.std_analog_output_0_ = signal_ratio;
  else if (output_id == 1)
    robot_cmd.std_analog_output_1_ = signal_ratio;
  return sendCommand(robot_cmd);
}

bool RTSIIOInterface::setInputIntRegister(int input_id, int value)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_INPUT_INT_REGISTER;
  
  if (input_id >= 0 && input_id <= 47)
    robot_cmd.recipe_id_ = 5 + input_id;
  else
    throw std::range_error(
        "The supported range of setInputIntRegister() is [0-47], you specified: " +
        std::to_string(input_id));

  robot_cmd.reg_int_val_ = value;
  return sendCommand(robot_cmd);
}

bool RTSIIOInterface::setInputDoubleRegister(int input_id, double value)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_INPUT_DOUBLE_REGISTER;
  
  if (input_id >= 0 && input_id <= 47)
    robot_cmd.recipe_id_ = 53 + input_id;
  else
    throw std::range_error(
        "The supported range of setInputDoubleRegister() is [0-47], you specified: " +
        std::to_string(input_id));

  robot_cmd.reg_double_val_ = value;
  return sendCommand(robot_cmd);
}

bool RTSIIOInterface::setInputBitRegisters0to31(std::uint32_t value)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_INPUT_BIT_REGISTER_X_TO_Y;
  robot_cmd.recipe_id_ = 101;
  robot_cmd.reg_bit_val_x_to_y_ = value;
  return sendCommand(robot_cmd);
}

bool RTSIIOInterface::setInputBitRegisters32to63(std::uint32_t value)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_INPUT_BIT_REGISTER_X_TO_Y;
  robot_cmd.recipe_id_ = 102;
  robot_cmd.reg_bit_val_x_to_y_ = value;
  return sendCommand(robot_cmd);
}

bool RTSIIOInterface::setInputBitRegister(int input_id, bool value)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_INPUT_BIT_REGISTER;
  
  if (input_id >= 64 && input_id <= 127)
    robot_cmd.recipe_id_ = 103 + input_id - 64;
  else
    throw std::range_error(
        "The supported range of setInputBitRegister() is [64-127], you specified: " +
        std::to_string(input_id));

  robot_cmd.reg_bit_val_ = value;
  return sendCommand(robot_cmd);
}

std::string RTSIIOInterface::inIntReg(int reg)
{
  return "input_int_register" + std::to_string(register_offset_ + reg);
}

std::string RTSIIOInterface::inDoubleReg(int reg)
{
  return "input_double_register" + std::to_string(register_offset_ + reg);
}

std::string RTSIIOInterface::inBitReg(int reg)
{
  return "input_bit_register" + std::to_string(reg);
}

bool RTSIIOInterface::sendCommand(const RTSI::RobotCommand &cmd)
{
  try
  {
    rtsi_->send(cmd);
    return true;
  }
  catch (std::exception &e)
  {
    std::cout << "RTSIIOInterface: Lost connection to robot..." << std::endl;
    std::cerr << e.what() << std::endl;
    if (rtsi_ != nullptr)
    {
      if (rtsi_->isConnected())
        rtsi_->disconnect();
    }
  }

  if (!rtsi_->isConnected())
  {
    std::cout << "RTSIIOInterface: Robot is disconnected..." << std::endl;
    // reconnect();
    // return sendCommand(cmd);
  }
  return false;
}

void* createRTSIIOInstance(std::string hostip, bool verbose, bool use_upper_range_registers)
{
  static RTSIIOInterfaceAPI* rtsi_io = nullptr;
  if(rtsi_io == nullptr)
  {
    rtsi_io = new RTSIIOInterface(hostip, verbose, use_upper_range_registers);
  }
  // rtsi_io->setStandardDigitalOut(0,true);
  return rtsi_io;
}