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
  // if(rtsi_ != nullptr)
  //   if(rtsi_->isConnected())
  //     rtsi_->disconnect();
}

void RTSIIOInterface::disconnect()
{
  if(rtsi_ != nullptr)
    if(rtsi_->isConnected())
      rtsi_->disconnect();
}

bool RTSIIOInterface::setupInputRecipes()
{
  // std::vector<std::string> ak_input = {"standard_digital_output_mask", "standard_digital_output",
  //                                      "configurable_digital_output_mask", "configurable_digital_output"};
  // rtsi_->sendInputSetup(ak_input);

  // // Recipe 1
  // std::vector<std::string> no_cmd_input = {inIntReg(23)};
  // rtsi_->sendInputSetup(no_cmd_input);

  // // Recipe 2
  // std::vector<std::string> set_speed_slider = {inIntReg(23), "speed_slider_mask", "speed_slider_fraction"};
  // rtsi_->sendInputSetup(set_speed_slider);

  // Recipe 3
  // std::vector<std::string> set_std_digital_output = {"standard_digital_output_mask",
  //                                                    "standard_digital_output"};
  // rtsi_->sendInputSetup(set_std_digital_output);

  // Recipe 4
  std::vector<std::string> set_conf_digital_output = {"configurable_digital_output_mask",
                                                      "configurable_digital_output"};
  rtsi_->sendInputSetup(set_conf_digital_output);


  // // Recipe 5
  // std::vector<std::string> set_std_analog_output = {inIntReg(23), "standard_analog_output_mask",
  //                                                   "standard_analog_output_type", "standard_analog_output_0",
  //                                                   "standard_analog_output_1"};
  // rtsi_->sendInputSetup(set_std_analog_output);

  // // Recipe 6
  // std::vector<std::string> set_input_int_reg_0_input = {inIntReg(23), inIntReg(18)};
  // rtsi_->sendInputSetup(set_input_int_reg_0_input);

  // // Recipe 7
  // std::vector<std::string> set_input_int_reg_1_input = {inIntReg(23), inIntReg(19)};
  // rtsi_->sendInputSetup(set_input_int_reg_1_input);

  // // Recipe 8
  // std::vector<std::string> set_input_int_reg_2_input = {inIntReg(23), inIntReg(20)};
  // rtsi_->sendInputSetup(set_input_int_reg_2_input);

  // // Recipe 9
  // std::vector<std::string> set_input_int_reg_3_input = {inIntReg(23), inIntReg(21)};
  // rtsi_->sendInputSetup(set_input_int_reg_3_input);

  // // Recipe 10
  // std::vector<std::string> set_input_int_reg_4_input = {inIntReg(23), inIntReg(22)};
  // rtsi_->sendInputSetup(set_input_int_reg_4_input);

  // Recipe 11
  // std::vector<std::string> set_input_double_reg_0_input = {inIntReg(23), inDoubleReg(0)};
  // rtsi_->sendInputSetup(set_input_double_reg_0_input);

  // // Recipe 12
  // std::vector<std::string> set_input_double_reg_1_input = {inIntReg(23), inDoubleReg(19)};
  // rtsi_->sendInputSetup(set_input_double_reg_1_input);

  // // Recipe 13
  // std::vector<std::string> set_input_double_reg_2_input = {inIntReg(23), inDoubleReg(20)};
  // rtsi_->sendInputSetup(set_input_double_reg_2_input);

  // // Recipe 14
  // std::vector<std::string> set_input_double_reg_3_input = {inIntReg(23), inDoubleReg(21)};
  // rtsi_->sendInputSetup(set_input_double_reg_3_input);

  // // Recipe 15
  // std::vector<std::string> set_input_double_reg_4_input = {inIntReg(23), inDoubleReg(22)};
  // rtsi_->sendInputSetup(set_input_double_reg_4_input);
  
  return true;
}

bool RTSIIOInterface::setAKData(std::uint16_t std_digital_id, bool std_digital_level,
                                  std::uint8_t conf_digital_id, bool conf_digital_level)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_AK_DATA;
  robot_cmd.recipe_id_ = 1;
  if (std_digital_level)
  {
    robot_cmd.std_digital_out_mask_ = static_cast<uint16_t>(1u << std_digital_id);
    robot_cmd.std_digital_out_ = static_cast<uint16_t>(1u << std_digital_id);
  }
  else
  {
    robot_cmd.std_digital_out_mask_ = static_cast<uint16_t>(1u << std_digital_id);
    robot_cmd.std_digital_out_ = 0;
  }
  if (conf_digital_level)
  {
    robot_cmd.configurable_digital_out_mask_ = static_cast<uint8_t>(1u << conf_digital_id);
    robot_cmd.configurable_digital_out_ = static_cast<uint8_t>(1u << conf_digital_id);
  }
  else
  {
    robot_cmd.configurable_digital_out_mask_ = static_cast<uint8_t>(1u << conf_digital_id);
    robot_cmd.configurable_digital_out_ = 0;
  }

  return sendCommand(robot_cmd);
}

bool RTSIIOInterface::setSpeedSlider(double speed)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_SPEED_SLIDER;
  robot_cmd.recipe_id_ = 2;
  robot_cmd.speed_slider_mask_ = 1;
  robot_cmd.speed_slider_fraction_ = speed;
  return sendCommand(robot_cmd);
}

bool RTSIIOInterface::setStandardDigitalOut(std::uint16_t output_id, bool signal_level)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_STD_DIGITAL_OUT;
  robot_cmd.recipe_id_ = 1;

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
  robot_cmd.recipe_id_ = 2;

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
  robot_cmd.recipe_id_ = 5;
  robot_cmd.std_analog_output_mask_ = static_cast<uint8_t>(1u << output_id);
  robot_cmd.std_analog_output_type_ = 1;  // set output type (0:Current; 1:voltage)
  if (output_id == 0)
    robot_cmd.std_analog_output_0_ = signal_ratio;
  else if (output_id == 1)
    robot_cmd.std_analog_output_1_ = signal_ratio;
  // robot_cmd.std_analog_output_ = signal_ratio;
  return sendCommand(robot_cmd);
}

bool RTSIIOInterface::setInputIntRegister(int input_id, int value)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_INPUT_INT_REGISTER;
  if (use_upper_range_registers_)
  {
    if (input_id == 42)
      robot_cmd.recipe_id_ = 6;
    else if (input_id == 43)
      robot_cmd.recipe_id_ = 7;
    else if (input_id == 44)
      robot_cmd.recipe_id_ = 8;
    else if (input_id == 45)
      robot_cmd.recipe_id_ = 9;
    else if (input_id == 46)
      robot_cmd.recipe_id_ = 10;
    else
      throw std::range_error(
          "The supported range of setInputIntRegister() is [42-46], when using upper range, you specified: " +
          std::to_string(input_id));

    robot_cmd.reg_int_val_ = value;
    return sendCommand(robot_cmd);
  }
  else
  {
    if (input_id == 18)
      robot_cmd.recipe_id_ = 6;
    else if (input_id == 19)
      robot_cmd.recipe_id_ = 7;
    else if (input_id == 20)
      robot_cmd.recipe_id_ = 8;
    else if (input_id == 21)
      robot_cmd.recipe_id_ = 9;
    else if (input_id == 22)
      robot_cmd.recipe_id_ = 10;
    else
      throw std::range_error(
          "The supported range of setInputIntRegister() is [18-22], when using lower range, you specified: " +
          std::to_string(input_id));

    robot_cmd.reg_int_val_ = value;
    return sendCommand(robot_cmd);
  }
}

bool RTSIIOInterface::setInputDoubleRegister(int input_id, double value)
{
  RTSI::RobotCommand robot_cmd;
  robot_cmd.type_ = RTSI::RobotCommand::Type::SET_INPUT_DOUBLE_REGISTER;
  if (use_upper_range_registers_)
  {
    if (input_id == 42)
      robot_cmd.recipe_id_ = 11;
    else if (input_id == 43)
      robot_cmd.recipe_id_ = 12;
    else if (input_id == 44)
      robot_cmd.recipe_id_ = 13;
    else if (input_id == 45)
      robot_cmd.recipe_id_ = 14;
    else if (input_id == 46)
      robot_cmd.recipe_id_ = 15;
    else
      throw std::range_error(
          "The supported range of setInputDoubleRegister() is [42-46], when using upper range, you specified: " +
          std::to_string(input_id));

    robot_cmd.reg_double_val_ = value;
    return sendCommand(robot_cmd);
  }
  else
  {
    if (input_id == 18)
      robot_cmd.recipe_id_ = 0;
    else if (input_id == 19)
      robot_cmd.recipe_id_ = 12;
    else if (input_id == 20)
      robot_cmd.recipe_id_ = 13;
    else if (input_id == 21)
      robot_cmd.recipe_id_ = 14;
    else if (input_id == 22)
      robot_cmd.recipe_id_ = 15;
    else
      throw std::range_error(
          "The supported range of setInputDoubleRegister() is [18-22], when using lower range, you specified: " +
          std::to_string(input_id));

    robot_cmd.reg_double_val_ = value;
    return sendCommand(robot_cmd);
  }
}

std::string RTSIIOInterface::inIntReg(int reg)
{
  return "input_int_register" + std::to_string(register_offset_+reg);
}

std::string RTSIIOInterface::inDoubleReg(int reg)
{
  return "input_double_register" + std::to_string(register_offset_+reg);
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