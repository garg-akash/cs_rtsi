#pragma once

#include <cs_rtsi/rtsi.h>
#include <string>

class RTSIIOInterface
{
 public:
	RTSIIOInterface(std::string hostip, bool verbose = false, bool use_upper_range_registers = false);

	virtual ~RTSIIOInterface();

   void disconnect();

	bool setupInputRecipes();

   std::string inIntReg(int reg);
   
   std::string inDoubleReg(int reg);

   bool sendCommand(const RTSI::RobotCommand &cmd);

   bool setSpeedSlider(double fraction);

   bool setStandardDigitalOut(std::uint16_t output_id, bool signal_level);

   bool setConfigurableDigitalOut(std::uint8_t output_id, bool signal_level);

   bool setAnalogOutput(std::uint8_t output_type, std::uint8_t output_id, double signal_ratio);

   // bool setInputBitRegister(int input_id, std::int32_t value); //TODO
   
   bool setInputIntRegister(int input_id, std::int32_t value);

   bool setInputDoubleRegister(int input_id, double value);

 private:
 	std::string hostip_;
 	int port_;
 	bool verbose_;
   bool use_upper_range_registers_;
   int register_offset_;
 	std::shared_ptr<RTSI> rtsi_;
};