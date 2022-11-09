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

   std::string inBitReg(int reg);

   bool sendCommand(const RTSI::RobotCommand &cmd);

   bool setSpeedSlider(double fraction);

   bool setStandardDigitalOut(std::uint16_t output_id, bool signal_level);

   bool setConfigurableDigitalOut(std::uint8_t output_id, bool signal_level);

   bool setAnalogOutputVoltage(std::uint8_t output_id, double signal_ratio);

   bool setAnalogOutputCurrent(std::uint8_t output_id, double signal_ratio);
   
   bool setInputIntRegister(int input_id, std::int32_t value);

   bool setInputDoubleRegister(int input_id, double value);

   bool setInputBitRegister0to31(std::uint32_t value);

   bool setInputBitRegister32to63(std::uint32_t value);

   bool setInputBitRegister(int input_id, bool value);

   bool setAKData(std::uint16_t output_id1, bool signal_level1, std::uint8_t output_id2, bool signal_level2);

   bool setInputDoubleRegisterPosition(double v1, double v2, double v3);
   
 private:
 	std::string hostip_;
 	int port_;
 	bool verbose_;
   bool use_upper_range_registers_;
   int register_offset_;
 	std::shared_ptr<RTSI> rtsi_;
};