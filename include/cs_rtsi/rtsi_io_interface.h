#pragma once

#include <cs_rtsi/rtsi_io_interface_api.h>
#include <cs_rtsi/rtsi.h>
#include <string>

class RTSIIOInterface : public RTSIIOInterfaceAPI
{
 public:
  RTSIIOInterface(std::string hostip, bool verbose = false, bool use_upper_range_registers = false);

	virtual ~RTSIIOInterface();

  std::string getUniqueName();

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

  bool setInputBitRegisters0to31(std::uint32_t value);

  bool setInputBitRegisters32to63(std::uint32_t value);

  bool setInputBitRegister(int input_id, bool value);

  bool setExternalForceTorque(const std::vector<double> &value);
 
 private:
 	std::string hostip_;
 	int port_;
 	bool verbose_;
  bool use_upper_range_registers_;
  int register_offset_;
 	std::shared_ptr<RTSI> rtsi_;
};

#ifdef __cplusplus
extern "C" {
#endif

void* createRTSIIOInstance(std::string hostip, bool verbose, bool use_upper_range_registers);

#ifdef __cplusplus 
}
#endif