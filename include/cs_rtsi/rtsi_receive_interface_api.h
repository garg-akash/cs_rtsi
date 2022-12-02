#ifndef RTSIReceiveInterface_API_H_Include
#define RTSIReceiveInterface_API_H_Include

#include <iostream>
#include <string>
#include <vector>

// #ifdef WIN32 // declared when compiling with windows
//   #ifdef API_EXPORT // must be declared in Preprocessor Definitions
//       #define API __declspec(dllexport)
//   #else
//       #define API __declspec(dllimport)
//   #endif
// #else
//     #define API
// #endif

// #ifdef __cplusplus
// extern "C" {
// #endif
class RTSIReceiveInterfaceAPI
{
 public:
  virtual std::string getUniqueName() = 0;

  virtual void disconnect() = 0;

	virtual bool setupOutputRecipes(const double& frequency) = 0;

  virtual void receiveCallback() = 0;

  virtual double getPayloadMass() = 0;

  virtual std::vector<double> getPayloadCog() = 0;

  virtual std::uint32_t getScriptControlLine() = 0;

  virtual double getTimestamp() = 0;

  virtual std::vector<double> getTargetJointPositions() = 0;

  virtual std::vector<double> getTargetJointSpeeds() = 0;

  virtual std::vector<double> getActualJointTorques() = 0;
  
  virtual std::vector<double> getActualJointPositions() = 0;

  virtual std::vector<double> getActualJointSpeeds() = 0;

  virtual std::vector<double> getActualJointCurrent() = 0;
  
  virtual std::vector<double> getActualTCPPose() = 0;

  virtual std::vector<double> getActualTCPSpeed() = 0;

  virtual std::vector<double> getTargetTCPPose() = 0;

  virtual std::vector<double> getTargetTCPSpeed() = 0;

  virtual std::uint32_t getActualDigitalInputBits() = 0;

  virtual std::vector<double> getJointTemperatures() = 0;

  virtual std::int32_t getRobotMode() = 0;

  virtual std::vector<std::int32_t> getJointMode() = 0;

  virtual std::int32_t getSafetyMode() = 0;

  virtual std::int32_t getSafetyStatus() = 0;

  virtual double getSpeedScaling() = 0;

  virtual double getTargetSpeedFraction() = 0;

  virtual double getActualRobotVoltage() = 0;

  virtual double getActualRobotCurrent() = 0;

  virtual std::vector<double> getActualJointVoltage() = 0;

  virtual std::uint32_t getActualDigitalOutputBits() = 0;

  virtual std::uint32_t getRuntimeState() = 0;

  virtual std::vector<double> getElbowPosition() = 0;

  virtual std::uint32_t getRobotStatusBits() = 0;

  virtual std::uint32_t getSafetyStatusBits() = 0;

  virtual std::uint32_t getAnalogIOTypes() = 0;

  virtual double getStandardAnalogInput0() = 0;

  virtual double getStandardAnalogInput1() = 0;

  virtual double getStandardAnalogOutput0() = 0;

  virtual double getStandardAnalogOutput1() = 0;

  virtual double getIOCurrent() = 0;

  virtual std::uint32_t getToolMode() = 0;

  virtual std::uint32_t getToolAnalogInputTypes() = 0;

  virtual std::uint32_t getToolAnalogOutputTypes() = 0;

  virtual double getToolAnalogInput() = 0;

  virtual double getToolAnalogOutput() = 0;

  virtual double getToolOutputVoltage() = 0;

  virtual double getToolOutputCurrent() = 0;

  virtual double getToolTemperature() = 0;

  virtual std::uint32_t getOutputBitRegisters0to31() = 0;

  virtual std::uint32_t getOutputBitRegisters32to63() = 0;

  virtual std::uint8_t getToolDigitalMode() = 0;

  virtual std::uint8_t getToolDigital0Mode() = 0;

  virtual std::uint8_t getToolDigital1Mode() = 0;

  virtual std::uint8_t getToolDigital2Mode() = 0;

  virtual std::uint8_t getToolDigital3Mode() = 0;

  virtual std::uint32_t getInputBitRegisters0to31() = 0;

  virtual std::uint32_t getInputBitRegisters32to63() = 0;

  virtual bool getOutputBitRegister(int output_id) = 0;

  virtual std::int32_t getOutputIntRegister(int output_id) = 0;

  virtual double getOutputDoubleRegister(int output_id) = 0;

  virtual bool getInputBitRegister(int input_id) = 0;

  virtual std::int32_t getInputIntRegister(int input_id) = 0;

  virtual double getInputDoubleRegister(int input_id) = 0;
};
// #ifdef __cplusplus 
// }
// #endif
#endif