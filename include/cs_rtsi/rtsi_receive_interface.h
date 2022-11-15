#pragma once

#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/robot_state.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <boost/thread/thread.hpp>

class RTSIReceiveInterface
{
 public:
	RTSIReceiveInterface(std::string hostip, double frequency = -1.0,
                         std::vector<std::string> variables = {},
                         bool verbose = false);

	virtual ~RTSIReceiveInterface();

    void disconnect();

	bool setupOutputRecipes(const double& frequency);

    template <typename T>
    bool isWithinBounds(const T& value, const T& low, const T& high)
    {
      return (low <= value && value <= high);
    }

    void receiveCallback();

    double getPayloadMass();

    std::vector<double> getPayloadCog();

    std::uint32_t getScriptControlLine();

    double getTimestamp();

    std::vector<double> getTargetJointPositions();

    std::vector<double> getTargetJointSpeeds();

    std::vector<double> getTargetJointTorques();
    
    std::vector<double> getActualJointPositions();

    std::vector<double> getActualJointSpeeds();

    std::vector<double> getActualJointCurrent();
    
    std::vector<double> getActualTCPPose();

    std::vector<double> getActualTCPSpeed();

    std::vector<double> getTargetTCPPose();

    std::vector<double> getTargetTCPSpeed();

    std::uint32_t getActualDigitalInputBits();

    std::vector<double> getJointTemperatures();

    std::int32_t getRobotMode();

    std::vector<std::int32_t> getJointMode();

    std::int32_t getSafetyMode();

    std::int32_t getSafetyStatus();

    double getSpeedScaling();

    double getTargeSpeedFraction();

    double getActualRobotVoltage();

    double getActualRobotCurrent();

    std::vector<double> getActualJointVoltage();

    std::uint32_t getActualDigitalOutputBits();

    std::uint32_t getRuntimeState();

    std::vector<double> getElbowPosition();

    std::uint32_t getRobotStatusBits();

    std::uint32_t getSafetyStatusBits();

    std::uint32_t getAnalogIOTypes();

    double getStandardAnalogInput0();

    double getStandardAnalogInput1();

    double getStandardAnalogOutput0();

    double getStandardAnalogOutput1();

    double getIOCurrent();

    std::uint32_t getToolMode();

    std::uint32_t getToolAnalogInputTypes();

    std::uint32_t getToolAnalogOutputTypes();

    double getToolAnalogInput();

    double getToolAnalogOutput();

    double getToolOutputVoltage();

    double getToolOutputCurrent();

    double getToolTemperature();

    std::uint32_t getOutputBitRegisters0to31();

    std::uint32_t getOutputBitRegisters32to63();

    std::uint8_t getToolDigitalMode();

    std::uint8_t getToolDigital0Mode();

    std::uint8_t getToolDigital1Mode();

    std::uint8_t getToolDigital2Mode();

    std::uint8_t getToolDigital3Mode();

    std::uint32_t getInputBitRegisters0to31();

    std::uint32_t getInputBitRegisters32to63();

    bool getOutputBitRegister(int output_id);

    std::int32_t getOutputIntRegister(int output_id);

    double getOutputDoubleRegister(int output_id);

    bool getInputBitRegister(int input_id);

    std::int32_t getInputIntRegister(int input_id);

    double getInputDoubleRegister(int input_id);

    std::shared_ptr<RobotState> robot_state_;

 private:
 	std::string hostip_;
 	double frequency_;
 	std::vector<std::string> variables_;
 	int port_;
 	bool verbose_;
 	double delta_time_;
 	std::shared_ptr<RTSI> rtsi_;
    std::atomic<bool> stop_receive_thread{false};
    std::shared_ptr<boost::thread> th_;
    size_t no_bytes_avail_cnt_;
};