#ifndef RTSIIOInterface_API_H_Include
#define RTSIIOInterface_API_H_Include

#include <iostream>
#include <string>

#ifdef WIN32 // declared when compiling with windows
  #ifdef API_EXPORT // must be declared in Preprocessor Definitions
      #define API __declspec(dllexport)
  #else
      #define API __declspec(dllimport)
  #endif
#else
    #define API
#endif

#ifdef __cplusplus
extern "C" {
#endif
  API class RTSIIOInterfaceAPI
  {
  public:
    // virtual void setHostIP(std::string hostip) = 0;

    // virtual void setVerbose(bool verbose) = 0;

    // virtual void setUseUpperRangeRegisters(bool use_upper_range_registers) = 0;

    virtual std::string getUniqueName() = 0;

    virtual bool setSpeedSlider(double fraction) = 0;

    virtual bool setStandardDigitalOut(std::uint16_t output_id, bool signal_level) = 0;

    virtual bool setConfigurableDigitalOut(std::uint8_t output_id, bool signal_level) = 0;

    virtual bool setAnalogOutputVoltage(std::uint8_t output_id, double signal_ratio) = 0;

    virtual bool setAnalogOutputCurrent(std::uint8_t output_id, double signal_ratio) = 0;
     
    virtual bool setInputIntRegister(int input_id, std::int32_t value) = 0;

    virtual bool setInputDoubleRegister(int input_id, double value) = 0;

    virtual bool setInputBitRegisters0to31(std::uint32_t value) = 0;

    virtual bool setInputBitRegisters32to63(std::uint32_t value) = 0;

    virtual bool setInputBitRegister(int input_id, bool value) = 0;

  };

#ifdef __cplusplus 
}
#endif
#endif