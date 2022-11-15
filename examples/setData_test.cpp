/*
Author: Elite_akashgarg
CreateDate: 2022-11-09
LastEdited: 2022-11-15
Description: example script to test input subscription
*/
#include <cs_rtsi/rtsi_io_interface.h>
#include <iostream>
#include <thread>
#include <jsoncpp/json/json.h>
#include <fstream>

const std::string hostip = "192.168.133.129";
const std::string recipe_file = "../examples/recipe.json";

int main(int argc, char const *argv[])
{
  bool verbose = true;
  RTSIIOInterface rtsi_io(hostip, verbose);
  std::ifstream recipes(recipe_file, std::ifstream::binary);
  Json::Value i_recipes;
  recipes >> i_recipes;
  std::vector<std::string> variables;
  for(auto i : i_recipes["input"])
  {
    // std::cout << i["subs"] << " " << i["number"] << " " << i["value"] << "\n";
    if(i["subs"].asString() == "speed_slider_fraction")
      rtsi_io.setSpeedSlider(i["value"].asDouble());

    else if(i["subs"].asString() == "standard_digital_output")
      rtsi_io.setStandardDigitalOut(i["number"].asUInt(), i["value"].asBool());

    else if(i["subs"].asString() == "configurable_digital_output")
      rtsi_io.setConfigurableDigitalOut(i["number"].asUInt(), i["value"].asBool());

    else if(i["subs"].asString() == "standard_analog_output_0")
    {
      if(i["type"].asBool() == 0)
        rtsi_io.setAnalogOutputCurrent(0, i["value"].asDouble());
      if(i["type"].asBool() == 1)
        rtsi_io.setAnalogOutputVoltage(0, i["value"].asDouble());
    }

    else if(i["subs"].asString() == "standard_analog_output_1")
    {
      if(i["type"].asBool() == 0)
        rtsi_io.setAnalogOutputCurrent(1, i["value"].asDouble());
      if(i["type"].asBool() == 1)
        rtsi_io.setAnalogOutputVoltage(1, i["value"].asDouble());
    }

    else if(i["subs"].asString() == "input_bit_registers0_to_31")
      rtsi_io.setInputBitRegister0to31(i["value"].asInt());

    else if(i["subs"].asString() == "input_bit_registers32_to_64")
      rtsi_io.setInputBitRegister32to63(i["value"].asInt());

    else if(i["subs"].asString() == "input_bit_registerX")
      rtsi_io.setInputBitRegister(i["number"].asInt(), i["value"].asBool());

    else if(i["subs"].asString() == "input_int_registerX")
      rtsi_io.setInputIntRegister(i["number"].asInt(), i["value"].asInt());

    else if(i["subs"].asString() == "input_double_registerX")
      rtsi_io.setInputDoubleRegister(i["number"].asInt(), i["value"].asDouble());

    else
      throw std::runtime_error("Invalid input subscription requested : " + i["subs"].asString());
  }

  std::cout << "IO set\n";

  return 0;
}