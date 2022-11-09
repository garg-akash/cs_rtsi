/*
Author: Elite_akashgarg
CreateDate: 2022-11-09
Description: example script to test input subscription
*/
#include <cs_rtsi/rtsi_io_interface.h>
#include <iostream>
#include <thread>

const std::string hostip = "192.168.133.129";

int main(int argc, char const *argv[])
{
  bool verbose = true;
  RTSIIOInterface rtsi_io(hostip, verbose);

  // rtsi_io.setSpeedSlider(1);
  // rtsi_io.setInputDoubleRegister(18,0.3);
  rtsi_io.setStandardDigitalOut(15, true);
  rtsi_io.setConfigurableDigitalOut(0, true);
  rtsi_io.setAnalogOutputVoltage(0,0.3);
  rtsi_io.setAnalogOutputCurrent(1,0.7);
  rtsi_io.setInputIntRegister(47,19);
  rtsi_io.setInputDoubleRegister(47,0.19);
  rtsi_io.setInputBitRegister0to31(1);
  rtsi_io.setInputBitRegister32to63(3);
  rtsi_io.setInputBitRegister(88,true);
  // for(int i = 0; i  < 50; i++)
  // {
  //   rtsi_io.setInputDoubleRegisterPosition(0.671, -0.460 + 0.001*i, 0.542);
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // }
  // rtsi_io.setInputDoubleRegisterPosition(0, 0, 0);
  std::cout << "IO set\n";

  return 0;
}