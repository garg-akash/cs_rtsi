#include <cs_rtsi/rtsi_io_interface.h>
#include <iostream>
#include <thread>

const std::string hostip = "192.168.51.139";

int main(int argc, char const *argv[])
{
  bool verbose = true;
  RTSIIOInterface rtsi_io(hostip, verbose);

  // rtsi_io.setStandardDigitalOut(9, true);
  // rtsi_io.setSpeedSlider(1);
  // rtsi_io.setInputDoubleRegister(18,0.3);
  // rtsi_io.setStandardDigitalOut(5, true);
  // rtsi_io.setConfigurableDigitalOut(5, true);
  // rtsi_io.setAnalogOutputVoltage(0,0.7);
  // rtsi_io.setAKData(11,true,6,true);
  for(int i = 0; i  < 50; i++)
  {
    rtsi_io.setInputDoubleRegisterPosition(0.671, -0.460 + 0.001*i, 0.542);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  rtsi_io.setInputDoubleRegisterPosition(0, 0, 0);
  std::cout << "IO set\n";

  return 0;
}