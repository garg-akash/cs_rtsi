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
  // rtsi_io.setStandardDigitalOut(3, true);
  // rtsi_io.setConfigurableDigitalOut(7, true);
  rtsi_io.setStandardDigitalOut(10, true);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));

  std::cout << "Digital pin set\n";

  return 0;
}