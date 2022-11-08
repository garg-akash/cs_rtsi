#include <cs_rtsi/rtsi_io_interface.h>
#include <iostream>
#include <thread>

const std::string hostip = "192.168.133.129";

int main(int argc, char const *argv[])
{
  bool verbose = true;
  RTSIIOInterface rtsi_io(hostip, verbose);

  // rtsi_io.setStandardDigitalOut(11, true);
  rtsi_io.setConfigurableDigitalOut(6, true);

  std::cout << "IO set\n";

  return 0;
}