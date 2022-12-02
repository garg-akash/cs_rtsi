/*
Author: Elite_akashgarg
CreateDate: 2022-11-28
LastEdited: 2022-12-02
Description: example script to test Plugin Managaer
*/
#include <string>
#include <iostream>
#include <cs_rtsi/rtsi_io_interface_api.h>
#include <cs_rtsi/rtsi_receive_interface_api.h>
#include <PluginManager.h>

const std::string hostip = "192.168.133.129";
const std::string lib_path = "/home/ak/ur_client/cs_rtsi/build/src/";
const std::string plugin_name = "liblibRTSI";
const bool verbose = true;
double frequency = 100;

int main(int argc, char const *argv[])
{
  std::cout << "Program started...\n";

  PluginManager obj;
  obj.setHostIP(hostip);
  obj.setVerbose(verbose);
  obj.setFrequency(frequency);
  obj.setPluginDirectory(lib_path);
  obj.setPluginName(plugin_name);
  auto handle = obj.loadPlugin();

  if (!handle)
  {
    std::cerr << "Handle to the requested library not found\n";
    throw std::exception();
  }

  if (!obj.setIoInterfaceName("ELITE ROBOT RTSI_IO_INTERFACE"))
  {
    std::cerr << "The requested IO Interface is not found!\n";
    throw std::exception();
  }

  obj.getCurrentRTSIIOAPI()->setStandardDigitalOut(2,true);
  
  if (!obj.setReceiveInterfaceName("ELITE ROBOT RTSI_RECEIVE_INTERFACE"))
  {
    std::cerr << "The requested Receive Interface is not found!\n";
    throw std::exception();
  }

  std::vector<double> jp = obj.getCurrentRTSIReceiveAPI()->getActualJointPositions();
  std::cout << "Joint positions : \n";
  for(auto j : jp)
    std::cout << j << "\n";

  return 0;
}