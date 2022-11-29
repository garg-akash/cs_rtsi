/*
Author: Elite_akashgarg
CreateDate: 2022-11-28
LastEdited: 2022-11-28
Description: example script to test Plugin Managaer
*/
#include <string>
#include <iostream>
#include <cs_rtsi/rtsi_io_interface_api.h>
#include <cs_rtsi/rtsi_receive_interface_api.h>
#include <PluginManager.h>

const std::string hostip = "192.168.133.129";
const std::string lib_path = "/home/ak/ur_client/cs_rtsi/build/pluginScript/";
const std::string plugin_name = "liblibRTSI";
const bool verbose = true;
double frequency = 100;

int main(int argc, char const *argv[])
{
  std::cout << "Program started...\n";
  PluginManager obj;
  obj.setPluginDirectory(lib_path);
  obj.setPluginName(plugin_name);
  auto handle = obj.loadPlugin();
  if(!handle)
  {
    std::cerr << "Handle not found\n";
    return 1;
  }
  void* (*createRTSIIOInstance)(std::string, bool, bool);
  createRTSIIOInstance = (void* (*)(std::string, bool, bool))dlsym(handle, "createRTSIIOInstance");
  RTSIIOInterfaceAPI* instance_io =  static_cast<RTSIIOInterfaceAPI *>((*createRTSIIOInstance)(hostip, verbose, false));
  instance_io->setStandardDigitalOut(2,true);

  void* (*createRTSIReceiveInstance)(std::string, double, std::vector<std::string>, bool);
  createRTSIReceiveInstance = (void* (*)(std::string, double, std::vector<std::string>, bool))dlsym(handle, "createRTSIReceiveInstance");
  // std::cout << "Sym : " << createRTSIReceiveInstance << "\n";
  RTSIReceiveInterfaceAPI* instance_receive =  static_cast<RTSIReceiveInterfaceAPI *>((*createRTSIReceiveInstance)(hostip, frequency, {}, verbose));
  std::vector<double> jp = instance_receive->getActualJointPositions();
  std::cout << "Joint positions : \n";
  for(auto j : jp)
    std::cout << j << "\n";
  return 0;
}