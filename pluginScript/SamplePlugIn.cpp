/*
Author: Elite_akashgarg
CreateDate: 2022-11-28
LastEdited: 2022-11-28
Description: example script to test RTSI_IO_INTERFACE plugin
*/
#include <dlfcn.h>
#include <dirent.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cs_rtsi/rtsi_io_interface_api.h>
#include <cs_rtsi/rtsi_receive_interface_api.h>

const std::string hostip = "192.168.133.129";
const std::string lib_path = "/home/ak/ur_client/cs_rtsi/build/pluginScript/";
const bool verbose = true;
double frequency = 100;

std::string extension(char* f)
{
  std::string file_name(f);
  int position = file_name.find_last_of(".");
  return file_name.substr(position+1);
}

class PlugInObject
{
  void* handle;
  char* error;
  public:
  void LoadSharedLib(const char* path)
  {
    DIR *dir = opendir(path);
    struct dirent *ent;
    if (dir != NULL)
    {
      std::cout << "Dir opened\n";
      while ((ent = readdir(dir)) != NULL)
      {
        auto ext = extension(ent->d_name);
        if (ext.compare("so") != 0)
          continue;

        std::string fname(ent->d_name);
        fname = path + fname;
        std::cout << "File Name : " << fname << "\n"; 
        handle = dlopen(fname.c_str(), RTLD_LAZY);
        // std::cout << "Handle : " << handle << "\n";
        if (!handle || ((error = dlerror()) != NULL))
        {
          std::cerr << error << "\n";
          continue;
        }

        dlerror();

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

        if ((error = dlerror()) != NULL)
        {
          std::cerr << error << "\n";
          continue;
        }
        dlclose(handle);
      }
      closedir(dir);
    }
  }
};

int main(int argc, char const *argv[])
{
  std::cout << "Program started...\n";
  PlugInObject obj;
  obj.LoadSharedLib(lib_path.c_str ());
  return 0;
}