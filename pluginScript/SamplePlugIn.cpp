#include <dlfcn.h>
#include <dirent.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cs_rtsi/rtsi_io_interface_api.h>

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
        std::cout << "Handle : " << handle << "\n";
        if (!handle || ((error = dlerror()) != NULL))
        {
          std::cerr << error << "\n";
          continue;
        }

        dlerror();
        // std::cout << "Sym : " << dlsym(handle, "createRTSIIOInstance") << "\n";

        void* (*createRTSIIOInstance)(std::string, bool, bool);
        createRTSIIOInstance = (void* (*)(std::string, bool, bool))dlsym(handle, "createRTSIIOInstance");
        RTSIIOInterfaceAPI* instance =  static_cast<RTSIIOInterfaceAPI *>(createRTSIIOInstance("192.168.133.129", true, false));
        // instance = 
        // if(!instance)
        //   std::cout << "Instance not created\n";
        // RTSIIOInterface* ins;
        // RTSIIOInterfaceAPI* ins = static_cast<RTSIIOInterfaceAPI *>(instance);
        instance->setStandardDigitalOut(2,true);

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
  obj.LoadSharedLib("/home/ak/ur_client/cs_rtsi/build/src/");
  return 0;
}