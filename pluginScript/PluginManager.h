/*
Author: Elite_akashgarg
CreateDate: 2022-11-29
LastEdited: 2022-12-02
Description: script to handle dynamic loading of libraries
*/
#ifndef plugInManager_h
#define plugInManager_h

#include <iostream>
#include <stdexcept>
#include <string>
#include <dlfcn.h>
#include <dirent.h>

class PluginManager
{
private:
  // void* handle;
  char* error;
  std::string directory;
  std::string pluginName; // this is w/o .so
  std::vector<RTSIIOInterfaceAPI*> allIOInterface;
  std::vector<RTSIReceiveInterfaceAPI*> allReceiveInterface;
  std::string hostip_;
  bool verbose_;
  double frequency_;

  RTSIIOInterfaceAPI* currentRTSIIOInterface;
  RTSIReceiveInterfaceAPI* currentRTSIReceiveInterface;

public:
  bool setIoInterfaceName(std::string IOName)
  {
    currentRTSIIOInterface = NULL;
    for (int i = 0; i < allIOInterface.size(); i++)
    {
      if (allIOInterface[i]->getUniqueName() == IOName)
      {
        currentRTSIIOInterface = allIOInterface[i];
        return true;
        break;
      }
    }
    return false;
  }

  bool setReceiveInterfaceName(std::string ReceiveName)
  {
    currentRTSIReceiveInterface = NULL;
    for (int i = 0; i < allReceiveInterface.size(); i++)
    {
      if (allReceiveInterface[i]->getUniqueName() == ReceiveName)
      {
        currentRTSIReceiveInterface = allReceiveInterface[i];
        return true;
        break;
      }
    }
    return false;
  }

  RTSIIOInterfaceAPI* getCurrentRTSIIOAPI()
  {
    if(currentRTSIIOInterface == NULL)
      throw std::exception();
    return currentRTSIIOInterface;
  }
  
  RTSIReceiveInterfaceAPI* getCurrentRTSIReceiveAPI()
  {
    if(currentRTSIReceiveInterface == NULL)
      throw std::exception();
    return currentRTSIReceiveInterface;
  }

  // PluginManager();
  // virtual ~PluginManager();

  std::pair<std::string,std::string> extension(char* f)
  {
    std::string file_name(f);
    int position = file_name.find_last_of(".");
    std::string name = file_name.substr(0,position);
    std::string ext = file_name.substr(position+1);
    return std::make_pair(name,ext);
  }

  void setHostIP(const std::string& hostip)
  {
    hostip_ = hostip;
  }

  void setVerbose(bool verbose)
  {
    verbose_ = verbose;
  }

  void setFrequency(bool frequency)
  {
    frequency_ = frequency;
  }

  void setPluginDirectory(std::string name)
  {
    directory = name;
  }

  void setPluginName(std::string name)
  {
    pluginName = name;
  }

  std::string getPluginName()
  {
    return pluginName;
  }

  void* loadPlugin()
  {
    void* handle;
    DIR *dir = opendir(directory.c_str());
    struct dirent *ent;
    if (dir != NULL)
    {
      std::cout << "Dir opened...\n";
      while ((ent = readdir(dir)) != NULL)
      {
        auto name = extension(ent->d_name).first;
        auto ext = extension(ent->d_name).second;
        if (ext.compare("so") != 0 || (pluginName.compare(name) != 0))
          continue;

        std::string fname(ent->d_name);
        fname = directory + fname;
        std::cout << "File name : " << fname << "\n"; 
        handle = dlopen(fname.c_str(), RTLD_LAZY);
        // std::cout << "Handle : " << handle << "\n";
        if (!handle || ((error = dlerror()) != NULL))
        {
          std::cerr << error << "\n";
          continue;
        }

        void* (*ioInstance)(std::string, bool, bool);
        ioInstance = (void* (*)(std::string, bool, bool))dlsym(handle, "createRTSIIOInstance");
        currentRTSIIOInterface =  static_cast<RTSIIOInterfaceAPI *>((*ioInstance)(hostip_, verbose_, false));
        allIOInterface.push_back(currentRTSIIOInterface);
        
        dlerror();

        void* (*receiveInstance)(std::string, double, std::vector<std::string>, bool);
        receiveInstance = (void* (*)(std::string, double, std::vector<std::string>, bool))dlsym(handle, "createRTSIReceiveInstance");
        currentRTSIReceiveInterface =  static_cast<RTSIReceiveInterfaceAPI *>((*receiveInstance)(hostip_, frequency_, {}, verbose_));
        allReceiveInterface.push_back(currentRTSIReceiveInterface);

        dlerror();
        dlclose(handle);
        break;
      }
      closedir(dir);
    }
    return handle;
  }
};

#endif