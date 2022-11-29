/*
Author: Elite_akashgarg
CreateDate: 2022-11-29
LastEdited: 2022-11-29
Description: script to handle dynamic loading of libraries
*/
#ifndef plugInManager_h
#define plugInManager_h

#include <iostream>
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

public:
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