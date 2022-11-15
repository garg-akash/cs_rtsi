# pragma once

#include <boost/variant.hpp>
#include <vector>
#include <unordered_map>
#include <sstream>
#include <mutex>
#include <iostream>

using rtsi_type_variant_ = boost::variant<bool, uint8_t, uint32_t, uint64_t, int32_t, double, std::vector<double>,
    std::vector<int32_t>>;

class RobotState
{
 public:
	RobotState(const std::vector<std::string> &variables);
	~RobotState();

	bool lockUpdateStateMutex();

	bool unlockUpdateStateMutex();

	void setFirstStateReceived(bool val);

	bool getFirstStateReceived();

	void initRobotState(const std::vector<std::string> &variables);

	static std::unordered_map<std::string, rtsi_type_variant_> state_types_;
	
	template <typename T> bool
	getStateData(const std::string& name, T& val)
	{
	    // for(auto i : state_data_)
	    // 	std::cout << " state : " << i.first << "\n";
	    std::lock_guard<std::mutex> lock(update_state_mutex_);
	
	    if (state_data_.find(name) != state_data_.end())
	    {
	     	val = boost::strict_get<T>(state_data_[name]);
	    }
	    else
	    {
	      	return false;
	    }
	    return true;
	};

	template <typename T>
  	bool setStateData(const std::string& name, T& val)
  	{
    	if (state_data_.find(name) != state_data_.end())
    	{
      		state_data_[name] = val;
    	}
    	else
    	{
      		return false;
    	}
    	return true;
  	};

 private:	
 	std::unordered_map<std::string, rtsi_type_variant_> state_data_;
 	std::mutex update_state_mutex_;
 	bool first_state_received_;
};