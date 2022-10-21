# pragma once

#include <boost/variant.hpp>
#include <vector>
#include <unordered_map>
#include <sstream>

using rtsi_type_variant_ = boost::variant<uint32_t, uint64_t, int32_t, double, std::vector<double>,
    std::vector<int32_t>>;

class RobotState
{
 public:
	RobotState(const std::vector<std::string> &variables);
	~RobotState();

	void setFirstStateReceived(bool val);

	bool getFirstStateReceived();

	void initRobotState(const std::vector<std::string> &variables);

	static std::unordered_map<std::string, rtsi_type_variant_> state_types_;
	
	template <typename T> 
	bool getStateData(const std::string& name, T& val)
	{
	// #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
	//     std::lock_guard<std::mutex> lock(update_state_mutex_);
	// #else
	//     std::lock_guard<PriorityInheritanceMutex> lock(update_state_mutex_);
	// #endif
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
 	bool first_state_received_;
};