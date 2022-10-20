# pragma once

#include <boost/variant>
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
	
 private:	
 	std::unordered_map<std::string, rtsi_type_variant_> state_data_;
 	bool first_state_received_;
};