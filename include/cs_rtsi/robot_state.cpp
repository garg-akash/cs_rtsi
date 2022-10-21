#include <cs_rtsi/robot_state.h>
#include <unordered_map>

std::unordered_map<std::string, rtsi_type_variant_> RobotState::state_types_ {
	{ "timestamp", double() },
	{ "actual_joint_positions", std::vector<double>() },
	{ "actual_TCP_pose", std::vector<double>() }
};

RobotState::RobotState(const std::vector<std::string> &variables)
{
  	initRobotState(variables);
}

RobotState::~RobotState() = default;

void RobotState::setFirstStateReceived(bool val)
{
  	first_state_received_ = val;
}

bool RobotState::getFirstStateReceived()
{
  	return first_state_received_;
}

void RobotState::initRobotState(const std::vector<std::string> &variables)
{
	for (auto& item : variables)
  	{
    	if (state_types_.find(item) != state_types_.end())
    	{
      		rtsi_type_variant_ entry = state_types_[item];
      		state_data_[item] = entry;
    	}
  	}
  	first_state_received_ = false;
}