/*
Author: Elite_akashgarg
CreateDate: 2022-11-09
Description: handles robot state parameters
*/
#include <cs_rtsi/robot_state.h>
#include <unordered_map>

std::unordered_map<std::string, rtsi_type_variant_> RobotState::state_types_ {
	{ "payload_cog", std::vector<double>() },
  { "payload_mass", double() },
  { "script_control_line", uint32_t() },
	{ "timestamp", double() },
  { "target_joint_positions", std::vector<double>() },
  { "target_joint_speeds", std::vector<double>() },
  { "target_joint_torques", std::vector<double>() },
  { "actual_joint_positions", std::vector<double>() },
  { "actual_joint_speeds", std::vector<double>() },
  { "actual_joint_current", std::vector<double>() },
  { "actual_joint_positions", std::vector<double>() },
	{ "actual_TCP_pose", std::vector<double>() },
	{ "actual_TCP_speed", std::vector<double>() },
	{ "target_TCP_pose", std::vector<double>() },
	{ "target_TCP_speed", std::vector<double>() },
	{ "actual_digital_input_bits", std::uint32_t() },
	{ "joint_temperatures", std::vector<double>() },
	{ "robot_mode", std::int32_t() },
	{ "joint_mode", std::vector<int32_t>() },
	{ "safety_mode", std::int32_t() },
	{ "safety_status", std::int32_t() },
	{ "speed_scaling", double() },
	{ "target_speed_fraction", double() },
	{ "actual_robot_voltage", double() },
	{ "actual_robot_current", double() },
	{ "actual_digital_output_bits", std::uint32_t() },
	{ "runtime_state", std::uint32_t() },
	{ "elbow_position", std::vector<double>() },
	{ "robot_status_bits", std::uint32_t() },
	{ "safety_status_bits", std::uint32_t() },
	{ "analog_io_types", std::uint32_t() }
};

RobotState::RobotState(const std::vector<std::string> &variables)
{
  initRobotState(variables);
}

RobotState::~RobotState() = default;

bool RobotState::lockUpdateStateMutex()
{
  update_state_mutex_.lock();
  return true;
}

bool RobotState::unlockUpdateStateMutex()
{
  update_state_mutex_.unlock();
  return true;
}

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
	std::lock_guard<std::mutex> lock(update_state_mutex_);
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