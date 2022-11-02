#pragma once

#include <cs_rtsi/robot_state.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class RTSI
{
public:
	RTSI(const std::string hostip, int port = 30004, bool verbose = false);
	virtual ~RTSI();

	class RobotCommand
	{
	public:
	  enum Type
	  {
	    NO_CMD = 0,
	    // MOVEJ = 1,
	    // MOVEJ_IK = 2,
	    // MOVEL = 3,
	    // MOVEL_FK = 4,
	    // FORCE_MODE = 6,
	    // FORCE_MODE_STOP = 7,
	    // ZERO_FT_SENSOR = 8,
	    // SPEEDJ = 9,
	    // SPEEDL = 10,
	    // SERVOJ = 11,
	    // SERVOC = 12,
	    SET_STD_DIGITAL_OUT = 13,
	    // SET_TOOL_DIGITAL_OUT = 14,
	    // SPEED_STOP = 15,
	    // SERVO_STOP = 16,
	    // SET_PAYLOAD = 17,
	    // TEACH_MODE = 18,
	    // END_TEACH_MODE = 19,
	    // FORCE_MODE_SET_DAMPING = 20,
	    // FORCE_MODE_SET_GAIN_SCALING = 21,
	    SET_SPEED_SLIDER = 22,
	    SET_STD_ANALOG_OUT = 23,
	    // SERVOL = 24,
	    // TOOL_CONTACT = 25,
	    // GET_STEPTIME = 26,
	    // GET_ACTUAL_JOINT_POSITIONS_HISTORY = 27,
	    // GET_TARGET_WAYPOINT = 28,
	    // SET_TCP = 29,
	    // GET_INVERSE_KINEMATICS_ARGS = 30,
	    // PROTECTIVE_STOP = 31,
	    // STOPL = 33,
	    // STOPJ = 34,
	    // SET_WATCHDOG = 35,
	    // IS_POSE_WITHIN_SAFETY_LIMITS = 36,
	    // IS_JOINTS_WITHIN_SAFETY_LIMITS = 37,
	    // GET_JOINT_TORQUES = 38,
	    // POSE_TRANS = 39,
	    // GET_TCP_OFFSET = 40,
	    // JOG_START = 41,
	    // JOG_STOP = 42,
	    // GET_FORWARD_KINEMATICS_DEFAULT = 43,
	    // GET_FORWARD_KINEMATICS_ARGS = 44,
	    // MOVE_PATH = 45,
	    // GET_INVERSE_KINEMATICS_DEFAULT = 46,
	    // IS_STEADY = 47,
	    SET_CONF_DIGITAL_OUT = 48,
	    SET_INPUT_INT_REGISTER = 49,
	    SET_INPUT_DOUBLE_REGISTER = 50,
	    // MOVE_UNTIL_CONTACT = 51,
	    // FREEDRIVE_MODE = 52,
	    // END_FREEDRIVE_MODE = 53,
	    // GET_FREEDRIVE_STATUS = 54,
	    // SET_EXTERNAL_FORCE_TORQUE = 55,
	    // FT_RTSI_INPUT_ENABLE = 56,
	    // ENABLE_EXTERNAL_FT_SENSOR = 57,
	    // GET_ACTUAL_TOOL_FLANGE_POSE = 58,
	    // SET_GRAVITY = 59,
	    WATCHDOG = 99,
	    // STOP_SCRIPT = 255
      SET_AK_DATA = 89
	  };

	  enum Recipe
	  {
	    RECIPE_1 = 1,
	    RECIPE_2 = 2,
	    RECIPE_3 = 3,
	    RECIPE_4 = 4,
	    RECIPE_5 = 5,
	    RECIPE_6 = 6,
	    RECIPE_7 = 7,
	    RECIPE_8 = 8,
	    RECIPE_9 = 9,
	    RECIPE_10 = 10,
	    RECIPE_11 = 11,
	    RECIPE_12 = 12,
	    RECIPE_13 = 13,
	    RECIPE_14 = 14,
	    RECIPE_15 = 15,
	    RECIPE_16 = 16,
	    RECIPE_17 = 17,
	    RECIPE_18 = 18,
	    RECIPE_19 = 19
	  };

	  // RobotCommand() : type_(NO_CMD), recipe_id_(1)
	  // {
	  // }

	  Type type_;// = NO_CMD;
	  std::uint8_t recipe_id_;
	  // std::int32_t async_;
	  // std::int32_t ft_rtsi_input_enable_;
	  std::int32_t reg_int_val_;
	  double reg_double_val_;
	  std::vector<double> val_;
	  // std::vector<int> selection_vector_;
	  // std::vector<int> free_axes_;
	  // std::int32_t force_mode_type_;
	  std::uint16_t std_digital_out_;
	  std::uint16_t std_digital_out_mask_;
	  std::uint8_t configurable_digital_out_;
	  std::uint8_t configurable_digital_out_mask_;
	  // std::uint8_t std_tool_out_;
	  // std::uint8_t std_tool_out_mask_;
	  std::uint8_t std_analog_output_mask_;
	  std::uint8_t std_analog_output_type_;
	  double std_analog_output_0_;
	  double std_analog_output_1_;
	  std::int32_t speed_slider_mask_;
	  double speed_slider_fraction_;
	  // std::uint32_t steps_;
	};

	enum RTSICommand
	{
	    RTSI_REQUEST_PROTOCOL_VERSION = 86,       // ascii V
	    RTSI_GET_URCONTROL_VERSION = 118,         // ascii v
	    RTSI_TEXT_MESSAGE = 77,                   // ascii M
	    RTSI_DATA_PACKAGE = 85,                   // ascii U
	    RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS = 79,  // ascii O
	    RTSI_CONTROL_PACKAGE_SETUP_INPUTS = 73,   // ascii I
	    RTSI_CONTROL_PACKAGE_START = 83,          // ascii S
	    RTSI_CONTROL_PACKAGE_PAUSE = 80           // ascii P
	};

	enum class ConnectionState : std::uint8_t
  	{
    	DISCONNECTED = 0,
    	CONNECTED = 1,
    	STARTED = 2,
    	PAUSED = 3
  	};

	void connect();
	void disconnect();
	bool isConnected();
	bool isDataAvailable();
	bool negotiateProtocolVersion();
	void send(const RobotCommand &robot_cmd);
	void sendAll(const std::uint8_t &command, std::string payload = "");
	void sendPause();
	void receive();
	boost::system::error_code receiveData(std::shared_ptr<RobotState> &robot_state);

	void sendStart();
	bool sendInputSetup(const std::vector<std::string> &input_names);
	bool sendOutputSetup(const std::vector<std::string> &output_names, double frequency);

private:
	std::string hostip_;
	int port_;
	bool verbose_;
	ConnectionState conn_state_;
	std::vector<std::string> output_types_;
	std::vector<std::string> output_names_;
	boost::asio::io_service io_service_;
	std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  	std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
  	std::vector<char> buffer_;
  	boost::asio::deadline_timer deadline_;

  	/**
  	 * Async socket read function with timeout.
  	 * If no timeout is given (value < 0) then an internal default timeout
  	 * value is used
  	 * \return Bytes received
  	 */
  	template <typename AsyncReadStream, typename MutableBufferSequence>
  	std::size_t async_read_some(AsyncReadStream& s, const MutableBufferSequence& buffers,
  	                            boost::system::error_code &error, int timeout_ms = -1);

};