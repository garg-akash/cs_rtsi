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
			MOVEJ = 1,
	    SET_STD_DIGITAL_OUT = 13,
	    SET_SPEED_SLIDER = 22,
	    SET_STD_ANALOG_OUT = 23,
	    SET_CONF_DIGITAL_OUT = 48,
	    SET_INPUT_INT_REGISTER = 49,
	    SET_INPUT_DOUBLE_REGISTER = 50,
			SET_INPUT_BIT_REGISTER_X_TO_Y = 81,
			SET_INPUT_BIT_REGISTER = 82,
	    WATCHDOG = 99,
			SET_EXTERNAL_FORCE_TORQUE = 96,
			SERVOJ = 11
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
		std::int32_t async_;
	  std::int32_t reg_int_val_;
	  double reg_double_val_;
		std::uint32_t reg_bit_val_x_to_y_;
		bool reg_bit_val_;
	  std::vector<double> val_;
	  std::uint16_t std_digital_out_;
	  std::uint16_t std_digital_out_mask_;
	  std::uint8_t configurable_digital_out_;
	  std::uint8_t configurable_digital_out_mask_;
	  std::uint8_t std_analog_output_mask_;
	  std::uint8_t std_analog_output_type_;
	  double std_analog_output_0_;
	  double std_analog_output_1_;
	  std::int32_t speed_slider_mask_;
	  double speed_slider_fraction_;
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
	bool isStarted();
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

  static std::shared_ptr<RTSI> getRTSIInstance(const std::string hostip, int port, bool verbose)
  {
    static std::shared_ptr<RTSI> rtsi_(nullptr);
    if(rtsi_ == nullptr)
    {
      rtsi_ = std::make_shared<RTSI>(hostip, port, verbose);
      rtsi_->connect();
      rtsi_->negotiateProtocolVersion();
    }
    return rtsi_;
  }

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

	template <typename AsyncReadStream, typename MutableBufferSequence>
	std::size_t async_read_some(AsyncReadStream& s, const MutableBufferSequence& buffers,
	                            boost::system::error_code &error, int timeout_ms = -1);

  void check_deadline();
};