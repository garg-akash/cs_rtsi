#pragma once

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
	bool negotiateProtocolVersion();
	void sendAll(const std::uint8_t &command, std::string payload = "");


private:
	std::string hostip_;
	int port_;
	bool verbose_;
	ConnectionState conn_state_;
	boost::asio::io_service io_service_;
	std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  	std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
  	std::vector<char> buffer_;
};