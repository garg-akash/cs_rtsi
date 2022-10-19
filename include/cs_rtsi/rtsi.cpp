#include <cs_rtsi/rtsi.h>

#include <boost/asio/connect.hpp>
#include <boost/asio/detail/socket_option.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/socket_base.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind/bind.hpp>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

const unsigned HEADER_SIZE = 3;
#define RTSI_PROTOCOL_VERSION 2
#define DEBUG_OUTPUT true

#if DEBUG_OUTPUT
#define DEBUG(a)                                                \
  {                                                             \
    std::cout << "RTSI:" << __LINE__ << ": " << a << std::endl; \
  }
#else
#define DEBUG(a) \
  {              \
  }
#endif

using boost::asio::ip::tcp;

RTSI::RTSI(const std::string hostip, int port, bool verbose)
	: hostip_(std::move(hostip)),
	  port_(port),
	  verbose_(verbose),
	  conn_state_(ConnectionState::DISCONNECTED)
{
	
}

RTSI::~RTSI() = default;

void RTSI::connect()
{
	try
	{
		buffer_.clear();
		socket_.reset(new boost::asio::ip::tcp::socket(io_service_));
		socket_->open(boost::asio::ip::tcp::v4());
		boost::asio::ip::tcp::no_delay no_delay_option(true);
	    boost::asio::socket_base::reuse_address sol_reuse_option(true);
	    socket_->set_option(no_delay_option);
	    socket_->set_option(sol_reuse_option);
		resolver_ = std::make_shared<boost::asio::ip::tcp::resolver>(io_service_);
	    boost::asio::ip::tcp::resolver::query query(hostip_, std::to_string(port_));
	    boost::asio::connect(*socket_, resolver_->resolve(query));
	    conn_state_ = ConnectionState::CONNECTED;
	    if (verbose_)
	      std::cout << "Connected successfully to: " << hostip_ << " at " << port_ << std::endl;
	}
  	catch (const boost::system::system_error &error)
  	{
    	std::cerr << error.what() << std::endl;
    	std::string error_msg =
        	"Error: Could not connect to: " + hostip_ + " at " + std::to_string(port_) + ", verify the IP";
    	throw std::runtime_error(error_msg);
  	}
}

bool RTSI::isConnected()
{
  return conn_state_ == ConnectionState::CONNECTED || conn_state_ == ConnectionState::STARTED;
}

bool RTSI::negotiateProtocolVersion()
{
  std::uint8_t cmd = RTSI_REQUEST_PROTOCOL_VERSION;
  // Pack RTSI_PROTOCOL_VERSION into payload
  uint8_t null_byte = 0;
  uint8_t version = RTSI_PROTOCOL_VERSION;
  std::vector<char> buffer;
  buffer.push_back(null_byte);
  buffer.push_back(version);
  std::string payload(buffer.begin(), buffer.end());
  sendAll(cmd, payload);
  DEBUG("Done sending RTDE_REQUEST_PROTOCOL_VERSION");
  // receive();
  return true;
}

void RTSI::sendAll(const std::uint8_t &command, std::string payload)
{
  DEBUG("Payload size is: " << payload.size());
  // Pack size and command into header
  uint16_t size = htons(HEADER_SIZE + (uint16_t)payload.size());
  uint8_t type = command;

  char buffer[3];
  memcpy(buffer + 0, &size, sizeof(size));
  memcpy(buffer + 2, &type, sizeof(type));

  // Create vector<char> that includes the header
  std::vector<char> header_packed;
  std::copy(buffer, buffer + sizeof(buffer), std::back_inserter(header_packed));

  // Add the payload to the header_packed vector
  std::copy(payload.begin(), payload.end(), std::back_inserter(header_packed));

  std::string sent(header_packed.begin(), header_packed.end());
  DEBUG("SENDING buf containing: " << sent << " with len: " << sent.size());

  // This is a workaround for the moment to prevent crash when calling this
  // function is RTDE is disconnected - i.e. in case of desynchronization
  if (isConnected())
  {
    boost::asio::write(*socket_, boost::asio::buffer(header_packed, header_packed.size()));
  }
}