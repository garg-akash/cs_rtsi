#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/rtsi_utility.h>

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
  // std::cout << "Version : " << unsigned(version) << " ; buffer : \n";
  // for(auto b : buffer)
  // 	std::cout << unsigned(b) << "\n";
  std::string payload(buffer.begin(), buffer.end());
  sendAll(cmd, payload);
  DEBUG("Done sending RTSI_REQUEST_PROTOCOL_VERSION");
  receive();
  return true;
}

bool RTSI::sendOutputSetup(const std::vector<std::string> &output_names, double frequency)
{
  std::uint8_t cmd = RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS;

  // First save the output_names for use in the receiveData function
  output_names_ = output_names;

  std::string freq_as_hexstr = RTSIUtility::double2hexstr(frequency);
  std::vector<char> freq_packed = RTSIUtility::hexToBytes(freq_as_hexstr);
  // Concatenate output_names to a single string
  std::string output_names_str;
  for (const auto &output_name : output_names)
    output_names_str += output_name + ",";

  std::copy(output_names_str.begin(), output_names_str.end(), std::back_inserter(freq_packed));
  std::string payload(std::begin(freq_packed), std::end(freq_packed));
  sendAll(cmd, payload);
  DEBUG("Done sending RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS");
  receive();
  return true;
}

void RTSI::sendAll(const std::uint8_t &command, std::string payload)
{
  DEBUG("Payload size is: " << payload.size());
  // Pack size and command into header
  uint16_t size = htons(HEADER_SIZE + (uint16_t)payload.size());
  uint8_t type = command;
  // std::cout << "payload: " << payload << " ; sz: " << size << " ; tp: " << unsigned(type) << "\n";
  char buffer[3];
  memcpy(buffer + 0, &size, sizeof(size));
  memcpy(buffer + 2, &type, sizeof(type));
  // std::cout << "buffer: \n";
  // for(auto a : buffer)
  // 	std::cout << unsigned(a) << "\n";
  // Create vector<char> that includes the header
  std::vector<char> header_packed;
  std::copy(buffer, buffer + sizeof(buffer), std::back_inserter(header_packed));
  // std::cout << "header_packed: \n";
  // for(auto a : header_packed)
  // 	std::cout << a << "\n";
  // Add the payload to the header_packed vector
  std::copy(payload.begin(), payload.end(), std::back_inserter(header_packed));
  // std::cout << "header_packed: \n";
  // for(auto a : header_packed)
  // 	std::cout << unsigned(a) << "\n";
  std::string sent(header_packed.begin(), header_packed.end());
  // std::cout << "sent: " << sent << "\n";
  DEBUG("SENDING buf containing: " << sent << " with len: " << sent.size());

  if (isConnected())
  {
    boost::asio::write(*socket_, boost::asio::buffer(header_packed, header_packed.size()));
  }
}

void RTSI::sendStart()
{
  std::uint8_t cmd = RTSI_CONTROL_PACKAGE_START;
  sendAll(cmd, "");
  DEBUG("Done sending RTSI_CONTROL_PACKAGE_START");
  receive();
}

void RTSI::receive()
{
	DEBUG("Receiving...");
	std::vector<char> data(HEADER_SIZE);
	boost::asio::read(*socket_, boost::asio::buffer(data));
	uint32_t message_offset = 0;
	uint16_t msg_size = RTSIUtility::getUInt16(data, message_offset);
	uint8_t msg_cmd = data.at(2);
	std::cout << unsigned(data[0]) << " " << unsigned(data[1]) << "\n";
	DEBUG("Control Header: ")
	DEBUG("size is: " << msg_size);
	DEBUG("command is: " << static_cast<int>(msg_cmd));

	data.resize(msg_size - HEADER_SIZE);
	boost::asio::read(*socket_, boost::asio::buffer(data));

	switch (msg_cmd)
	{
		case RTSI_TEXT_MESSAGE:
		{
			uint8_t msg_length = data.at(0);
			for (int i = 1; i < msg_length; i++)
			{
				DEBUG(data[i]);
			}
			break;
		}

		case RTSI_REQUEST_PROTOCOL_VERSION:
		{
			break;
		}

		case RTSI_GET_URCONTROL_VERSION:
		{
			DEBUG("ControlVersion: ");
			break;
		}

		case RTSI_CONTROL_PACKAGE_SETUP_INPUTS:
		{
			std::string datatypes(std::begin(data) + 1, std::end(data));
			DEBUG("Datatypes:" << datatypes);
			std::string in_use_str("IN_USE");
			if (datatypes.find(in_use_str) != std::string::npos)
			{
				throw std::runtime_error(
					"One of the RTSI input registers are already in use! Currently you must disable the EtherNet/IP adapter, "
            		"PROFINET or any MODBUS unit configured on the robot. This might change in the future.");
			}
			break;
		}

		case RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS:
	    {
	      std::string datatypes(std::begin(data) + 1, std::end(data));
	      DEBUG("Datatype:" << datatypes);
	      output_types_ = RTSIUtility::split(datatypes, ',');

	      std::string not_found_str("NOT_FOUND");
	      std::vector<int> not_found_indexes;
	      if (datatypes.find(not_found_str) != std::string::npos)
	      {
	        for (unsigned int i = 0; i < output_types_.size(); i++)
	        {
	          if (output_types_[i] == "NOT_FOUND")
	            not_found_indexes.push_back(i);
	        }

	        std::string vars_not_found;
	        for (unsigned int i = 0; i < not_found_indexes.size(); i++)
	        {
	          vars_not_found += output_names_[not_found_indexes[i]];
	          if (i != not_found_indexes.size() - 1)
	            vars_not_found += ", ";
	        }

	        std::string error_str(
	            "The following variables was not found by the controller: [" + vars_not_found +
	            "]\n ");
	        throw std::runtime_error(error_str);
	      }
	      break;
	    }

		case RTSI_CONTROL_PACKAGE_START:
	    {
	      char success = data.at(0);
	      DEBUG("success: " << static_cast<bool>(success));
	      auto rtsi_success = static_cast<bool>(success);
	      if (rtsi_success)
	      {
	        conn_state_ = ConnectionState::STARTED;
	        if (verbose_)
	          std::cout << "RTSI synchronization started" << std::endl;
	      }
	      else
	        std::cerr << "Unable to start synchronization" << std::endl;
	      break;
	    }

	    case RTSI_CONTROL_PACKAGE_PAUSE:
	    {
	      char success = data.at(0);
	      auto pause_success = static_cast<bool>(success);
	      DEBUG("success: " << pause_success);
	      if (pause_success)
	      {
	        conn_state_ = ConnectionState::PAUSED;
	        DEBUG("RTSI synchronization paused!");
	      }
	      else
	        std::cerr << "Unable to pause synchronization" << std::endl;
	      break;
	    }

	    default:
      		DEBUG("Unknown Command: " << static_cast<int>(msg_cmd));
      		break;
	}
}