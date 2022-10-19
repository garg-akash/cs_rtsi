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