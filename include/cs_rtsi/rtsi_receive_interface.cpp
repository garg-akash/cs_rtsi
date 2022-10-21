#include <cs_rtsi/robot_state.h>
#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/rtsi_receive_interface.h>

#include <boost/thread/thread.hpp>
#include <chrono>
#include <iostream>
#include <thread>

RTSIReceiveInterface::RTSIReceiveInterface(std::string hotsip, double frequency, std::vector<std::string> variables,
										   bool verbose)
	: hostip_(std::move(hotsip)),
	  frequency_(frequency),
	  variables_(std::move(variables)),
	  verbose_(verbose)
{
	if(verbose_)
	{
		std::cout << "RTSIReceiveInterface: initiated\n";
	}

	port_ = 30004;
	rtsi_ = std::make_shared<RTSI>(hostip_, port_, verbose_);
	rtsi_->connect();
	rtsi_->negotiateProtocolVersion();
	frequency_ = 250;
	delta_time_ = 1 / frequency_;

	setupRecipes(frequency_);

	robot_state_ = std::make_shared<RobotState>(variables_);

  	rtsi_->sendStart();

  	th_ = std::make_shared<boost::thread>(boost::bind(&RTSIReceiveInterface::receiveCallback, this));

	while(!robot_state_->getFirstStateReceived())
	{
		std::this_thread::sleep_for(std::chrono::microseconds(100));
	}
}

bool RTSIReceiveInterface::setupRecipes(const double& frequency)
{
	if(variables_.empty())
	{
		variables_ = {"timestamp",
					  "actual_joint_positions",
					  "actual_TCP_pose"};
	}
	// Setup output
  	rtsi_->sendOutputSetup(variables_, frequency);
  	return true;
}

void RTSIReceiveInterface::receiveCallback()
{
  	while (!stop_receive_thread)
  	{
    	// Receive and update the robot state
    	try
    	{
      		if (rtsi_->isDataAvailable())
      		{
        	no_bytes_avail_cnt_ = 0;
        	boost::system::error_code ec = rtsi_->receiveData(robot_state_);
        		if(ec)
        		{
          			if(ec == boost::asio::error::eof)
          			{
            			std::cerr << "RTSIReceiveInterface: Robot closed the connection!" << std::endl;
          			}
          			throw std::system_error(ec);
        		}
      		}
      		else
      		{
        		// Register that data was not available in this cycle
        		no_bytes_avail_cnt_++;
        		// If at least 2ms has passed without data available, try to read data again to detect de-synchronization.
        		if (no_bytes_avail_cnt_ > 20)
        		{
          			boost::system::error_code ec = rtsi_->receiveData(robot_state_);
         			if(ec)
          			{
            			if(ec == boost::asio::error::eof)
            			{
              				std::cerr << "RTSIReceiveInterface: Robot closed the connection!" << std::endl;
            			}
            			throw std::system_error(ec);
        			}
          			no_bytes_avail_cnt_ = 0;
        		}

// #if defined(__linux__) || defined(__APPLE__)
//         // Data not available on socket yet, yield to other threads and sleep before trying to receive data again.
//         std::this_thread::yield();
//         std::this_thread::sleep_for(std::chrono::microseconds(100));
// #endif
    		}
  		}
		catch (std::exception& e)
		{
		  std::cerr << "RTSIReceiveInterface Exception: " << e.what() << std::endl;
		  if (rtsi_->isConnected())
		    rtsi_->disconnect();
		  stop_receive_thread = true;
		  // stop_record_thread = true;
		}
	}
}

std::vector<double> RTSIReceiveInterface::getActualJointPositions()
{
	std::vector<double> actual_joint_positions;
	if (robot_state_->getStateData("actual_joint_positions", actual_joint_positions))
		return actual_joint_positions;
	else
		throw std::runtime_error("unable to get state data for actual_joint_positions");
}
