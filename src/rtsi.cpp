#include <cs_rtsi/robot_state.h>
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
#define RTSI_PROTOCOL_VERSION 1
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
	  conn_state_(ConnectionState::DISCONNECTED),
	  deadline_(io_service_)
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

void RTSI::disconnect()
{
	sendPause();
	socket_.reset();
	conn_state_ = ConnectionState::DISCONNECTED;
	if(verbose_)
		std::cout << "RTSI - Socket disconnected\n";
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

bool RTSI::sendInputSetup(const std::vector<std::string> &input_names)
{
  std::uint8_t cmd = RTSI_CONTROL_PACKAGE_SETUP_INPUTS;
  std::string input_names_str;
  for (const auto &input_name : input_names)
    input_names_str += input_name + ",";
  sendAll(cmd, input_names_str);
  DEBUG("Done sending RTSI_CONTROL_PACKAGE_SETUP_INPUTS");
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

  std::cout << "size 1 : " << freq_packed.size() << "\n";
  std::copy(output_names_str.begin(), output_names_str.end(), std::back_inserter(freq_packed));
  std::cout << "size 2 : " << freq_packed.size() << "\n";
  std::string payload(std::begin(freq_packed), std::end(freq_packed));
  sendAll(cmd, payload);
  DEBUG("Done sending RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS");
  receive();
  return true;
}

void RTSI::send(const RobotCommand &robot_cmd)
{
  std::uint8_t command = RTSI_DATA_PACKAGE;
  std::vector<char> cmd_packed;
  // cmd_packed = RTSIUtility::packInt32(robot_cmd.type_);

  // if (robot_cmd.type_ == RobotCommand::FT_RTSI_INPUT_ENABLE ||
  //     robot_cmd.type_ == RobotCommand::ENABLE_EXTERNAL_FT_SENSOR)
  // {
  //   std::vector<char> ft_rtsi_input_enable_packed = RTSIUtility::packInt32(robot_cmd.ft_rtsi_input_enable_);
  //   cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(ft_rtsi_input_enable_packed.begin()),
  //                     std::make_move_iterator(ft_rtsi_input_enable_packed.end()));
  // }

  // if (robot_cmd.type_ == RobotCommand::FREEDRIVE_MODE)
  // {
  //   std::vector<char> free_axes_packed = RTSIUtility::packVectorNInt32(robot_cmd.free_axes_);
  //   cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(free_axes_packed.begin()),
  //                     std::make_move_iterator(free_axes_packed.end()));
  // }

  if (robot_cmd.type_ == RobotCommand::SET_INPUT_INT_REGISTER)
  {
    std::vector<char> reg_int_packed = RTSIUtility::packInt32(robot_cmd.reg_int_val_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(reg_int_packed.begin()),
                      std::make_move_iterator(reg_int_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::SET_EE_POSITION)
  {
    std::vector<char> reg_double_packed1 = RTSIUtility::packDouble(robot_cmd.reg_double_val1_);
    std::vector<char> reg_double_packed2 = RTSIUtility::packDouble(robot_cmd.reg_double_val2_);
    std::vector<char> reg_double_packed3 = RTSIUtility::packDouble(robot_cmd.reg_double_val3_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(reg_double_packed1.begin()),
                      std::make_move_iterator(reg_double_packed1.end()));
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(reg_double_packed2.begin()),
                      std::make_move_iterator(reg_double_packed2.end()));
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(reg_double_packed3.begin()),
                      std::make_move_iterator(reg_double_packed3.end()));
  }

  if (robot_cmd.type_ == RobotCommand::SET_INPUT_DOUBLE_REGISTER)
  {
    std::cout << "Comes here...............\n" << robot_cmd.reg_double_val_ << "\n";
    std::vector<char> reg_double_packed = RTSIUtility::packDouble(robot_cmd.reg_double_val_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(reg_double_packed.begin()),
                      std::make_move_iterator(reg_double_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::WATCHDOG)
  {
    cmd_packed = RTSIUtility::packInt32(RobotCommand::NO_CMD);
  }

  // if (robot_cmd.type_ == RobotCommand::FORCE_MODE)
  // {
  //   std::vector<char> force_mode_type_packed = RTSIUtility::packInt32(robot_cmd.force_mode_type_);
  //   cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(force_mode_type_packed.begin()),
  //                     std::make_move_iterator(force_mode_type_packed.end()));

  //   std::vector<char> sel_vector_packed = RTSIUtility::packVectorNInt32(robot_cmd.selection_vector_);
  //   cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(sel_vector_packed.begin()),
  //                     std::make_move_iterator(sel_vector_packed.end()));
  // }

  // if (robot_cmd.type_ == RobotCommand::GET_ACTUAL_JOINT_POSITIONS_HISTORY)
  // {
  //   std::vector<char> actual_joint_positions_history_packed = RTSIUtility::packUInt32(robot_cmd.steps_);
  //   cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(actual_joint_positions_history_packed.begin()),
  //                     std::make_move_iterator(actual_joint_positions_history_packed.end()));
  // }

  if (!robot_cmd.val_.empty())
  {
    std::vector<char> vector_nd_packed = RTSIUtility::packVectorNd(robot_cmd.val_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(vector_nd_packed.begin()),
                      std::make_move_iterator(vector_nd_packed.end()));
  }

  // if (robot_cmd.type_ == RobotCommand::MOVEJ || robot_cmd.type_ == RobotCommand::MOVEJ_IK ||
  //     robot_cmd.type_ == RobotCommand::MOVEL || robot_cmd.type_ == RobotCommand::MOVEL_FK ||
  //     robot_cmd.type_ == RobotCommand::MOVE_PATH)
  // {
  //   std::vector<char> async_packed = RTSIUtility::packInt32(robot_cmd.async_);
  //   cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(async_packed.begin()),
  //                     std::make_move_iterator(async_packed.end()));
  // }

  if (robot_cmd.type_ == RobotCommand::SET_AK_DATA)
  {
    std::vector<char> std_digital_out_mask_packed = RTSIUtility::packUInt16(robot_cmd.std_digital_out_mask_);
    std::vector<char> std_digital_out_packed = RTSIUtility::packUInt16(robot_cmd.std_digital_out_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(std_digital_out_mask_packed.begin()),
                      std::make_move_iterator(std_digital_out_mask_packed.end()));
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(std_digital_out_packed.begin()),
                      std::make_move_iterator(std_digital_out_packed.end()));

    cmd_packed.push_back(robot_cmd.configurable_digital_out_mask_);
    cmd_packed.push_back(robot_cmd.configurable_digital_out_);
  }

  if (robot_cmd.type_ == RobotCommand::SET_STD_DIGITAL_OUT)
  {
    std::vector<char> std_digital_out_mask_packed = RTSIUtility::packUInt16(robot_cmd.std_digital_out_mask_);
    std::vector<char> std_digital_out_packed = RTSIUtility::packUInt16(robot_cmd.std_digital_out_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(std_digital_out_mask_packed.begin()),
                      std::make_move_iterator(std_digital_out_mask_packed.end()));
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(std_digital_out_packed.begin()),
                      std::make_move_iterator(std_digital_out_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::SET_CONF_DIGITAL_OUT)
  {
    cmd_packed.push_back(robot_cmd.configurable_digital_out_mask_);
    cmd_packed.push_back(robot_cmd.configurable_digital_out_);
  }

  // if (robot_cmd.type_ == RobotCommand::SET_TOOL_DIGITAL_OUT)
  // {
  //   cmd_packed.push_back(robot_cmd.std_tool_out_mask_);
  //   cmd_packed.push_back(robot_cmd.std_tool_out_);
  // }

  if (robot_cmd.type_ == RobotCommand::SET_SPEED_SLIDER)
  {
    std::cout << "Comes here...............\n" << robot_cmd.speed_slider_mask_ << " ; " << robot_cmd.speed_slider_fraction_ << "\n";
    std::vector<char> speed_slider_mask_packed = RTSIUtility::packInt32(robot_cmd.speed_slider_mask_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(speed_slider_mask_packed.begin()),
                      std::make_move_iterator(speed_slider_mask_packed.end()));

    std::vector<char> speed_slider_fraction_packed = RTSIUtility::packDouble(robot_cmd.speed_slider_fraction_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(speed_slider_fraction_packed.begin()),
                      std::make_move_iterator(speed_slider_fraction_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::SET_STD_ANALOG_OUT)
  {
    cmd_packed.push_back(robot_cmd.std_analog_output_mask_);
    cmd_packed.push_back(robot_cmd.std_analog_output_type_);
    std::vector<char> std_analog_output_0_packed = RTSIUtility::packDouble(robot_cmd.std_analog_output_0_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(std_analog_output_0_packed.begin()),
                      std::make_move_iterator(std_analog_output_0_packed.end()));
    std::vector<char> std_analog_output_1_packed = RTSIUtility::packDouble(robot_cmd.std_analog_output_1_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(std_analog_output_1_packed.begin()),
                      std::make_move_iterator(std_analog_output_1_packed.end()));
  }

  cmd_packed.insert(cmd_packed.begin(), robot_cmd.recipe_id_);
  std::string sent(cmd_packed.begin(), cmd_packed.end());

  sendAll(command, sent);
  DEBUG("Done sending RTSI_DATA_PACKAGE");
}

void RTSI::sendAll(const std::uint8_t &command, std::string payload)
{
  DEBUG("Payload is: " << payload);
  DEBUG("Payload size is: " << payload.size()); //this is in bytes
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
  std::cout << "header_packed (size and type): \n";
  for(auto a : header_packed)
  	std::cout << a << "\n";
  // Add the payload to the header_packed vector
  std::copy(payload.begin(), payload.end(), std::back_inserter(header_packed));
  std::cout << "header_packed (size, type and payload): \n";
  for(auto a : header_packed)
  	std::cout << unsigned(a) << "\n";
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

template <typename AsyncReadStream, typename MutableBufferSequence>
std::size_t RTSI::async_read_some(AsyncReadStream &s, const MutableBufferSequence &buffers,
                                  boost::system::error_code &ec, int timeout_ms)
{
  if (timeout_ms < 0)
  {
    timeout_ms = 2500;
  }

  deadline_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));

  // Set up the variable that receives the result of the asynchronous
  // operation. The error code is set to would_block to signal that the
  // operation is incomplete. Asio guarantees that its asynchronous
  // operations will never fail with would_block, so any other value in
  // ec indicates completion.
  ec = boost::asio::error::would_block;
  size_t bytes_received = 0;

  // Start the asynchronous operation itself. The boost::lambda function
  // object is used as a callback and will update the ec variable when the
  // operation completes.
  s.async_read_some(buffers,
                    [&](const boost::system::error_code &error, std::size_t bytes_transferred)
                    {
                      ec = error;
                      bytes_received = bytes_transferred;
                    });

  // Block until the asynchronous operation has completed.
  do
    io_service_.run_one();
  while (ec == boost::asio::error::would_block);
  if (ec)
  {
    throw std::system_error(ec);
  }

  return bytes_received;
}

bool RTSI::isDataAvailable()
{
	if(socket_->available() > 0) //number of bytes availbale for reading
		return true;
	else
		return false;
}

void RTSI::sendPause()
{
	std::uint8_t cmd = RTSI_CONTROL_PACKAGE_PAUSE;
	sendAll(cmd, "");
	DEBUG("Done sending RTSI_CONTROL_PACKAGE_PAUSE");
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
	std::cout << unsigned(data[0]) << " " << unsigned(data[1]) << " " << msg_size << "\n";
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
			uint8_t negotiate_result = data.at(0);
			std::cout << "Protocol negotiate result : " << unsigned(negotiate_result) << "\n";
			break;
		}

		case RTSI_GET_URCONTROL_VERSION:
		{
			DEBUG("ControlVersion: ");
			break;
		}

		case RTSI_CONTROL_PACKAGE_SETUP_INPUTS:
		{
			char id = data.at(0);
      DEBUG("ID:" << (int)id);
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
    	uint8_t subscription_id = data.at(0);
			std::cout << "Subscription id : " << unsigned(subscription_id) << "\n";
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

boost::system::error_code RTSI::receiveData(std::shared_ptr<RobotState> &robot_state)
{
  boost::system::error_code error;
  uint32_t message_offset = 0;
  uint32_t packet_data_offset = 0;

  // Prepare buffer of 4096 bytes
  std::vector<char> data(4096);
  size_t data_len = 0;

  data_len = async_read_some(*socket_, boost::asio::buffer(data), error);
  if (error)
    return error;

  // Add data to the buffer
  buffer_.insert(buffer_.end(), data.begin(), data.begin() + data_len);
  // std::cout << "In this !!!!!!!!!!!" << data_len << " ; " << buffer_.size() << "; "  << HEADER_SIZE << "\n";

  while (buffer_.size() >= HEADER_SIZE)
  {
    message_offset = 0;
    RTSIControlHeader packet_header = RTSIUtility::readRTSIHeader(buffer_, message_offset);
    std::cout << "RTSIControlHeader: " << std::endl;
    std::cout << "size is: " << packet_header.msg_size << std::endl;
    std::cout << "command is: " << static_cast<int>(packet_header.msg_cmd) << std::endl;

    if (buffer_.size() >= packet_header.msg_size)
    {
      // Read data package and adjust buffer
      std::vector<char> packet(buffer_.begin() + HEADER_SIZE, buffer_.begin() + packet_header.msg_size);
      buffer_.erase(buffer_.begin(), buffer_.begin() + packet_header.msg_size);
      // std::cout << "After erase buffer size is : " << buffer_.size() << "\n";
      if (buffer_.size() >= HEADER_SIZE && packet_header.msg_cmd == RTSI_DATA_PACKAGE)
      {
        RTSIControlHeader next_packet_header = RTSIUtility::readRTSIHeader(buffer_, message_offset);
        // std::cout << "2nd if " << static_cast<int>(next_packet_header.msg_cmd) << std::endl;
        if (next_packet_header.msg_cmd == RTSI_DATA_PACKAGE)
        {
          if (verbose_)
            std::cout << "skipping package(1)" << std::endl;
          continue;
        }
      }

      if (packet_header.msg_cmd == RTSI_DATA_PACKAGE)
      {
        packet_data_offset = 0;
        char subs_id = RTSIUtility::getUChar(packet, packet_data_offset);
        std::cout << "subscription_id : " << unsigned(subs_id) << "\n";
        robot_state->lockUpdateStateMutex();

        // Read all the variables specified by the user.
        for (const auto &output_name : output_names_)
        {
          if (robot_state->state_types_.find(output_name) != robot_state->state_types_.end())
          {
            rtsi_type_variant_ entry = robot_state->state_types_[output_name];
            if (entry.type() == typeid(std::vector<double>))
            {
              std::vector<double> parsed_data;
              if (output_name == "payload_cog" || output_name == "elbow_position")
              {  
                parsed_data = RTSIUtility::unpackVector3d(packet, packet_data_offset);
              	std::cout << "Parsing 3D\n";
              }
              else
              {  
                parsed_data = RTSIUtility::unpackVector6d(packet, packet_data_offset);
              	std::cout << "Parsing 6D\n";
              }
              robot_state->setStateData(output_name, parsed_data);
            }
            else if (entry.type() == typeid(double))
            {
              std::cout << "Parsing 1D\n";
              double parsed_data = RTSIUtility::getDouble(packet, packet_data_offset);
              robot_state->setStateData(output_name, parsed_data);
            }
            else if (entry.type() == typeid(int32_t))
            {
              int32_t parsed_data = RTSIUtility::getInt32(packet, packet_data_offset);
              robot_state->setStateData(output_name, parsed_data);
            }
            else if (entry.type() == typeid(uint32_t))
            {
              uint32_t parsed_data = RTSIUtility::getUInt32(packet, packet_data_offset);
              robot_state->setStateData(output_name, parsed_data);
            }
            else if (entry.type() == typeid(uint64_t))
            {
              uint64_t parsed_data = RTSIUtility::getUInt64(packet, packet_data_offset);
              robot_state->setStateData(output_name, parsed_data);
            }
            else if (entry.type() == typeid(std::vector<int32_t>))
            {
              std::vector<int32_t> parsed_data = RTSIUtility::unpackVector6Int32(packet, packet_data_offset);
              robot_state->setStateData(output_name, parsed_data);
            }
          }
          else
          {
            DEBUG("Unknown variable name: " << output_name << " please verify the output setup!");
          }
        }

        if (!robot_state->getFirstStateReceived())
          robot_state->setFirstStateReceived(true);

        robot_state->unlockUpdateStateMutex();
      }
      else
      {
        if (verbose_)
          std::cout << "skipping package(2)" << std::endl;
      }
    }
    else
    {
      break;
    }
  }
  return error;
}