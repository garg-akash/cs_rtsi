#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <iomanip>

struct RTSIControlHeader
{
  uint16_t msg_size;
  uint8_t msg_cmd;
};

class RTSIUtility
{
  public:

  static inline RTSIControlHeader readRTSIHeader(const std::vector<char> &data, uint32_t &message_offset)
  {
    RTSIControlHeader rtsi_control_header{};
    rtsi_control_header.msg_size = RTSIUtility::getUInt16(data, message_offset);
    rtsi_control_header.msg_cmd = RTSIUtility::getUInt8(data, message_offset);
    return rtsi_control_header;
  }

  static inline double getDouble(const std::vector<char> &data, uint32_t &message_offset)
  {
  	double output;

  	((char *)(&output))[7] = data[message_offset];
  	((char *)(&output))[6] = data[message_offset + 1];
  	((char *)(&output))[5] = data[message_offset + 2];
  	((char *)(&output))[4] = data[message_offset + 3];
  	((char *)(&output))[3] = data[message_offset + 4];
  	((char *)(&output))[2] = data[message_offset + 5];
  	((char *)(&output))[1] = data[message_offset + 6];
  	((char *)(&output))[0] = data[message_offset + 7];
  
  	message_offset += 8;
  	return output;
  }

  static inline std::vector<double> unpackVector6d(const std::vector<char> &data, uint32_t &message_offset)
  {
  	std::vector<double> vector_6d;
  	for(unsigned int i = 0; i < 6; i++)
  	{
  		double d = getDouble(data, message_offset);
  		vector_6d.push_back(d);
  	}
  	return vector_6d;
  }
  static inline unsigned char getUChar(const std::vector<char> &data, uint32_t &message_offset)
  {
  	unsigned char output = data[message_offset];
  	message_offset += 1;
  	return output;
  }

  static inline uint8_t getUInt8(const std::vector<char> &data, uint32_t &message_offset)
  {
  	uint8_t output = data[message_offset];
  	message_offset += 1;
  	return output;
  }
	static inline uint16_t getUInt16(const std::vector<char> &data, uint32_t &message_offset)
	{
		uint16_t output = 0;
		((char *)(&output))[1] = data[message_offset + 0];
		((char *)(&output))[0] = data[message_offset + 1];
		message_offset += 2;

		return output;
	}

	static inline std::vector<std::string> split(const std::string &s, char delimiter)
  {
   	std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
      tokens.push_back(token);
    }
    return tokens;
  }

  static inline std::string double2hexstr(double x)
  {
    union
    {
      long long i;
      double d;
    } value;

    value.d = x;

    std::ostringstream buf;
    buf << std::hex << std::setw(6) << value.i;
    return buf.str();
  }

  static inline std::vector<char> hexToBytes(const std::string &hex)
  {
    std::vector<char> bytes;

    for (unsigned int i = 0; i < hex.length(); i += 2)
    {
      std::string byteString = hex.substr(i, 2);
      char byte = (char)strtol(byteString.c_str(), nullptr, 16);
      bytes.push_back(byte);
    }

    return bytes;
  }
};