#pragma once

#include <string>

class RTSIUtility
{
  public:
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
};