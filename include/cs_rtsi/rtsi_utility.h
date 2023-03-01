#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <iomanip>
#include <chrono>

#define SLACK_TIME_IN_MICROS 300UL

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

  static inline std::vector<double> unpackVector3d(const std::vector<char> &data, uint32_t &message_offset)
  {
  	std::vector<double> vector_3d;
  	for(unsigned int i = 0; i < 3; i++)
  	{
  		double d = getDouble(data, message_offset);
  		vector_3d.push_back(d);
  	}
  	return vector_3d;
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

  static inline bool getBool(const std::vector<char> &data, uint32_t &message_offset)
  {
    bool output = data[message_offset];
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
		((char *)(&output))[1] = data[message_offset];
		((char *)(&output))[0] = data[message_offset + 1];
		message_offset += 2;

		return output;
	}

  static inline uint16_t getUInt16(const std::vector<unsigned char> &data, uint32_t &message_offset)
  {
    uint16_t output = 0;
    ((char *)(&output))[1] = data[message_offset];
    ((char *)(&output))[0] = data[message_offset + 1];
    message_offset += 2;

    return output;
  }

	static inline uint32_t getUInt32(const std::vector<char> &data, uint32_t &message_offset)
  {
    uint32_t output = 0;
    ((char *)(&output))[3] = data[message_offset];
    ((char *)(&output))[2] = data[message_offset + 1];
    ((char *)(&output))[1] = data[message_offset + 2];
    ((char *)(&output))[0] = data[message_offset + 3];
    message_offset += 4;

    return output;
  }

  static inline uint64_t getUInt64(const std::vector<char> &data, uint32_t &message_offset)
  {
    uint64_t output;

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

  static inline int32_t getInt32(const std::vector<char> &data, uint32_t &message_offset)
  {
    int32_t output = 0;
    ((char *)(&output))[3] = data[message_offset];
    ((char *)(&output))[2] = data[message_offset + 1];
    ((char *)(&output))[1] = data[message_offset + 2];
    ((char *)(&output))[0] = data[message_offset + 3];
    message_offset += 4;
    return output;
  }

	static inline std::vector<int32_t> unpackVector6Int32(const std::vector<char> &data, uint32_t &message_offset)
  {
    std::vector<int32_t> vector_6_int32;
    for (unsigned int i = 0; i < 6; i++)
    {
      int32_t int32_value = getInt32(data, message_offset);
      vector_6_int32.push_back(int32_value);
    }
    return vector_6_int32;
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

  static inline std::vector<char> packBool(bool val)
  {
    std::vector<char> result;
    result.push_back(val);
    return result;
  }

  static inline std::vector<char> packUInt16(uint16_t uint16)
  {
    std::vector<char> result;
    result.push_back(uint16 >> 8); // Tested OK
    result.push_back(uint16);
    return result;
  }

  static inline std::vector<char> packUInt32(uint32_t uint32)
  {
    std::vector<char> result;
    result.push_back(uint32 >> 24); // Tested OK
    result.push_back(uint32 >> 16);
    result.push_back(uint32 >> 8);
    result.push_back(uint32);
    return result;
  }

  static inline std::vector<char> packInt32(int32_t int32)
  {
    std::vector<char> result;
    result.push_back(int32 >> 24);  // Tested OK
    result.push_back(int32 >> 16);
    result.push_back(int32 >> 8);
    result.push_back(int32);
    return result;
  }

  static inline std::vector<char> packVectorNInt32(std::vector<int32_t> vector_n_int32)
  {
    std::vector<char> result;
    for (auto i : vector_n_int32)
    {
      result.push_back(i >> 24);
      result.push_back(i >> 16);
      result.push_back(i >> 8);
      result.push_back(i);
    }

    return result;
  }

  static inline std::vector<char> packVectorNd(std::vector<double> vector_nd)
  {
    std::vector<char> output;

    for (auto d : vector_nd)
    {
      union temp
      {
        double value;
        char c[8];
      } in{}, out{};

      in.value = d;
      out.c[0] = in.c[7];
      out.c[1] = in.c[6];
      out.c[2] = in.c[5];
      out.c[3] = in.c[4];
      out.c[4] = in.c[3];
      out.c[5] = in.c[2];
      out.c[6] = in.c[1];
      out.c[7] = in.c[0];

      for (auto const &character : out.c)
        output.push_back(character);
    }

    return output;
  }

  static inline std::vector<char> packDouble(double d)
  {
    std::vector<char> output;
    union temp
    {
      double value;
      char c[8];
    } in{}, out{};

    in.value = d;
    out.c[0] = in.c[7];
    out.c[1] = in.c[6];
    out.c[2] = in.c[5];
    out.c[3] = in.c[4];
    out.c[4] = in.c[3];
    out.c[5] = in.c[2];
    out.c[6] = in.c[1];
    out.c[7] = in.c[0];

    for (auto const &character : out.c)  // Tested OK
      output.push_back(character);
    
    return output;
  }

  static timespec timepointToTimespec(std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> tp)
  {
    auto secs = std::chrono::time_point_cast<std::chrono::seconds>(tp);
    auto ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(tp) -
              std::chrono::time_point_cast<std::chrono::nanoseconds>(secs);

    return timespec{secs.time_since_epoch().count(), ns.count()};
  }

  static void waitFunction(const std::chrono::steady_clock::time_point& t_start, double delta_time)
  {
    auto t_stop = std::chrono::steady_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);
    if(t_duration.count() < delta_time)
    {
      auto cycle_time_in_ms = static_cast<int64_t>(delta_time * 1000);
      auto t_cycle_ideal_end = t_start + std::chrono::milliseconds(cycle_time_in_ms);
      auto t_cycle_end_with_slack = t_cycle_ideal_end - (std::chrono::microseconds(SLACK_TIME_IN_MICROS) +
                                                         t_duration);

      struct timespec tv_cycle_end_with_slack{}, tv_cycle_end{}, curr{};
      tv_cycle_end = timepointToTimespec(std::chrono::time_point_cast<std::chrono::nanoseconds>(t_cycle_ideal_end));
      tv_cycle_end_with_slack = timepointToTimespec(std::chrono::time_point_cast<std::chrono::nanoseconds>(t_cycle_end_with_slack));
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tv_cycle_end_with_slack, NULL);

      clock_gettime(CLOCK_MONOTONIC, &curr);
      for(; curr.tv_nsec < tv_cycle_end.tv_nsec; clock_gettime(CLOCK_MONOTONIC, &curr))
        ;
    }
  }
};