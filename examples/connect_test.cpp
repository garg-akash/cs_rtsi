/*
Author: Elite_akashgarg
CreateDate: 2022-11-09
Description: example script to test protocol negotiation
*/
#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/robot_state.h>

const int PORT = 30004;
const bool VERBOSE = true;

int main(int argc, char const *argv[])
{
	std::string hostip = "192.168.51.60";
	auto rtsi_ = std::make_shared<RTSI>(hostip, PORT, VERBOSE);
	rtsi_->connect();
	rtsi_->negotiateProtocolVersion();
	return 0;
}