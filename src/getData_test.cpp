#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/rtsi.cpp>
#include <cs_rtsi/robot_state.h>
#include <cs_rtsi/robot_state.cpp>
#include <cs_rtsi/rtsi_receive_interface.h>
#include <cs_rtsi/rtsi_receive_interface.cpp>

const std::string hostip = "192.168.51.60";

int main(int argc, char const *argv[])
{
	RTSIReceiveInterface rtsi_receive(hostip);
	std::vector<double> joint_pos = rtsi_receive.getActualJointPositions();
	std::cout << "Joint Positions are: \n";
	for(auto j : joint_pos)
		std::cout << j << "\t";
	return 0;
}