#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/rtsi.cpp>
#include <cs_rtsi/robot_state.h>
#include <cs_rtsi/robot_state.cpp>
#include <cs_rtsi/rtsi_receive_interface.h>
#include <cs_rtsi/rtsi_receive_interface.cpp>

const std::string hostip = "192.168.51.60";

int main(int argc, char const *argv[])
{
	double frequency = 250;
	std::vector<std::string> variables = {"timestamp",
					  					  "actual_joint_positions",
					  					  "actual_TCP_pose"};
	bool verbose = true;
	RTSIReceiveInterface rtsi_receive(hostip, frequency, variables, verbose);
	int STEPS = 10;
	while(STEPS--)
	{
		// double ts = rtsi_receive.getTimestamp();
		// std::cout << "Timestamp is: " << ts << "\n";

		std::vector<double> joint_pos = rtsi_receive.getActualJointPositions();
		std::cout << "\nJoint Positions are: \n";
		for(auto j : joint_pos)
			std::cout << j << "\t";

		// std::vector<double> tcp_pos = rtsi_receive.getActualTCPPose();
		// std::cout << "TCP Pose is: \n";
		// for(auto j : tcp_pos)
		// 	std::cout << j << "\t";
	}
	return 0;
}