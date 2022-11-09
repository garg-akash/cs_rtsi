/*
Author: Elite_akashgarg
CreateDate: 2022-11-09
Description: example script to test output subscription
*/
#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/robot_state.h>
#include <cs_rtsi/rtsi_receive_interface.h>
#include <thread>

const std::string hostip = "192.168.133.129";

int main(int argc, char const *argv[])
{
	double frequency = 1;
	std::vector<std::string> variables = {"timestamp",
					  					  "actual_joint_positions",
					  					  "actual_TCP_pose",
					  					  "payload_cog",
					  					  "robot_mode"};				  					  
	bool verbose = true;
	RTSIReceiveInterface rtsi_receive(hostip, frequency, variables, verbose);
	int steps = 400;
	while(steps--)
	{
		double ts = rtsi_receive.getTimestamp();
		std::cout << "\n step : " << steps << "\n";
		std::cout << "\nTimestamp is: " << ts << "\n";

		std::vector<double> joint_pos = rtsi_receive.getActualJointPositions();
		std::cout << "\nJoint Positions are: \n";
		for(auto j : joint_pos)
			std::cout << j << "\t";

		// std::vector<double> tcp_pos = rtsi_receive.getActualTCPPose();
		// std::cout << "\nTCP Pose is: \n";
		// for(auto j : tcp_pos)
		// 	std::cout << j << "\t";

		// std::vector<double> payload_cog = rtsi_receive.getPayloadCog();
		// std::cout << "\nPayload Cog is: \n";
		// for(auto j : payload_cog)
		// 	std::cout << j << "\t";

		std::int32_t robot_md = rtsi_receive.getRobotMode();
		std::cout << "\nRobot Mode is: " << robot_md << "\n";

  		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	return 0;
}