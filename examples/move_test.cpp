/*
Author: Elite_akashgarg
CreateDate: 2023-04-06
Description: example script to rtsi control interface - move commands
*/
#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/robot_state.h>
#include <cs_rtsi/rtsi_control_interface.h>

#include <chrono>

const int PORT = 30004;
const bool VERBOSE = true;

int main(int argc, char const *argv[])
{
	std::string hostip = "192.168.133.130";

	RTSIControlInterface rtsi_ctrl(hostip); //freq is by default 250
  
  std::vector<double> move_q1;
  move_q1.push_back(0);
  move_q1.push_back(-1.57);
  move_q1.push_back(-1.57);
  move_q1.push_back(-1.57);
  move_q1.push_back(1.57);
  move_q1.push_back(0);

  std::vector<double> move_q2;
  move_q2.push_back(1.57);
  move_q2.push_back(-1.57);
  move_q2.push_back(-1.57);
  move_q2.push_back(-1.57);
  move_q2.push_back(1.57);
  move_q2.push_back(0);

  for(int i = 0; i < 100; i++)
  {  
    // **********moveJ test**********
    if(i % 2 == 1)
    {
      rtsi_ctrl.moveJ(move_q1,5,1,0,0);
      std::cout << "Moved to q1\n";
    }
    else
    {
      rtsi_ctrl.moveJ(move_q2,5,1,0,0);
      std::cout << "Moved to q2\n";
    }

    sleep(1);
  }
	return 0;
}