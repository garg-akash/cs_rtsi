/*
Author: Elite_akashgarg
CreateDate: 2023-04-07
Description: example script to rtsi control interface - multiple commands
*/
#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/robot_state.h>
#include <cs_rtsi/rtsi_control_interface.h>

#include <chrono>

std::string hostip = "192.168.133.130";

int main(int argc, char const *argv[])
{
	RTSIControlInterface rtsi_ctrl(hostip); //freq is by default 250
  
  std::vector<double> move_q1;
  move_q1.push_back(0);
  move_q1.push_back(-1.57);
  move_q1.push_back(-1.57);
  move_q1.push_back(-1.57);
  move_q1.push_back(1.57);
  move_q1.push_back(0);

  rtsi_ctrl.moveJ(move_q1,5,1,0,0);
  std::cout << "Moved to q1\n";

  double dt = 1.0/250; // 4ms
  double lookahead_time = 0.1;
  double gain = 300;

  for(int i = 0; i < 100; i++)
  {
    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
    rtsi_ctrl.servoJ(move_q1, dt, lookahead_time, gain);
    move_q1[0] -= 0.01;
    std::cout << "servoj took : " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_start).count() << "s\n";
    sleep(1);
  }
  std::cout << "Commands executed\n";
	return 0;
}