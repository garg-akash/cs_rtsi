/*
Author: Elite_akashgarg
CreateDate: 2023-02-09
Description: example script to rtsi control interface
*/
#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/robot_state.h>
#include <cs_rtsi/rtsi_control_interface.h>

#include <chrono>

const int PORT = 30004;
const bool VERBOSE = true;

int main(int argc, char const *argv[])
{
	std::string hostip = "192.168.133.129";
	RTSIControlInterface rtsi_ctrl(hostip); //freq is by default 250
  
  std::vector<double> q(6,0);
  double dt = 1.0/250; // 4ms
  double lookahead_time = 0.1;
  double gain = 300;

  for(int i = 0; i < 100; i++)
  {
    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
    rtsi_ctrl.servoJ(q, dt, lookahead_time, gain);
    q[1] -= 0.01;
    q[3] -= 0.01;
    std::cout << "took : " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_start).count() << "s\n";
    sleep(1);
  }
	return 0;
}