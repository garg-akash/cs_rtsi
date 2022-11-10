/*
Author: Elite_akashgarg
CreateDate: 2022-11-09
Description: example script to test input & output subscription
*/
#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/robot_state.h>
#include <cs_rtsi/rtsi_receive_interface.h>
#include <cs_rtsi/rtsi_io_interface.h>
#include <thread>

const std::string hostip = "192.168.133.129";

int main(int argc, char const *argv[])
{
  double frequency = 250;
  double wave_T = 100;
  double wave_freq = 1/wave_T;
  double wave_amp = 0.1;
  std::vector<std::string> variables = {"actual_TCP_pose"};
  bool verbose = true;
  RTSIReceiveInterface rtsi_receive(hostip, frequency, variables, verbose);
  RTSIIOInterface rtsi_io(hostip, verbose);
  std::vector<double> tcp_pose = rtsi_receive.getActualTCPPose();
  std::cout << "InitX : " << tcp_pose[0] << "\n";
  std::cout << "InitY : " << tcp_pose[1] << "\n";
  std::cout << "InitZ : " << tcp_pose[2] << "\n";
  // rtsi_io.setInputBitRegister(64,true); // Set the flag bit to True (Enable the TCP position update)
  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  for(int t = 0; t < wave_T; t++)
  {
    double final_x = tcp_pose[0];
    double final_y = tcp_pose[1] + 0.001 * t;
    double final_z = tcp_pose[2] + wave_amp * sin(2 * M_PI * wave_freq * t);
    rtsi_io.setInputBitRegister(64,true); // Set the flag bit to True (Enable the TCP position update)
    rtsi_io.setInputDoubleRegister(0,final_x);
    rtsi_io.setInputDoubleRegister(1,final_y);
    rtsi_io.setInputDoubleRegister(2,final_z);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  rtsi_io.setInputBitRegister(64,false); // Set the flag bit to False (Disable the TCP position update) 
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  return 0;
}

