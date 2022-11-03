#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/robot_state.h>
#include <cs_rtsi/rtsi_receive_interface.h>
#include <cs_rtsi/rtsi_io_interface.h>
#include <thread>

const std::string hostip = "192.168.51.139";

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
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  for(int t = 0; t < wave_T; t++)
  {
    double final_x = tcp_pose[0];
    double final_y = tcp_pose[1] + 0.001 * t;
    double final_z = tcp_pose[2] + wave_amp * sin(2 * M_PI * wave_freq * t);
    rtsi_io.setInputDoubleRegisterPosition(final_x, final_y, final_z);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  rtsi_io.setInputDoubleRegisterPosition(0, 0, 0);
  std::cout << "IO set\n";

  return 0;
}

