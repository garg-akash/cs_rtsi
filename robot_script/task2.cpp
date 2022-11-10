#include <arpa/inet.h>
#include <iostream>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

const int PORT = 30001;
const char HOSTIP[] = "192.168.133.129";

int main(int argc, char const *argv[])
{
  int sock = 0, valread, client_fd;
  struct sockaddr_in serv_addr;
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    std::cout << "Socket creation error\n";
    return -1;
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT);

  if (inet_pton(AF_INET, HOSTIP, &serv_addr.sin_addr) <= 0)
  {
    std::cout << "Invalid IP address\n";
    return -1;
  }

  if ((client_fd = connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr))) < 0)
  {
    std::cout << "Connection failed\n";
    return -1;
  }

  std::string cmd = "";
  std::string spacestr = "  ";
  std::string spacestr2 = "    ";
  std::string spacestr3 = "      ";
  std::string spacestr4 = "        ";
  cmd.append("def plotSin():\n");
  cmd.append(spacestr);
  cmd.append("global state_init\n");
  cmd.append(spacestr);
  cmd.append("while (True):\n");
  cmd.append(spacestr2);
  cmd.append("state_init = get_actual_tcp_pose()\n");
  cmd.append(spacestr2);
  cmd.append("flag_bit = read_input_boolean_register(64)\n");
  cmd.append(spacestr2);
  cmd.append("x = state_init[0]\n");
  cmd.append(spacestr2);
  cmd.append("y = state_init[1]\n");
  cmd.append(spacestr2);
  cmd.append("z = state_init[2]\n");
  cmd.append(spacestr2);
  cmd.append("Rx = state_init[3]\n");
  cmd.append(spacestr2);
  cmd.append("Ry = state_init[4]\n");
  cmd.append(spacestr2);
  cmd.append("Rz = state_init[5]\n");
  cmd.append(spacestr2);
  cmd.append("print(\"flag : \", flag_bit)\n");
  cmd.append(spacestr2);
  cmd.append("if (flag_bit):\n");
  cmd.append(spacestr3);
  cmd.append("x = read_input_float_register(0)\n");
  cmd.append(spacestr3);
  cmd.append("y = read_input_float_register(1)\n");
  cmd.append(spacestr3);
  cmd.append("z = read_input_float_register(2)\n");
  cmd.append(spacestr3);
  cmd.append("if (x != 0 and y != 0 and z != 0):\n");
  cmd.append(spacestr4);
  cmd.append("print(\"x : \", x)\n");
  cmd.append(spacestr4);
  cmd.append("print(\"y : \", y)\n");
  cmd.append(spacestr4);
  cmd.append("print(\"z : \", z)\n");
  cmd.append(spacestr4);
  cmd.append("movel([x,y,z,Rx,Ry,Rz],a=0.7,v=0.5,t=0.5,r=0)\n");
  cmd.append(spacestr);
  cmd.append("end\n");
  cmd.append("end\n");
  int len = cmd.length();
  char cmd_char[len+1];
  strcpy(cmd_char, cmd.c_str());
  if (send(sock, cmd_char, strlen(cmd_char), 0) < 0)
  {
    std::cout << "Error sending data to the controller\n";
    return -1;
  }
  close(client_fd);
  std::cout << "Done sending data\n";
  return 0;
}