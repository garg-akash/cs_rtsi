# CS RTSI
This is the standalone C++ library for the RTSI interface for the ELITE CS robot.

# Instructions

The code was tested with Ubuntu 20.04. Different OS versions are possible but not supported.

### Clone the main package

`cd <CS_LIB>/`

`git clone https://github.com/garg-akash/cs_rtsi.git`

### Compile 

`cd <CS_LIB>/cs_rtsi`

`mkdir build`

`cd  build/`

`cmake ..`

`make`

### Run

Before testing, run the robot script 'cs_robot_script.script' present inside the `robot_script/` dir in the CS simulator (preferably in single mode)

To test the movej/movel function

`cd build/`

`./move_test`

To test the movej - movel - servoj - stopscript functions

`cd build/`

`./control_test`