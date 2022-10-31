include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(libRTSI rtsi.cpp robot_state.cpp rtsi_receive_interface.cpp rtsi_io_interface.cpp)
