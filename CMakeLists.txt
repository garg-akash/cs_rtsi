cmake_minimum_required(VERSION 3.5)
project(cs_client_rtsi)

find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(Threads)
add_subdirectory(src)
find_package(jsoncpp REQUIRED)

add_executable(connect_test examples/connect_test.cpp)
target_include_directories(connect_test PUBLIC include)
target_link_directories(connect_test PRIVATE src)
target_link_libraries(connect_test libRTSI Threads::Threads)

add_executable(getData_test examples/getData_test.cpp)
target_include_directories(getData_test PUBLIC include)
target_link_directories(getData_test PRIVATE src)
target_link_libraries(getData_test libRTSI Boost::thread jsoncpp)

add_executable(setData_test examples/setData_test.cpp)
target_include_directories(setData_test PUBLIC include)
target_link_directories(setData_test PRIVATE src)
target_link_libraries(setData_test libRTSI Boost::thread)

add_executable(getSetData_test examples/getSetData_test.cpp)
target_include_directories(getSetData_test PUBLIC include)
target_link_directories(getSetData_test PRIVATE src)
target_link_libraries(getSetData_test libRTSI Boost::thread)
