cmake_minimum_required(VERSION 3.5)
project(cs_client_rtsi)

set(LIB_HEADER_FILES
	include/cs_rtsi/rtsi.h
	)

set(LIB_SOURCE_FILES
	src/rtsi.cpp
	)

file(GLOB SOURCES
	include/cs_rtsi/rtsi.h
	src/rtsi.cpp
	)

include_directories(
	"include"
	)

find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(Threads)

add_executable(connect_test src/connect_test.cpp)

target_link_libraries(connect_test Threads::Threads)

add_executable(getData_test src/getData_test.cpp)

target_link_libraries(getData_test Boost::thread)