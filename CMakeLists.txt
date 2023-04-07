cmake_minimum_required(VERSION 3.5)
project(cs_rtsi)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/" ${CMAKE_MODULE_PATH})

find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(Threads)
add_subdirectory(src)
find_package(jsoncpp REQUIRED)

add_executable(connect_test examples/connect_test.cpp)
target_include_directories(connect_test PUBLIC include)
target_link_directories(connect_test PRIVATE src)
target_link_libraries(connect_test csrtsi Threads::Threads)

add_executable(getData_test examples/getData_test.cpp)
target_include_directories(getData_test PUBLIC include)
target_link_directories(getData_test PRIVATE src)
target_link_libraries(getData_test csrtsi Boost::thread jsoncpp)

add_executable(setData_test examples/setData_test.cpp)
target_include_directories(setData_test PUBLIC include)
target_link_directories(setData_test PRIVATE src)
target_link_libraries(setData_test csrtsi Boost::thread jsoncpp)

add_executable(getSetData_test examples/getSetData_test.cpp)
target_include_directories(getSetData_test PUBLIC include)
target_link_directories(getSetData_test PRIVATE src)
target_link_libraries(getSetData_test csrtsi Boost::thread)

add_executable(testPlugin pluginScript/testPlugin.cpp)
target_include_directories(testPlugin PUBLIC include pluginScript)
target_link_directories(testPlugin PRIVATE src)
target_link_libraries(testPlugin csrtsi ${CMAKE_DL_LIBS} Boost::thread)

add_executable(servoj_test examples/servoj_test.cpp)
target_include_directories(servoj_test PUBLIC include)
target_link_directories(servoj_test PRIVATE src)
target_link_libraries(servoj_test csrtsi Threads::Threads)

add_executable(move_test examples/move_test.cpp)
target_include_directories(move_test PUBLIC include)
target_link_directories(move_test PRIVATE src)
target_link_libraries(move_test csrtsi Threads::Threads)

add_executable(control_test examples/control_test.cpp)
target_include_directories(control_test PUBLIC include)
target_link_directories(control_test PRIVATE src)
target_link_libraries(control_test csrtsi Threads::Threads)

include(GNUInstallDirs)
install(TARGETS csrtsi EXPORT csrtsi_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION include)

install(EXPORT csrtsi_targets
  DESTINATION lib/cmake/cs_rtsi
  FILE csrtsiTargets.cmake)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/cs_rtsiConfigVersion.cmake VERSION 0.0.1
  COMPATIBILITY SameMajorVersion)
install(FILES cs_rtsiConfig.cmake
  DESTINATION lib/cmake/cs_rtsi)
