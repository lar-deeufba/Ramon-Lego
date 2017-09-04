cmake_minimum_required (VERSION 2.8)

link_libraries(apriltags)

add_executable(apriltags_demo apriltags_demo.cpp serial_port.cpp serial_port.hpp Funcoes.cpp Funcoes.hpp)
pods_install_executables(apriltags_demo)


